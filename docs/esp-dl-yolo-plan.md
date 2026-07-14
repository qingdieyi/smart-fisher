# ESP-DL + YOLO 鱼群检测方案

## 概述

用 YOLO 目标检测模型替换现有的帧差法（`fish_detector.c`），在 ESP32-S3 芯片内部完成神经网络推理。不依赖云端 GPU，不消耗网络流量，推理在本地完成。

> 📖 **配套实操指南**：[`yolo-practical-guide.md`](yolo-practical-guide.md) — 标注工具使用、YOLO 格式原理、训练命令详解、模型部署代码示例。

---

## 1. 技术选型

### 为什么选 ESP-Detection（基于 ESP-DL + YOLOv11）？

| 方案 | 推理时间 (ESP32-S3) | 开发难度 | 生态成熟度 |
|------|---------------------|---------|-----------|
| **ESP-Detection (espdet_pico)** | **~120ms** | ⭐ 低 — 一行命令训练+部署 | 2025.04 首发，乐鑫官方维护 |
| ESP-DL + YOLOv11n 手动转换 | ~400-600ms | ⭐⭐⭐ 高 — 需要手写前后处理 | 成熟 |
| ESP-DL + YOLOv26n | ~2100ms | ⭐⭐ 中 | 2026 年新支持 |
| TensorFlow Lite Micro | ~800ms | ⭐⭐ 中 | 生态最广 |

**推荐：ESP-Detection `espdet_pico`**

理由：
- **pico 模型专为 MCU 设计**：仅 0.36M 参数（MobileNet 的 1/10）
- **115~126ms/帧** = 约 8 FPS，鱼缸场景绰绰有余
- **端到端工具链**：PyTorch 训练 → 一键量化 → `.espdl` 模型文件
- **YOLOv11 架构**：你要求的 YOLO 系列，社区最活跃

### 模型规格

```
模型名称:    espdet_pico
基础架构:    YOLOv11-nano 魔改（专为 MCU 优化）
参数量:      0.36M
输入尺寸:    224×224×3 (或 160×288)
精度:        mAP@0.5:0.95 ≈ 69.9（COCO 基准）
推理耗时:    115~126ms @ ESP32-S3 240MHz
模型大小:    ~500KB (INT8 量化后)
量化方式:    INT8 权重量化
```

---

## 2. 和现有架构的集成

### 替换范围（最小改动）

```
当前架构                          新架构
─────────────────────────────    ─────────────────────────────
fish_detector.c                  fish_detector.c
  ├─ JPEG 解码 (esp_jpeg)  ✅      ├─ JPEG 解码 (esp_jpeg)    ✅ 保留
  ├─ RGB565 → 灰度         ✅      ├─ RGB565 → 224×224 RGB    🆕 缩放
  ├─ 帧差法                ❌      ├─ ESP-DL YOLO 推理         🆕 替换
  ├─ 连通域分析            ❌      ├─ NMS + 后处理              🆕 替换
  └─ 输出 fish_result_t    ✅      └─ 输出 fish_result_t       ✅ 保留 API
```

**关键设计**：`fish_result_t` 结构体和 `fish_detector_analyze()` 的 API 签名**保持不变**。上层 `main.c` 的调用代码一行不用改。

### 数据流

```
  OV5640 拍照 (800×600 JPEG)
       │
       ▼
  esp_jpeg 解码 → RGB565 (100×75, 1/8 scale)
       │
       ▼
  双线性插值缩放 → 224×224 RGB888      ← 🆕 软件缩放
       │
       ▼
  ESP-DL 推理 (espdl::Model::run)      ← 🆕 模型推理
       │
       ├─ 输出: bounding boxes + class + confidence
       │   [{class: fish, conf: 0.92, x: 50, y: 30, w: 40, h: 25},
       │    {class: fish, conf: 0.87, x: 120, y: 80, w: 35, h:22},
       │    {class: fish, conf: 0.79, x: 200, y: 55, w: 30, h:20}]
       │
       ▼
  NMS (非极大值抑制) + 后处理
       │
       ├─ fish_count = bbox 数量 = 3
       ├─ 跟踪相邻帧 bbox 的质心位移 → 运动分数
       └─ 活跃度评估 (calm/moderate/active)
       │
       ▼
  fish_result_t {fish_count: 3, motion_score: 65, activity: "active"}
       │
       ▼
  MQTT 上报 (不变)
```

---

## 3. 实现步骤

### Phase A: 环境准备（PC 端，1 天）

```bash
# 1. 克隆 ESP-Detection 仓库
git clone https://github.com/espressif/esp-detection.git
cd esp-detection

# 2. 创建 conda 环境
conda create -n espdet python=3.10 -y
conda activate espdet

# 3. 安装依赖
pip install ultralytics torch torchvision opencv-python
pip install esp-dl-toolkit  # 乐鑫模型转换工具
```

### Phase B: 数据集准备（1-3 天）

需要在鱼缸场景标注数据。核心思路：**从鱼缸视频中截取帧，标注鱼的位置**。

```
数据集结构:
datasets/
├── images/
│   ├── train/
│   │   ├── tank_001.jpg    # 含 3 条鱼
│   │   ├── tank_002.jpg    # 含 2 条鱼
│   │   └── ...             # 建议 200-500 张
│   └── val/
│       ├── tank_101.jpg
│       └── ...             # 建议 50-100 张
└── labels/
    ├── train/
    │   ├── tank_001.txt    # YOLO 格式标注
    │   └── ...
    └── val/
        └── ...
```

**YOLO 标注格式**（每行一个目标）：
```
# tank_001.txt — 对应 tank_001.jpg
# class x_center y_center width height (归一化到 0~1)
0 0.25 0.35 0.08 0.06
0 0.60 0.45 0.07 0.05
0 0.80 0.30 0.09 0.07
```

**标注工具**：
- [LabelImg](https://github.com/HumanSignal/labelImg) — 免费，YOLO 格式输出
- [Roboflow](https://roboflow.com) — 在线标注 + 数据增强，免费 tier 够用

**标注策略**：
- 类别：只标注 `fish` 一个类别（单类检测最简单）
- 数量：200~500 张足够（迁移学习，不是从头训练）
- 多样性：不同时间（早/晚）、不同鱼群密度、有无饲料
- 数据增强（自动）：随机裁剪、翻转、亮度变化

### Phase C: 模型训练 + 量化（PC 端，半天）

ESP-Detection 提供了 `espdet_run.py` 一键脚本：

```bash
python espdet_run.py \
    --class_name fish \
    --dataset "cfg/datasets/fish_tank.yaml" \   # 数据集配置
    --size 224 224 \                             # 输入分辨率
    --target "esp32s3" \                          # 目标芯片
    --calib_data "datasets/images/val" \          # 校准数据集（用于 INT8 量化）
    --espdl "espdet_pico_fish.espdl" \            # 输出模型文件
    --epochs 50 \                                 # 训练轮数
    --batch 16
```

输出文件：
```
espdet_pico_fish.espdl    ← 量化后的模型（~500KB）
espdet_pico_fish.info     ← 模型信息（输入/输出 tensor 名、维度）
```

### Phase D: 集成到项目（ESP32-S3 端，2-3 天）

#### 步骤 1：添加 ESP-DL 依赖

修改 `main/idf_component.yml`：

```yaml
dependencies:
  espressif/esp32-camera:
    version: "^2.0.0"
  espressif/esp-dl:
    version: "^3.3.0"
```

或通过 CMake 直接依赖：
```cmake
# main/CMakeLists.txt
PRIV_REQUIRES esp_wifi nvs_flash esp_driver_gpio esp32-camera esp_timer esp_mqtt esp_jpeg esp_dl
```

#### 步骤 2：将模型文件放入项目

```bash
# 将 .espdl 模型文件放到 Flash 分区
mkdir -p main/models
cp espdet_pico_fish.espdl main/models/

# 或者放到 SPIFFS/LittleFS 分区，运行时加载
```

#### 步骤 3：重写 `fish_detector.c`

新的实现架构：

```c
// fish_detector.c — YOLO 版本

#include "fish_detector.h"
#include "esp_dl.h"           // ESP-DL 推理 API
#include "jpeg_decoder.h"     // JPEG 解码 (保留)

// ── 模型全局句柄 ──
static espdl_handle_t model = NULL;

// ── 缩放缓冲区 (PSRAM) ──
static uint8_t *rgb888_buf = NULL;    // 224×224×3 = 150KB

esp_err_t fish_detector_init(void)
{
    // 1. 加载模型（从 Flash 或文件系统）
    espdl_model_config_t cfg = {
        .model_data = espdet_pico_fish_espdl,   // 模型二进制数据
        .model_size = espdet_pico_fish_espdl_len,
        .mem_pool = NULL,       // 使用默认内存池
    };
    model = espdl_model_init(&cfg);

    // 2. 分配缩放缓冲区
    rgb888_buf = heap_caps_malloc(224 * 224 * 3, MALLOC_CAP_SPIRAM);

    return (model && rgb888_buf) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t fish_detector_analyze(const uint8_t *jpeg_data, size_t jpeg_len,
                                fish_result_t *result)
{
    // ── 步骤 1: JPEG 解码 → RGB565 (和之前一样) ──
    // ... esp_jpeg_decode → rgb565_buf (100×75) ...

    // ── 步骤 2: 缩放至 224×224 RGB888 (双线性插值) ──
    resize_rgb565_to_rgb888(rgb565_buf, 100, 75,
                            rgb888_buf, 224, 224);

    // ── 步骤 3: ESP-DL 推理 ──
    espdl_tensor_t input = {
        .data = rgb888_buf,
        .shape = {1, 224, 224, 3},   // NHWC 格式
        .dtype = ESPDL_DTYPE_UINT8,
    };
    espdl_tensor_t *outputs = espdl_model_run(model, &input, 1);

    // ── 步骤 4: 后处理 ──
    // outputs[0] → bounding boxes [N, 6]: [x1, y1, x2, y2, conf, class]
    // outputs[1] → 或直接回归格式（取决于模型输出头）

    int fish_count = 0;
    float total_confidence = 0.0f;

    // NMS + 阈值过滤
    bbox_t *detections = nms(outputs, CONF_THRESHOLD, IOU_THRESHOLD, &fish_count);

    // ── 步骤 5: 活跃度评估 ──
    int motion_score = compute_motion_from_bboxes(
        detections, fish_count,
        prev_detections, prev_fish_count);

    const char *activity = get_activity_label(motion_score);

    // ── 保存当前检测结果（用于下一帧对比） ──
    save_prev_detections(detections, fish_count);

    // ── 输出 ──
    result->fish_count = fish_count;
    result->motion_score = motion_score;
    result->activity = activity;

    return ESP_OK;
}
```

#### 步骤 4：活跃度检测改进

YOLO 模型给出每帧的 bounding box，可以更精确地判断活跃度：

```c
// 基于 bbox 质心位移的活跃度算法
static int compute_motion_from_bboxes(
    const bbox_t *current, int cur_count,
    const bbox_t *previous, int prev_count)
{
    if (prev_count == 0) return 0;  // 第一帧

    int total_displacement = 0;

    // 匈牙利算法匹配相邻帧的同一目标
    // 简化版：找最近邻匹配
    for (int i = 0; i < cur_count; i++) {
        float cx = (current[i].x1 + current[i].x2) / 2.0f;
        float cy = (current[i].y1 + current[i].y2) / 2.0f;

        // 找上一帧最近的 bbox
        float min_dist = INFINITY;
        for (int j = 0; j < prev_count; j++) {
            float px = (previous[j].x1 + previous[j].x2) / 2.0f;
            float py = (previous[j].y1 + previous[j].y2) / 2.0f;
            float dist = sqrtf((cx-px)*(cx-px) + (cy-py)*(cy-py));
            if (dist < min_dist) min_dist = dist;
        }
        total_displacement += (int)min_dist;
    }

    // 映射到 0-100 分数
    int score = (total_displacement * 100) / (cur_count * 100);
    return (score > 100) ? 100 : score;
}
```

### Phase E: 测试和优化（1-2 天）

```
# 1. 编译测试
idf.py build
# 预期: 固件增加约 600KB (模型 ~500KB + ESP-DL 库 ~100KB)

# 2. 烧录测试
idf.py -p COM8 flash monitor

# 3. 观察日志
I (12345) fish-detector: YOLO model loaded (0.36M params, 224x224)
I (23456) fish-detector: Inference: 120ms, 3 fish detected (conf: 0.92, 0.87, 0.79)
I (23456) fish-detector: 🐟 3 fish, activity=active (score=65%)

# 4. 内存监控
I (34567) smart-fisher: Status — free_heap: 4234567, uptime: 3600s
```

---

## 4. 内存预算

ESP32-S3-CAM R16N8 的内存分配：

```
8 MB PSRAM:
├── 摄像头帧缓冲 (双缓冲)    ~30KB × 2 = ~60KB
├── RGB565 解码输出 (100×75)            ~15KB
├── RGB888 模型输入 (224×224×3)         ~150KB
├── ESP-DL 推理工作区                     ~200KB
├── 模型权重 (如果放 PSRAM)               ~500KB
├── 系统堆 + other                        ~1MB
└── 剩余可用                              ~6MB  ← 绰绰有余

512KB Internal SRAM:
├── FreeRTOS 任务栈                         ~40KB
├── 系统保留                                ~100KB
├── .bss / .data (全局变量)                 ~50KB
└── 剩余                                    ~322KB
```

**关键**：模型权重放在 **Flash** 中（通过 `__attribute__((section(".rodata")))`），推理时按需加载到 PSRAM，不占用宝贵的内部 SRAM。

---

## 5. 性能预估

| 阶段 | 耗时 | 说明 |
|------|------|------|
| JPEG 解码 (1/8 scale) | ~50ms | esp_jpeg，硬件加速 |
| 缩放到 224×224 | ~15ms | 双线性插值，纯 CPU |
| YOLO 推理 | **~120ms** | espdet_pico，INT8 量化 |
| NMS + 后处理 | ~10ms | 轻量级 NMS |
| **总计** | **~195ms** | 60s 拍照周期绰绰有余 |

实际 FPS：5 FPS 以上，完全满足定时拍照 + 检测场景。

---

## 6. 与帧差法的切换策略

建议保留帧差法作为 Fallback：

```c
// main/Kconfig.projbuild
choice FISH_DETECTION_METHOD
    prompt "Fish Detection Method"
    default FISH_DETECTION_FRAME_DIFF
    config FISH_DETECTION_YOLO
        bool "YOLO (ESP-DL)"
    config FISH_DETECTION_FRAME_DIFF
        bool "Frame Differencing"
endchoice
```

```c
// fish_detector.c
esp_err_t fish_detector_analyze(...)
{
#ifdef CONFIG_FISH_DETECTION_YOLO
    return yolo_analyze(jpeg_data, jpeg_len, result);
#else
    return frame_diff_analyze(jpeg_data, jpeg_len, result);  // 现有的
#endif
}
```

这样可以在 menuconfig 中一键切换，出问题时退回到帧差法。

---

## 7. 开发时间线

| 阶段 | 内容 | 预计耗时 |
|------|------|---------|
| A | 环境准备 + ESP-Detection 跑通 demo | 1 天 |
| B | 鱼缸数据采集 + 标注 (200-500 张) | 1-3 天 |
| C | 迁移学习训练 + INT8 量化 | 0.5 天 |
| D | 集成到项目 + 前后处理代码 | 2-3 天 |
| E | 测试 + 优化 + Fallback 机制 | 1-2 天 |
| **合计** | | **5-10 天** |

---

## 8. 参考资源

- [ESP-Detection GitHub](https://github.com/espressif/esp-detection) — 一行命令训练+部署
- [ESP-DL 文档](https://docs.espressif.com/projects/esp-dl/) — API 参考
- [ESP-Model Zoo](https://github.com/espressif/esp-model-zoo) — 预训练模型仓库
- [Ultralytics YOLOv11](https://docs.ultralytics.com/models/yolo11/) — 训练框架
- [LabelImg 标注工具](https://github.com/HumanSignal/labelImg)
- [Roboflow 在线标注](https://roboflow.com)

---

## 附录：为什么不选其他方案

### 不选 ESP-DL 手动转换的原因
ESP-Detection 已经包装好了 YOLOv11 → ESP-DL 的完整流程，手写前后处理没有额外收益。

### 不选 TensorFlow Lite Micro 的原因
- YOLO 在 TFLM 上的前后处理比 ESP-DL 复杂得多
- 无法利用 ESP32-S3 的 SIMD 加速
- Flash 占用更大 (~500KB vs ~200KB)

### 不选 YOLOv26n 的原因
- 推理 2.1 秒/帧，太慢（鱼缸不需要 80 类 COCO）
- espdet_pico 0.12 秒/帧，快 17 倍
- 鱼缸只需检测 "fish" 一个类别，pico 完全够用
