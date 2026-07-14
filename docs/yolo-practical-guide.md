# YOLO 鱼群检测：标注→训练→部署 实操指南

> 📌 **关联文档**: 技术方案 → [`esp-dl-yolo-plan.md`](esp-dl-yolo-plan.md) | 项目代码 → `main/fish_detector.c` (YOLO 部分) | 配置 → `main/Kconfig.projbuild` (SMART_FISHER_DETECTION_YOLO)

## 目录

1. [标注原理：模型到底学到了什么](#1-标注原理)
2. [标注工具安装与使用](#2-标注工具)
3. [标注规范与技巧](#3-标注规范)
4. [数据集组织](#4-数据集)
5. [模型训练](#5-模型训练)
6. [模型转换与部署](#6-模型部署)

---

## 1. 标注原理

### 模型到底学到了什么？

训练 YOLO 就像教一个小孩认鱼：

```
训练前:                             训练后:
┌──────────┐                      ┌──────────┐
│ 输入: 鱼缸照片  │                │ 输入: 鱼缸照片  │
│ 输出: ???       │                │ 输出: 🐟在(120,80) │
│                 │   ──训练──→    │      🐟在(300,150)│
│ (随机权重)      │                │      🐟在(450,60) │
└──────────┘                      └──────────┘
```

标注就是**告诉模型正确答案**。一张含 3 条鱼的照片：

```
标注数据 (label):
  0 0.15 0.20 0.04 0.03    ← "在 15%,20% 的位置有 1 条鱼，宽高约 4%,3%"
  0 0.40 0.35 0.05 0.04    ← "在 40%,35% 的位置有 1 条鱼"
  0 0.70 0.15 0.04 0.03    ← "在 70%,15% 的位置有 1 条鱼"
```

训练时，模型读到照片算出一个预测（比如"我觉得鱼在 14%,21% 的位置"），和标注的正确答案对比，**算出差值 → 反向传播 → 调整内部权重**。重复几千次后，模型学会了"什么样的像素排列 = 鱼"。

### YOLO 标注格式

每个目标框用 5 个数字描述，**全部归一化到 0~1 之间**（除以图片宽高）：

```
class x_center y_center width height
  │      │        │       │      │
  │      │        │       │      └── bbox 高度 / 图片高度
  │      │        │       └─────── bbox 宽度 / 图片宽度
  │      │        └────────────── bbox 中心点 y / 图片高度
  │      └─────────────────────── bbox 中心点 x / 图片宽度
  └────────────────────────────── 类别 ID（0=fish, 1=betta, ...）
```

**原理**：归一化后，不管原始照片是 800×600 还是 1920×1080，标注都是 0~1 之间的小数。训练时模型读不同尺寸的照片，统一 resize 到 224×224，标注也自动等比缩放——这就叫**尺度不变性**。

---

## 2. 标注工具

### 推荐：LabelImg

最轻量的 YOLO 标注工具，Windows 直接跑，不需要 Python 环境。

#### 安装

```bash
# 方法 1: 下载预编译 exe（推荐，免安装）
# https://github.com/HumanSignal/labelImg/releases
# 下载 labelImg.exe → 双击运行

# 方法 2: pip 安装
pip install labelImg
labelImg    # 命令行启动
```

#### 界面操作

```
┌─────────────────────────────────────────────────────┐
│  [Open Dir] [Change Save Dir]        [PascalVOC]  ▽ │  ← 菜单栏
├────────────────────────────┬────────────────────────┤
│                            │  Box Labels:           │
│                            │  ┌──────────────────┐  │
│    鱼缸照片（主窗口）        │  │ fish             │  │  ← 标签列表
│                            │  └──────────────────┘  │
│   ┌──────────────┐        │                        │
│   │    🐟        │        │  File List:            │
│   │  ┌────┐      │        │  tank_001.jpg          │
│   │  │bbox│      │        │  tank_002.jpg          │
│   │  └────┘      │        │  tank_003.jpg  ← 当前  │
│   │         🐟   │        │  ...                   │
│   │    🐟  ┌──┐ │        │                        │
│   │        └──┘ │        │                        │
│   └──────────────┘        │                        │
├────────────────────────────┴────────────────────────┤
│  状态栏: 3 boxes | fish × 3                         │
└─────────────────────────────────────────────────────┘
```

### 完整操作流程

```
第 1 步: 准备照片目录
  mkdir D:\fish_dataset\images
  # 把鱼缸照片全部复制到这里

第 2 步: 启动 LabelImg
  labelImg

第 3 步: 打开照片目录
  点击 [Open Dir] → 选择 D:\fish_dataset\images

第 4 步: 设置标注保存目录
  点击 [Change Save Dir] → 选择 D:\fish_dataset\labels

第 5 步: 切换为 YOLO 格式（关键！）
  点击左侧菜单栏的 [PascalVOC] → 切换为 [YOLO]

第 6 步: 创建标签类别
  在右侧 Box Labels 区域的输入框输入:
    fish
  点 [Save]（这一步只需做一次）

第 7 步: 开始标注每一张照片
  按 W → 进入画框模式
  鼠标左键拖拽 → 画矩形框框住一条鱼
  在弹出的对话框中选 "fish"
  重复直到框完所有鱼
  
第 8 步: 保存
  按 Ctrl+S → 保存标注文件
  按 D → 下一张照片

快捷键速查:
  W          进入画框模式
  Ctrl+S     保存当前标注
  D          下一张
  A          上一张
  Del        删除选中的框
  Ctrl+滚轮   缩放照片
```

### 备选：Roboflow（在线标注）

如果不想装软件，用浏览器标注：

```
1. 打开 https://roboflow.com → 注册免费账号
2. Create New Project → Object Detection
3. Upload Images → 拖入鱼缸照片
4. 进入 Annotate → 画框 → 选类别
5. 标注完导出: Export → YOLOv11 format → Download ZIP
```

优点：网页操作、自动保存、内置数据增强。缺点：免费版有限额。

---

## 3. 标注规范

### 框怎么画？

```
好的标注 ✅                      不好的标注 ❌

┌──────────────────┐           ┌──────────────────┐
│                  │           │  ┌────────────┐   │
│  ┌──────┐        │           │  │🐟    🐟   │   │  ← 两条鱼一个框？
│  │ 🐟   │        │           │  └────────────┘   │     模型会把它当成
│  └──────┘        │           │                    │     一个目标
│          ┌──────┐│           │  ┌┐                │
│          │ 🐟   ││           │  ││🐟              │  ← 框太紧？
│          └──────┘│           │  └┘                │     边界信息丢失
└──────────────────┘           └──────────────────┘
```

| 规则 | 说明 |
|------|------|
| **一条鱼 = 一个框** | 紧贴鱼身，留 2~5 像素余量 |
| **部分遮挡也要标** | 鱼钻进水草后面露出半条？框住露出来的部分 |
| **模糊的鱼也要标** | 快速游动导致的运动模糊 → 框你判断是鱼的位置 |
| **太小看不清？不标** | < 20×20 像素的模糊物体，标了反而误导模型 |
| **边界被裁切的鱼** | 如果鱼只有 1/4 在照片边缘 → 不标 |

### 标注数量建议

```
最少:     200 张（迁移学习，够用但精度有限）
推荐:     500 张（精度和标注成本的最佳平衡）
理想:    1000+ 张（覆盖各种光照、密度、鱼种）
```

200 张就够了——因为我们用的是**迁移学习**（基于 224×224 COCO 预训练的 espdet_pico 模型，已经学会了什么是"边缘"、"纹理"、"物体"等底层特征），只需教会它"这个特定形状 = fish"。不需要从零开始学数千张。

### 数据多样性

```
必须覆盖的场景:
  ☐ 不同时间拍的照片（早/中/晚，光照变化）
  ☐ 不同数量的鱼（0 条 / 1 条 / 3 条 / 满缸）
  ☐ 鱼在不同深度（上浮/中层/底栖）
  ☐ 有饲料在水中（避免把饲料当成鱼）
  ☐ 有气泡（避免把气泡当成鱼）
  ☐ 鱼在角落/边缘
  ☐ 鱼静止不动 vs 正在游动

可以忽略的场景:
  ✗ 换水时拍的照片（水浑浊）
  ✗ 刚开灯时鱼应激躲藏的照片
```

### 标注产出预估

```
标注速度:
  新手: 1~2 分钟/张（每条鱼画一个框）
  熟练: 30 秒/张
  
总耗时:
  200 张 × 1.5 分钟 = 5 小时
  500 张 × 1.5 分钟 = 12.5 小时（可以分批做）
```

---

## 4. 数据集组织

### 目录结构

```
D:\fish_dataset\
├── images/
│   ├── train/
│   │   ├── tank_001.jpg
│   │   ├── tank_002.jpg
│   │   ├── ...
│   │   └── tank_400.jpg      ← 80% 的照片放这里（训练用）
│   └── val/
│       ├── tank_401.jpg
│       ├── tank_402.jpg
│       ├── ...
│       └── tank_500.jpg      ← 20% 的照片放这里（验证用）
├── labels/
│   ├── train/
│   │   ├── tank_001.txt      ← 对应 tank_001.jpg 的标注
│   │   ├── tank_002.txt
│   │   ├── ...
│   │   └── tank_400.txt
│   └── val/
│       ├── tank_401.txt
│       ├── tank_402.txt
│       ├── ...
│       └── tank_500.txt
└── fish_tank.yaml            ← 数据集配置文件
```

### 数据集配置文件

创建 `D:\fish_dataset\fish_tank.yaml`：

```yaml
# 数据集路径（相对于此 yaml 文件的位置）
path: .

# 训练集和验证集
train: images/train
val: images/val

# 类别定义
names:
  0: fish

# 类别数量
nc: 1
```

### 标注文件格式验证

标注完后，随机打开几个 `.txt` 文件确认格式正确：

```
# tank_001.txt — 每行一个目标，5 个空格分隔的数字
# 所有值必须在 0~1 之间！
0 0.1523 0.2031 0.0469 0.0313
0 0.4023 0.3516 0.0508 0.0391
0 0.7031 0.1484 0.0430 0.0273
```

验证脚本 (`check_labels.py`)：

```python
import os
from pathlib import Path

labels_dir = Path("D:/fish_dataset/labels/train")
for txt_file in labels_dir.glob("*.txt"):
    with open(txt_file) as f:
        for line_num, line in enumerate(f, 1):
            parts = line.strip().split()
            if len(parts) != 5:
                print(f"❌ {txt_file.name}: line {line_num} 格式错误")
                continue
            cls, cx, cy, w, h = map(float, parts)
            if not (0 <= cls <= 0 and 0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
                print(f"❌ {txt_file.name}: line {line_num} 数值越界: {parts}")

print("✅ 验证完成")
```

---

## 5. 模型训练

### 环境搭建

```bash
# 1. 创建 Python 虚拟环境
conda create -n espdet python=3.10 -y
conda activate espdet

# 2. 克隆 ESP-Detection 仓库
git clone https://github.com/espressif/esp-detection.git
cd esp-detection

# 3. 安装依赖
pip install -r requirements.txt
# 这会安装 ultralytics, torch, opencv-python 等

# 4. 验证安装
python -c "import ultralytics; print(ultralytics.__version__)"
# 输出: 8.x.x  ← 版本号
```

### 从预训练模型开始训练

```bash
# 一键训练 + 量化 + 导出
python espdet_run.py \
    --class_name fish \
    --dataset "D:/fish_dataset/fish_tank.yaml" \
    --size 224 224 \
    --target "esp32s3" \
    --calib_data "D:/fish_dataset/images/val" \
    --espdl "espdet_pico_fish.espdl" \
    --epochs 50 \
    --batch 16 \
    --device cpu
```

**参数说明**：

| 参数 | 含义 | 为什么这个值 |
|------|------|-------------|
| `--class_name fish` | 要检测的类别名 | 和 yaml 里的 `names` 对应 |
| `--size 224 224` | 模型输入分辨率 | espdet_pico 的标准输入 |
| `--target esp32s3` | 目标芯片 | 决定量化策略 |
| `--calib_data` | 校准数据集 | 用于 INT8 量化校准 |
| `--espdl` | 输出模型文件名 | 最后部署到板子上的文件 |
| `--epochs 50` | 训练轮数 | 迁移学习 50 轮够用 |
| `--batch 16` | 每批图片数 | CPU 训练用 16，GPU 可用 32 |

### 训练过程解读

```
Epoch 1/50:  ← 第一轮，模型几乎瞎猜
  Box Loss:    3.452    ← 位置误差很大
  Class Loss:  0.891    ← 分类误差
  mAP@0.5:     0.123    ← 准确率 12.3%（≈ 蒙的）

Epoch 10/50: ← 第十轮，模型开始"开窍"
  Box Loss:    1.234    ← 位置开始准了
  Class Loss:  0.345    ← 基本能区分鱼和非鱼
  mAP@0.5:     0.678    ← 准确率 67.8%

Epoch 50/50: ← 收敛
  Box Loss:    0.456    ← 位置很准
  Class Loss:  0.089    ← 分类很准
  mAP@0.5:     0.923    ← 准确率 92.3% ✅
```

`mAP@0.5` 是你最需要关心的指标：**> 0.85 = 够用，> 0.90 = 好，> 0.95 = 非常好**。

### 如果 PC 没有 GPU

```bash
# CPU 训练（慢但可行）
python espdet_run.py \
    ... \
    --device cpu \
    --batch 8 \           # 减小 batch size
    --epochs 30 \         # 可以减少轮数
    --workers 4           # 使用 4 个 CPU 核心
```

CPU 训练预计：30 轮约 2~4 小时（取决于 CPU 规格）。

### 训练产出文件

训练完成后，当前目录生成：

```
espdet_pico_fish.espdl       ← INT8 量化好的模型文件（~500KB）
espdet_pico_fish.info        ← 模型元信息
runs/train/exp/weights/
  ├── best.pt                ← 训练中最优的权重（PyTorch 格式）
  └── last.pt                ← 最后一轮的权重
runs/train/exp/results.png   ← 训练曲线图（loss 下降、mAP 上升）
runs/train/exp/val_batch.jpg ← 验证集检测结果可视化
```

---

## 6. 模型部署

### 6.1 硬件端集成

将 `espdet_pico_fish.espdl` 放入项目：

```
smart-fisher/
├── main/
│   ├── models/
│   │   └── espdet_pico_fish.espdl    ← 模型文件
│   ├── fish_detector.c               ← 重写为 YOLO 版本
│   ├── fish_detector.h               ← 不变
│   └── ...
```

### 6.2 CMake 配置（代码已就绪 ✅）

`main/CMakeLists.txt` 已包含条件模型嵌入逻辑——不需要手动修改。启用 `CONFIG_SMART_FISHER_DETECTION_YOLO` 后自动激活：

```cmake
# main/CMakeLists.txt (已实现，无需修改)
if(CONFIG_SMART_FISHER_DETECTION_YOLO)
    set(YOLO_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/espdet_pico_fish.espdl")
    if(EXISTS "${YOLO_MODEL_PATH}")
        target_add_binary_data(${COMPONENT_LIB} "${YOLO_MODEL_PATH}" BINARY)
        message(STATUS "YOLO model embedded: ${YOLO_MODEL_PATH}")
    else()
        message(WARNING "YOLO model file not found. See docs/yolo-practical-guide.md")
    endif()
endif()
```

**Kconfig 配置**也在 `main/Kconfig.projbuild` 中预先定义好了：

- `CONFIG_SMART_FISHER_DETECTION_YOLO` — 切换检测引擎
- `CONFIG_SMART_FISHER_YOLO_CONFIDENCE_THRESHOLD` — 置信度阈值 (默认 0.50)
- `CONFIG_SMART_FISHER_YOLO_IOU_THRESHOLD` — IoU 阈值 (默认 0.45)

通过 `idf.py menuconfig → Smart Fisher Configuration` 修改。

### 6.3 fish_detector.c — 代码已就绪 ✅

YOLO 推理管线的完整实现已在 `main/fish_detector.c` 中（`#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO` 块内），包括：

- JPEG → RGB565 → 224×224 RGB888 预处理 (双线性插值)
- ESP-DL 模型加载 + 推理调用
- NMS 非极大值抑制后处理
- 基于 bbox 质心位移的活跃度追踪

**不需要手动编写任何代码**——启用 Kconfig 选项后，编译系统自动选择 YOLO 引擎。下面是核心代码的结构概览（供理解，非需要手写）：

```c
#include "fish_detector.h"
#include "esp_dl.h"
#include "jpeg_decoder.h"

// ── 模型（编译时嵌入固件）──
extern const uint8_t espdet_pico_fish_espdl[] asm(
    "_binary_models_espdet_pico_fish_espdl_start");
extern const uint8_t espdet_pico_fish_espdl_end[] asm(
    "_binary_models_espdet_pico_fish_espdl_end");

// ── 常量 ──
#define MODEL_INPUT_W    224
#define MODEL_INPUT_H    224
#define CONF_THRESHOLD   0.50f    // 置信度阈值（>50% 才认为有鱼）
#define IOU_THRESHOLD    0.45f    // NMS 的 IoU 阈值

// ── 检测结果 ──
typedef struct {
    float x1, y1, x2, y2;   // bbox 坐标
    float confidence;        // 置信度
} detection_t;

static detection_t prev_detections[20];
static int prev_count = 0;

// ── 模型句柄 ──
static void *model_handle = NULL;
static uint8_t *model_input = NULL;   // 224×224×3 RGB888

esp_err_t fish_detector_init(void)
{
    // 1. 加载模型
    size_t model_size = espdet_pico_fish_espdl_end -
                        espdet_pico_fish_espdl;
    espdl_handle_t cfg = {
        .model_data = (void *)espdet_pico_fish_espdl,
        .model_size = model_size,
    };
    model_handle = espdl_create_model(&cfg);
    if (!model_handle) return ESP_FAIL;

    // 2. 分配输入 buffer（PSRAM）
    model_input = heap_caps_malloc(
        MODEL_INPUT_W * MODEL_INPUT_H * 3,
        MALLOC_CAP_SPIRAM);
    if (!model_input) return ESP_ERR_NO_MEM;

    ESP_LOGI("fish-detector", "YOLO model loaded: %u bytes", model_size);
    return ESP_OK;
}

esp_err_t fish_detector_analyze(const uint8_t *jpeg_data, size_t jpeg_len,
                                fish_result_t *result)
{
    if (!jpeg_data || !result) return ESP_ERR_INVALID_ARG;

    memset(result, 0, sizeof(*result));
    result->activity = "unknown";

    int64_t t0 = esp_timer_get_time();

    // ── 步骤 1: JPEG → RGB565 (1/8 scale, 100×75) ──
    uint16_t *rgb565 = NULL;
    int jpeg_w = 0, jpeg_h = 0;
    if (decode_jpeg_to_rgb565(jpeg_data, jpeg_len,
                              &rgb565, &jpeg_w, &jpeg_h) != ESP_OK) {
        return ESP_FAIL;
    }

    // ── 步骤 2: RGB565 → 224×224 RGB888 (双线性插值) ──
    resize_to_rgb888(rgb565, jpeg_w, jpeg_h,
                     model_input, MODEL_INPUT_W, MODEL_INPUT_H);
    free(rgb565);

    // ── 步骤 3: ESP-DL 推理 ──
    int64_t t1 = esp_timer_get_time();
    espdl_tensor_t input = {
        .data   = model_input,
        .shape  = {1, MODEL_INPUT_H, MODEL_INPUT_W, 3},  // NHWC
        .dtype  = ESPDL_DTYPE_UINT8,
    };
    espdl_tensor_t *output = espdl_model_run(model_handle, &input, 1);
    int64_t t2 = esp_timer_get_time();

    if (!output) {
        ESP_LOGE("fish-detector", "Model inference failed");
        return ESP_FAIL;
    }

    // ── 步骤 4: NMS 后处理 ──
    detection_t detections[20];
    int det_count = 0;
    nms_yolo(output, CONF_THRESHOLD, IOU_THRESHOLD,
             MODEL_INPUT_W, MODEL_INPUT_H,
             detections, &det_count, 20);

    // ── 步骤 5: 活跃度评估（对比上一帧 bbox） ──
    int motion_score = compute_motion_score(
        detections, det_count,
        prev_detections, prev_count);

    // ── 保存当前帧 ──
    memcpy(prev_detections, detections, det_count * sizeof(detection_t));
    prev_count = det_count;

    // ── 步骤 6: 填充结果 ──
    result->fish_count   = det_count;
    result->motion_score = motion_score;
    result->activity     = (motion_score < 5)  ? "calm"
                         : (motion_score < 15) ? "moderate"
                         :                       "active";

    int64_t t3 = esp_timer_get_time();
    ESP_LOGI("fish-detector",
             "🐟 %d fish, %s (score=%d%%) "
             "[prep=%lldms, infer=%lldms, post=%lldms]",
             det_count, result->activity, motion_score,
             (long long)((t1-t0)/1000),
             (long long)((t2-t1)/1000),
             (long long)((t3-t2)/1000));

    return ESP_OK;
}
```

### 6.4 编译 & 烧录

```bash
# 清理重编译（加了新依赖）
idf.py fullclean
idf.py build

# 预期固件增大 ~600KB（模型 500KB + ESP-DL 库 100KB）
# 16MB Flash 完全够用

# 烧录
idf.py -p COM8 flash monitor
```

---

## 7. 踩坑预警

| 常见问题 | 原因 | 解决方案 |
|---------|------|---------|
| 模型加载失败 | ESP-DL 版本不兼容 | 用 esp-detection 仓库自带的 esp-dl 版本 |
| 推理结果全是 0 | 输入数据格式不对 | 确认是 NHWC（HWC 顺序），不是 NCHW |
| 推理极慢 (>2 秒) | 权重被放到了慢速 SPI Flash | 推理时将权重加载到 PSRAM |
| 检测框位置不准 | 缩放算法有问题 | 用双线性插值，不要用最近邻 |
| 编译报 `esp_dl.h` 找不到 | 没加 `esp_dl` 到 PRIV_REQUIRES | 参考上面的 CMake 配置 |
| OOM (Out of Memory) | 工作区 buffer 太大 | 减小 `MODEL_INPUT_W/H`，或启用 PSRAM |

---

## 8. 快速检查清单

### 准备工作

- [ ] 已从鱼缸拍了 **200+ 张照片**（不同时间、不同鱼数）
- [ ] 已安装 LabelImg（或注册 Roboflow）
- [ ] 已搭建 ESP-Detection Python 环境
- [ ] 已理解 YOLO 标注格式：`class cx cy w h`
- [ ] 知道怎么打开串口看 ESP32 日志

### 训练完成后的部署

- [ ] `espdet_pico_fish.espdl` 已放入 `main/models/`
- [ ] `idf_component.yml` 中 `espressif/esp-dl` 依赖已启用
- [ ] `idf.py menuconfig → Smart Fisher Configuration → Fish Detection Method → YOLO Object Detection`
- [ ] `idf.py fullclean && idf.py build` 编译通过
- [ ] 固件烧录后串口日志显示 "YOLO model loaded successfully"
- [ ] MQTTX 订阅 `smart-fisher/+/fish_status` 验证检测结果

### 实际工作流

```
Day 1~2:  标注 200~500 张照片
Day 3:    训练模型（CPU 4h / GPU 1h）
Day 3~4:  放置模型 + menuconfig 切换 + 编译测试
Day 5:    在真实鱼缸上验证 + 与帧差法对比
```

### 关键文件速查

| 文件 | 作用 |
|------|------|
| `main/fish_detector.c` | YOLO 推理代码 (已实现, `#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO`) |
| `main/Kconfig.projbuild` | 检测引擎选择 + 置信度/IoU 阈值配置 |
| `main/CMakeLists.txt` | 条件模型嵌入 (`target_add_binary_data`) |
| `main/idf_component.yml` | `espressif/esp-dl` 依赖声明 |
| `sdkconfig.defaults` | 默认帧差法，YOLO 注释掉待启用 |
