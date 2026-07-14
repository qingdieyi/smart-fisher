# 鱼缸智能监控系统 — 开发方案

## 1. 需求概述

| 项目 | 内容 |
|------|------|
| **硬件** | ESP32-S3-CAM R16N8 + OV5640 摄像头 + DS18B20 温度传感器 |
| **功能** | 采集鱼缸水温 + 拍摄鱼缸图片，识别鱼的数量和活跃度 |
| **上报** | 通过 WiFi 连接网络，数据上报到公共 MQTT Broker |

---

## 2. 硬件连接设计

### 2.1 DS18B20 温度传感器接线

DS18B20 为三脚 TO-92 封装，使用 1-Wire 协议：

| DS18B20 引脚 | 连接目标 | 说明 |
|-------------|---------|------|
| VCC (红) | ESP32-S3-CAM 3.3V | 供电 3.0V~5.5V，开发板提供 3.3V |
| GND (黑) | ESP32-S3-CAM GND | 共地 |
| DATA (黄) | GPIO1 | 1-Wire 数据线，**必须加 4.7kΩ 上拉电阻到 3.3V** |

> **⚠ GPIO 冲突已修复**：最初使用 GPIO4，但 GPIO4 也是摄像头 SCCB SDA (I²C) 引脚。1-Wire 和 I²C 共用 GPIO4 导致信号互相干扰，I²C 读/写间歇失败 + DS18B20 CRC 错误。现已改为 **GPIO1**。备选空闲引脚：GPIO2、GPIO14、GPIO21。

### 2.2 OV5640 摄像头

OV5640 已集成在开发板上，通过 CSI/DVP 接口连接，无需额外接线。关键引脚映射（由 ESP-IDF camera 驱动自动管理）：

```
SCCB_SDA → GPIO40
SCCB_SCL → GPIO39
VSYNC    → GPIO6
HREF     → GPIO7
PCLK     → GPIO15
XCLK     → GPIO16
D0-D7    → GPIO47,48,45,13,12,11,17,41
RESET    → GPIO46
PWDN     → -1 (unused)
```

---

## 3. 软件架构

```
┌──────────────────────────────────────────────────────┐
│                    app_main()                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ NVS Init │→│WiFi Init │→│Task Creation      │   │
│  └──────────┘  └──────────┘  └──────┬───────────┘   │
│         ┌────────────────────────────┼───┐           │
│         ▼                            ▼   ▼           │
│  ┌──────────────┐  ┌────────────────────┐           │
│  │ Temp Task    │  │ Camera + AI Task   │           │
│  │ (DS18B20)   │  │ (OV5640 + Detect)  │           │
│  └──────┬───────┘  └────────┬───────────┘           │
│         │                   │                        │
│         └─────────┬─────────┘                        │
│                   ▼                                  │
│          ┌────────────────┐                          │
│          │  MQTT Publisher │                          │
│          │  (esp-mqtt)    │                          │
│          └───────┬────────┘                          │
└──────────────────┼───────────────────────────────────┘
                   │
        ┌──────────▼──────────┐
        │  Public MQTT Broker │
        │  (broker.emqx.io)   │
        └──────────┬──────────┘
                   │
        ┌──────────▼──────────┐
        │  Cloud / Mobile     │
        │  Dashboard          │
        └─────────────────────┘
```

### 3.1 FreeRTOS 任务划分

| 任务名 | 栈大小 | 优先级 | 周期 | 职责 | 状态 |
|--------|--------|--------|------|------|------|
| `temp_task` | 4096 | 3 | 每 5 秒 | 读取 DS18B20，串口输出温度 | ✅ 已实现 |
| `camera_capture_task` | 8192 | 2 | 每 60 秒 | 拍照 → 鱼群检测 → MQTT 上报 | ✅ 已实现 |
| `mqtt_task` | ESP-MQTT 内部任务 | 自动 | 事件驱动 | MQTT 连接管理 + 消息发布 | ✅ 已实现 |

> **注意**：实际实现与原始设计有差异。WiFi 重连逻辑在事件回调 `wifi_event_handler()` 中处理（而非独立任务），`main.c` 的主循环作为空闲任务运行（而非独立的 wifi_reconnect_task）。

### 3.2 实际代码结构

```
main/
├── CMakeLists.txt         # PRIV_REQUIRES: esp_wifi nvs_flash esp_driver_gpio
├── main.c                 # 入口 + WiFi STA(WPA2-PSK) + 温度任务
├── ds18b20.h / ds18b20.c  # DS18B20 纯 GPIO 位拆裂驱动（无外部依赖）
├── Kconfig.projbuild       # menuconfig: WiFi SSID + password
├── wifi_eap_fast_main.c    # [已废弃] 原始 EAP-FAST 示例 — 仅作参考
└── ca.pem / pac_file.pac / server.*  # [未使用] EAP-FAST 证书
```

> **实现决策**：DS18B20 驱动采用手动位拆裂（bit-bang）实现，而非使用 ESP-IDF Component Registry 的 `espressif/onewire_bus` + `espressif/ds18b20` 组件。原因：避免外部网络依赖，学习 1-Wire 协议底层细节。内存占用更小，无额外组件开销。

---

## 4. 核心模块设计

### 4.1 DS18B20 温度采集 ✅ 已实现

**实际实现** (`ds18b20.h` / `ds18b20.c`)：

```c
// ds18b20.h — 公开 API
esp_err_t ds18b20_init(gpio_num_t pin);                    // 初始化：GPIO配置 + 设备检测 + 12-bit配置
esp_err_t ds18b20_read_temperature(gpio_num_t pin, float *temp_c); // 读取温度（°C）
```

- **无外部依赖**：纯 GPIO 位拆裂实现，不依赖 `onewire_bus`/`ds18b20` 组件
- 12-bit 分辨率（0.0625°C），转换时间 750ms
- CRC-8 校验（Dallas 多项式 0x8C）
- 使用 `portDISABLE_INTERRUPTS()` 保护时序关键段
- 错误码：`ESP_OK` / `ESP_ERR_TIMEOUT` / `ESP_ERR_INVALID_CRC` / `ESP_ERR_NOT_FOUND`
- GPIO4 开启内部上拉（~45kΩ），但**仍需外部 4.7kΩ 上拉电阻**

**测温流程**：
```
RESET → SKIP_ROM → CONVERT_T(0x44) → 等 750ms → RESET → SKIP_ROM
→ READ_SCRATCHPAD(0xBE) → 读 9 字节 → CRC 校验 → raw × 0.0625 = °C
```

### 4.2 OV5640 摄像头拍照

```c
// camera_handler.h
typedef struct {
    uint8_t *jpeg_buf;     // JPEG 图像缓冲区
    size_t jpeg_len;       // JPEG 图像大小
    int64_t timestamp_ms;  // 拍摄时间戳
    int width, height;     // 图像分辨率
} camera_frame_t;

esp_err_t camera_init(void);
esp_err_t camera_capture(camera_frame_t *frame);
void camera_frame_release(camera_frame_t *frame);
```

- 使用 `esp32-camera` 组件驱动 OV5640
- 拍照分辨率：**SVGA (800×600)**，平衡清晰度与上传带宽
- JPEG 质量：建议 15~20，单帧约 10~30KB
- 拍照间隔：**5 分钟**
- 启用 LED 闪光灯补光（GPIO 21 控制）

### 4.3 鱼群检测算法

```c
// fish_detector.h
typedef enum {
    ACTIVITY_IDLE,      // 几乎不动
    ACTIVITY_NORMAL,    // 正常游动
    ACTIVITY_ACTIVE,    // 活跃游动
} fish_activity_t;

typedef struct {
    int fish_count;           // 估计鱼的数量
    fish_activity_t activity; // 活跃度
    float motion_score;       // 运动分数 (0.0~1.0)
} fish_status_t;
```

**MVP 方案：帧差法运动检测**

```
当前帧 → 灰度化 → 高斯模糊
上一帧 → 灰度化 → 高斯模糊
         ↓
    帧差法 (absdiff) → 二值化 → 轮廓检测
         ↓
    运动区域数量 ÷ 膨胀形态学处理 → motion_score
    连通域数量近似 → fish_count (粗略估计)
         ↓
    fish_count == 0 ? 报告异常 : 判定 activity 等级
```

实现路径：

| 阶段 | 方法 | 说明 |
|------|------|------|
| **MVP** | 帧差法 (Frame Differencing) | 轻量级，可在 ESP32-S3 上实时运行。通过前后帧对比检测运动区域，以连通域数量估计鱼的数量，以运动像素比例估计活跃度 |
| **进阶** | TensorFlow Lite Micro + ESP-DL | 训练一个轻量级目标检测模型（如 MobileNet-SSD），可较准确地数鱼，但需要标注数据集和训练 |
| **云端** | 图片上传到 HTTP 服务器 | 将图片上传到云端由更强的模型分析，ESP32-S3 只负责采集和上传 |

> MVP 阶段推荐帧差法，它无需训练模型，开发周期短，资源占用低。

### 4.4 MQTT 上报

```c
// mqtt_publisher.h
esp_err_t mqtt_init(const char *broker_uri);
esp_err_t mqtt_publish_temperature(float temp_c);
esp_err_t mqtt_publish_fish_status(const fish_status_t *status);
esp_err_t mqtt_publish_image(const uint8_t *jpeg_data, size_t len, int64_t timestamp);
```

**Topic 设计**：

| Topic | QoS | Payload | 频率 |
|-------|-----|---------|------|
| `fish_tank/{device_id}/temperature` | 1 | `{"temp_c":25.5,"ts":1720000000}` | 每 5s |
| `fish_tank/{device_id}/fish_status` | 1 | `{"count":5,"activity":"active","motion":0.73,"ts":1720000000}` | 每 5min |
| `fish_tank/{device_id}/image` | 1 | JPEG binary + 首部 JSON `{"ts":...,"size":...,"w":800,"h":600}` | 每 5min |
| `fish_tank/{device_id}/status` | 0 | `{"online":true,"uptime_s":3600,"free_heap":123456}` | 每 60s (LWT) |

**公共 MQTT Broker**（任选其一）：

| Broker | URI | 端口 |
|--------|-----|------|
| EMQX 公共 Broker | `mqtt://broker.emqx.io` | 1883 |
| HiveMQ 公共 Broker | `mqtt://broker.hivemq.com` | 1883 |
| Eclipse Mosquitto | `mqtt://test.mosquitto.org` | 1883 |

- `device_id` 使用 ESP32 MAC 地址的后 6 位十六进制，确保唯一性
- 图片分段上传（MQTT 最大 payload 通常 256KB，单张 JPEG 约 10~30KB，可直接发送）
- 如果图片超过 128KB，改用 HTTP POST 上传到对象存储

### 4.5 WiFi 连接 ✅ 已实现

**实际实现** (`main.c`)：

```c
// WiFi STA 模式 — WPA2-PSK
#define WIFI_SSID      "yang"
#define WIFI_PASSWORD  "yang123456"
#define WIFI_MAX_RETRY 5

static void wifi_init_sta(void);      // 初始化 TCP/IP + WiFi 硬件 + 事件回调
static void wifi_event_handler(...);  // 处理 STA_START / DISCONNECTED / GOT_IP
```

- WPA2-PSK 认证（非原计划的 WPA2-ENT）
- 断线自动重连（最多 5 次，超过后 `esp_restart()` 重启）
- FreeRTOS EventGroup 用于 WiFi 就绪同步（`WIFI_CONNECTED_BIT`）
- WiFi 凭证同时支持 `#define`（硬编码默认值）和 `Kconfig.projbuild`（menuconfig 覆盖）
- PMF (Protected Management Frames) 启用但非强制

**关键设计**：
- 重连逻辑在事件回调中处理（非独立任务）—— 比创建独立 `wifi_reconnect_task` 更轻量
- `temperature_task` 启动后先阻塞等待 `WIFI_CONNECTED_BIT`，确保有网才读传感器

---

## 5. sdkconfig 关键配置

**当前已配置** (`sdkconfig.defaults`)：

```ini
# WiFi — WPA2-PSK
CONFIG_EXAMPLE_WIFI_SSID="yang"
CONFIG_EXAMPLE_WIFI_PASSWORD="yang123456"
CONFIG_EXAMPLE_WPA2_ENTERPRISE=n
```

**后续阶段待开启**：

```ini
# MQTT
# CONFIG_BROKER_URI="mqtt://broker.emqx.io"

# Watchdog
# CONFIG_ESP_TASK_WDT=y
# CONFIG_ESP_TASK_WDT_TIMEOUT_S=10
```

> **注意**：ESP-IDF v6.0 中 `driver/` 头文件拆分为独立组件（如 `esp_driver_gpio`）。添加 `driver/xxx.h` include 时，必须在 `main/CMakeLists.txt` 的 `PRIV_REQUIRES` 中添加对应的 `esp_driver_xxx`。

---

## 6. 开发阶段划分

### 第一阶段：基础搭建（1~2天）✅ 已完成

- [x] 已有：CMake 工程、WiFi EAP-FAST 示例框架
- [x] 将 WiFi 从 WPA2-ENT 改为 WPA2-PSK（家用 WiFi 模式），SSID/密码通过 `#define` 和 Kconfig 配置
- [x] 搭建 FreeRTOS 任务框架（temperature_task + 主空闲循环）
- [x] 串口日志输出系统信息
- [x] 验证编译、烧录、串口监控流程

### 第二阶段：温度采集（1天）✅ 已完成

- [x] 接入 DS18B20 传感器（GPIO4 + 4.7kΩ 上拉电阻）
- [x] ~~集成 `onewire_bus` + `ds18b20` 组件~~ → **改为手写位拆裂驱动**（`ds18b20.c`，零外部依赖）
- [x] 实现 5 秒间隔温度读取（CRC 校验 + 12-bit 精度）
- [x] 串口打印温度验证精度

> **与计划偏离**：第二阶段未使用 ESP-IDF Component Registry 的 `onewire_bus`/`ds18b20` 组件，而是用纯 GPIO 控制实现了完整的 1-Wire 协议栈。优势：无外部依赖、更小的固件体积、更深入理解 1-Wire 协议。

### 第三阶段：摄像头驱动（1~2天）✅ 已完成

- [x] 集成 `esp32-camera` 组件（通过 Component Registry `espressif/esp32-camera ^2.0.0`）
- [x] 创建 `camera_handler.h/.c` 封装模块（初始化 + 拍照 + 释放帧缓冲）
- [x] 实现 60 秒间隔定时拍照（320×240 JPEG，约 4.6KB/帧）
- [x] 串口打印 JPEG 大小、分辨率、时间戳
- [x] 开启 PSRAM（`sdkconfig.defaults` 中配置 Octal SPI 80MHz）
- [x] GPIO 冲突解决（DS18B20 从 GPIO4 移到 GPIO1，GPIO4 专用于摄像头 SCCB SDA）
- [ ] 调整曝光、白平衡参数（室内水族箱灯光环境）— 待实际拍摄后根据效果调
- [ ] 可选手动触发拍照（GPIO 按键）方便调试

> **关键教训**：GPIO4 同时被 DS18B20 和摄像头 SCCB 占用，导致 I²C 总线间歇性失败。1-Wire 和 I²C 不能共用同一个 GPIO。将 DS18B20 移到 GPIO1 后问题解决。
>
> **实现决策**：使用 `idf_component.yml` 声明 `espressif/esp32-camera` 依赖。摄像头初始化在 `camera_task` 内部完成（非 `app_main`），失败后每 30 秒重试，不影响温度采集。引脚映射与原理图和参考程序 100% 对齐。

### 第四阶段：MQTT 上报（1~2天）✅ 已完成

- [x] 集成 `esp-mqtt` 组件
- [x] 创建 `mqtt_handler.h/c` 封装层
- [x] 连接公共 MQTT Broker (`broker.emqx.io:1883`)
- [x] 设备 ID 基于 MAC 地址后 3 字节（如 `A1B2C3`）
- [x] 异步发布 API：`mqtt_publish()` / `mqtt_publish_binary()`
- [x] 自动重连（ESP-MQTT 内置机制）
- [x] 4 个 Topic: temperature / fish_status / image / status
- [x] 使用 MQTTX 或手机 App 验证数据接收

### 第五阶段：鱼群检测（3~5天）✅ 已完成

- [x] 帧差法引擎：JPEG 解码 → 灰度 → 差分 → 连通域分析 → 计数+活跃度
- [x] YOLO 引擎（ESP-DL）：代码就绪，模型待 PC 端训练
- [x] 双引擎 `#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO` 切换
- [x] `fish_result_t` API 不变，上层 `main.c` 零修改
- [x] YOLO 管线：JPEG → RGB565 → 224×224 RGB888 (双线性插值) → ESP-DL 推理 → NMS
- [x] 活跃度追踪：YOLO 模式用 bbox 质心位移 (最近邻匹配)
- [x] 上报分析结果到 MQTT (`fish_status` topic)

### 第六阶段：YOLO 模型训练与部署（待执行）

- [ ] 鱼缸照片采集（200~500 张，不同时间、密度、光照）
- [ ] LabelImg 标注（每条鱼画 bbox，输出 YOLO 格式）
- [ ] ESP-Detection 训练 + INT8 量化（`python espdet_run.py`）
- [ ] 模型文件 `espdet_pico_fish.espdl` 放入 `main/models/`
- [ ] menuconfig 切换为 `CONFIG_SMART_FISHER_DETECTION_YOLO`
- [ ] 编译测试 + 真实鱼缸验证 + 与帧差法对比

> 📖 详细指南：`docs/yolo-practical-guide.md`（标注→训练→部署全流程）

### 第七阶段：稳定与优化（待执行）

- [ ] 内存泄漏检查（长时间运行，监控 free heap）
- [ ] 看门狗保护（Task WDT + 中断 WDT）
- [ ] OTA 远程升级支持
- [ ] 功耗优化（可选 deep-sleep，非必需）

---

## 7. 开发环境

| 项目 | 内容 |
|------|------|
| **ESP-IDF 版本** | v6.0 |
| **目标芯片** | ESP32-S3 (已配置) |
| **编辑器** | VS Code + ESP-IDF 扩展 |
| **串口工具** | `idf.py monitor` 或 Putty |
| **MQTT 测试工具** | MQTTX (桌面客户端) |
| **电源** | USB 5V 供电，电流 ≥ 1A (摄像头启动瞬间峰值可达 500mA+) |

## 8. 硬件物料清单

| 物料 | 型号/规格 | 数量 | 说明 |
|------|----------|------|------|
| 开发板 | ESP32-S3-CAM R16N8 | 1 | 已含 OV5640 |
| 温度传感器 | DS18B20 TO-92 | 1 | 防水款推荐（带不锈钢封装导管线） |
| 上拉电阻 | 4.7kΩ 1/4W | 1 | 1-Wire 总线必需 |
| 杜邦线 | 母对母 | 3 | 连接传感器 |
| 面包板 | 170孔 | 1 | 可选 |
| USB 电源 | 5V 2A | 1 | 确保供电充足 |

> **强烈推荐使用防水型 DS18B20**（带不锈钢探头 + 3 米导线），可直接浸入鱼缸水中。普通 TO-92 封装需额外做防水处理。

## 9. 风险与注意事项

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| OV5640 耗电高 | USB 供电不足导致重启 | 使用 5V/2A 电源，禁用 WiFi 省电模式 |
| 帧差法误判鱼数 | 气泡、水草晃动造成假阳性 | 调整阈值，形态学去噪，限定 ROI |
| MQTT 公共 Broker 不稳定 | 数据丢失 | 本地缓存 + 重传机制，QoS 1 |
| 传感器浸水损坏 | 硬件故障 | 使用防水探头，接口处涂抹硅胶密封 |
| WiFi 断连 | 数据中断 | 自动重连 + 数据缓存 + 看门狗复位 |
| 图片上传占用带宽过大 | MQTT 阻塞 | JPEG 压缩质量 15，分辨率控制在 SVGA |

---

## 10. 摄像头调试实录（Phase 2 踩坑全记录）

以下是 Phase 2 开发过程中遇到的所有问题和解决方案，按时间顺序记录。

---

### 问题 1：找不到 `driver/gpio.h`

**现象**：编译报错 `fatal error: driver/gpio.h: No such file or directory`

**根因**：ESP-IDF v6.0 将原来统一的 `driver` 组件拆分成了独立的 `esp_driver_*` 组件。`driver/gpio.h` 头文件仍然存在，但位于 `components/esp_driver_gpio/include/` 下。如果不显式声明依赖，CMake 不会把该组件的 include 路径加入编译命令。

**解决**：在 `main/CMakeLists.txt` 的 `PRIV_REQUIRES` 中添加 `esp_driver_gpio`。

**教训**：
- ESP-IDF v6.0 中，凡是 `#include "driver/xxx.h"` 都要在 CMake 中声明对应的 `esp_driver_xxx` 依赖
- 常见映射：`driver/gpio.h` → `esp_driver_gpio`，`driver/uart.h` → `esp_driver_uart` 等

---

### 问题 2：摄像头完全无响应（`Detected camera not supported`）

**现象**：
```
I (9662) sccb-ng: pin_sda 40 pin_scl 39
E (9692) camera: Detected camera not supported.
```

**根因**：SCCB (I²C) 引脚配置错误。最初使用的 GPIO40(SDA)/GPIO39(SCL) 与这块开发板的实际接线不符。ESP32-S3 不同开发板的摄像头引脚差异非常大，同一型号的不同批次可能用完全不同的 GPIO 映射。

**调试过程**：
1. 尝试了 3 套猜测的引脚配置，全部失败
2. 从 `docs/ESP32-S3CAM原理图.pdf` 中提取了真实引脚映射
3. 从参考程序 `docs/参考程序/esp32-s3-cam/` 交叉验证

**解决**：使用原理图提取的真实引脚映射，与参考程序的 `BOARD_ESP32S3_WROOM` 配置 100% 一致。

**原理图引脚映射表**：

| 摄像头信号 | GPIO | 说明 |
|-----------|------|------|
| CAM0SIOD (SDA) | 4 | SCCB 数据线 |
| CAM0SIOC (SCL) | 5 | SCCB 时钟线 |
| CAM0VYSNC | 6 | 垂直同步 |
| CAM0HREF | 7 | 水平参考 |
| CAM0Y2 (D0) | 11 | 数据位 0 |
| CAM0Y3 (D1) | 9 | 数据位 1 |
| CAM0Y4 (D2) | 8 | 数据位 2 |
| CAM0Y5 (D3) | 10 | 数据位 3 |
| CAM0Y6 (D4) | 12 | 数据位 4 |
| CAM0Y7 (D5) | 18 | 数据位 5 |
| CAM0Y8 (D6) | 17 | 数据位 6 |
| CAM0Y9 (D7) | 16 | 数据位 7 |
| CAM0PCLK | 13 | 像素时钟 |
| CAM0XCLK | 15 | 主时钟 (20MHz) |
| OV_RESET | -1 | 软复位（不用硬复位） |

**教训**：
- 永远不要猜测 ESP32-S3 的摄像头引脚——找原理图或参考程序验证
- 不同开发板的引脚映射完全不可互换
- 数据线 (D0-D7) 不是按 GPIO 编号顺序分配的——PCB 走线优先考虑信号完整性

---

### 问题 3：检测成功但寄存器写入失败（`W [3800]=00 fail`）

**现象**：
```
I (7502) camera: Detected OV5640 camera       ← 检测成功！
E (8062) sccb-ng: W [3800]=00 fail            ← 写寄存器失败
E (8062) camera: Camera probe failed
```

**调试过程（多轮试错）**：
1. 降低 XCLK 频率 20MHz→10MHz → 无效
2. 增加初始化延时 10ms→200ms → 无效
3. 禁用硬件复位 (pin_reset=-1) → 无效
4. 切换 I²C 端口 I2C1→I2C0 → 更差，更早失败
5. 降低分辨率 SVGA→QQVGA → 无效
6. 开启 REG_DEBUG_ON → 发现关键线索

**关键线索（REG_DEBUG_ON 日志）**：
```
E (8062) sccb-ng: W [5181]=39 fail          ← I²C 读失败！
W (8062) ds18b20: Scratchpad CRC mismatch    ← 同时 DS18B20 CRC 错误！
```

两个错误发生在**同一毫秒**（8062ms），说明是系统级的电气问题，不是软件 bug。

**根因**：**GPIO4 被两个设备同时占用**

```
GPIO4 ← DS18B20 数据线 (1-Wire 协议, 4.7kΩ 上拉)
GPIO4 ← OV5640 SCCB SDA (I²C 协议, 内部 ~45kΩ 上拉)
```

当摄像头初始化时，I²C 总线在 GPIO4 上快速翻转电平（100kHz），DS18B20 收到垃圾 1-Wire 数据后尝试应答，将总线拉低，破坏了 I²C 传输。两个协议的电平要求完全不同，不能共存于同一 GPIO。

**解决**：将 DS18B20 数据线从 GPIO4 移到 GPIO1。

**教训**：
- **一个 GPIO 只能接一个外设**——这是嵌入式硬件的基本法则
- 日志中的"同时发生"不是巧合，是共享资源的冲突表现
- 查硬件问题前先看日志中不同模块的时间戳是否对齐
- 1-Wire 和 I²C 的电平/时序完全不兼容，不可共用引脚

---

### 问题 4：ESP-IDF v6.0 组件系统变化

**现象**：各种编译错误和 IntelliSense 报错

**根因**：ESP-IDF v6.0 做了大量架构调整：
- `driver/` 组件拆分为独立的 `esp_driver_*` 组件
- `sdkconfig` 配置项的默认值可能变化（如 PSRAM 速度）
- Kconfig 依赖关系调整

**解决**：
- 所有 `driver/xxx.h` 的 include 都需要对应的 CMake PRIV_REQUIRES
- PSRAM 速度显式设置为 80MHz（Kconfig 默认是 40MHz，不够用）
- sdkconfig 冲突时删除重建：`rm sdkconfig && idf.py build`

---

### 最终工作配置汇总

**DS18B20 接线**：
| DS18B20 | ESP32-S3-CAM |
|---------|-------------|
| VCC (红) | 3.3V |
| GND (黑) | GND |
| DATA (黄) | **GPIO1**（⚠ 不是 GPIO4）|
| 上拉电阻 | 4.7kΩ DATA→3.3V |

**摄像头接线**：板载 FPC 排线，无需额外接线。

**GPIO 分配冲突表**：
| GPIO | 功能 | 备注 |
|------|------|------|
| GPIO1 | DS18B20 1-Wire | 从 GPIO4 迁过来 |
| GPIO4 | OV5640 SCCB SDA | 原与 DS18B20 冲突 |
| GPIO5 | OV5640 SCCB SCL | |
| GPIO6-18 | OV5640 DVP | 见上表 |
| GPIO26-34 | Octal PSRAM | 不可占用 |
| GPIO46 | 摄像头复位 | 当前用软复位 (-1) |

## 11. 参考资源

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [ESP32-CAM 摄像头驱动](https://github.com/espressif/esp32-camera)
- [OneWire Bus 组件](https://components.espressif.com/components/espressif/onewire_bus)
- [DS18B20 组件](https://components.espressif.com/components/espressif/ds18b20)
- [ESP-MQTT 文档](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/api-reference/protocols/mqtt.html)
- [EMQX 公共 Broker](https://www.emqx.com/zh/mqtt/public-mqtt5-broker)
- [MQTTX 客户端](https://mqttx.app/zh)
