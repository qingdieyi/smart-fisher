# Smart Fisher — 鱼缸智能监控系统

基于 ESP32-S3-CAM R16N8 的鱼缸水温监测 + 摄像头拍照 + 鱼群检测系统，数据通过 WiFi + MQTT 上报。

## 硬件

| 物料 | 型号 |
|------|------|
| 开发板 | ESP32-S3-CAM R16N8 (16MB Flash, 8MB Octal PSRAM) |
| 摄像头 | OV5640 (500万像素, 板载 FPC 排线) |
| 温度传感器 | DS18B20 防水探头 (1-Wire 协议) |
| 上拉电阻 | 4.7kΩ (DS18B20 DATA ↔ 3.3V) |

### 接线

**DS18B20**：

| DS18B20 引脚 | ESP32-S3-CAM | 说明 |
|-------------|-------------|------|
| VCC (红) | 3.3V | |
| GND (黑) | GND | |
| DATA (黄) | **GPIO1** | 必须加 4.7kΩ 上拉电阻到 3.3V |

> ⚠ **GPIO 冲突教训**：DS18B20 最初接在 GPIO4，但 GPIO4 同时也是摄像头 SCCB SDA (I²C)。两个协议共用同一 GPIO 会导致信号干扰。改为 GPIO1 后解决。

**OV5640 摄像头**：板载 FPC 排线连接，无需额外接线。引脚映射详见 `docs/development-plan.md`。

## 功能

### 已完成的阶段

| 阶段 | 功能 | 状态 |
|------|------|------|
| Phase 1 | WiFi WPA2-PSK 连接 (自动重连, 最多 5 次失败后重启) | ✅ |
| Phase 2 | DS18B20 温度采集 (12-bit, 0.0625°C, 每 5 秒) | ✅ |
| Phase 3 | OV5640 摄像头拍照 (800×600 JPEG, 硬件编码, 每 60 秒) | ✅ |
| Phase 4 | MQTT 数据上报 (broker.emqx.io:1883) | ✅ |
| Phase 5 | 鱼群检测 — 帧差法 (默认) / YOLO (可选) | ✅ |
| Phase 6 | ESP-DL + YOLO 神经网络检测 (代码就绪, 待模型训练) | 🔧 |

### 检测引擎

项目支持**双引擎**架构，通过 `menuconfig` 一键切换：

| 引擎 | 原理 | 速度 | 内存 | 能检测静止鱼 |
|------|------|------|------|:---:|
| **帧差法** (默认) | 相邻帧像素差分 | ~80ms | ~40KB PSRAM | ❌ |
| **YOLO** (可选) | ESP-DL 神经网络推理 | ~200ms | ~700KB PSRAM + 500KB Flash | ✅ |

> 📖 切换到 YOLO 的完整指南见 [`docs/esp-dl-yolo-plan.md`](docs/esp-dl-yolo-plan.md) 和 [`docs/yolo-practical-guide.md`](docs/yolo-practical-guide.md)

### MQTT Topics

| Topic | QoS | 内容 |
|-------|-----|------|
| `smart-fisher/{id}/temperature` | 0 | `{"temperature":25.50,"unit":"celsius","timestamp":123456}` |
| `smart-fisher/{id}/fish_status` | 0 | `{"fish_count":3,"motion_score":12,"activity":"moderate","timestamp":123456}` |
| `smart-fisher/{id}/image` | 1 | JPEG 二进制 (~10-30KB) |
| `smart-fisher/{id}/status` | 1 | `{"status":"online","uptime_s":3600,"free_heap":4234567,"wifi_rssi":-45}` (retained) |

设备 ID 由 MAC 地址后 3 字节生成（如 `A1B2C3`）。

## 快速开始

### 环境要求

- **ESP-IDF** v6.0
- **串口**: COM8 (可在 `idf.py flash` 时指定)
- **电源**: 5V/2A USB（OV5640 + WiFi 峰值需 600mA+）

### 编译 & 烧录

```bash
cd smart-fisher

# 首次编译（需要网络下载 esp32-camera 和 esp_jpeg 组件）
idf.py build

# 烧录（按住 BOOT → 按 RESET → 松 RESET → 松 BOOT 进入下载模式）
idf.py -p COM8 flash

# 查看串口日志
idf.py -p COM8 monitor
```

### 配置 WiFi

编辑 `sdkconfig.defaults`：

```ini
CONFIG_SMART_FISHER_WIFI_SSID="你的WiFi名"
CONFIG_SMART_FISHER_WIFI_PASSWORD="你的WiFi密码"
```

或通过 menuconfig：

```bash
idf.py menuconfig → Smart Fisher Configuration
```

### 切换到 YOLO 检测

```bash
# 前提：已完成模型训练（见 docs/yolo-practical-guide.md）
# 1. 放置模型文件
cp espdet_pico_fish.espdl main/models/

# 2. menuconfig 切换
idf.py menuconfig → Smart Fisher Configuration
    → Fish Detection Method → YOLO Object Detection

# 3. 重新编译
idf.py fullclean && idf.py build && idf.py -p COM8 flash monitor
```

## 项目结构

```
smart-fisher/
├── main/
│   ├── main.c                   # 入口 — WiFi + 任务调度 + MQTT 总控
│   ├── ds18b20.h / ds18b20.c    # DS18B20 1-Wire 位拆裂驱动（零外部依赖）
│   ├── camera_handler.h / .c    # OV5640 摄像头封装（依赖 esp32-camera）
│   ├── mqtt_handler.h / .c      # MQTT 数据上报（依赖 esp_mqtt）
│   ├── fish_detector.h / .c     # 鱼群检测 — 帧差法 + YOLO 双引擎
│   ├── models/
│   │   └── README.md            # 模型文件放置说明
│   ├── idf_component.yml        # 组件依赖声明
│   ├── CMakeLists.txt           # 源文件 + 编译依赖 + 模型嵌入
│   └── Kconfig.projbuild        # menuconfig 配置项
├── docs/
│   ├── brd.md                   # 原始需求文档
│   ├── development-plan.md      # 完整开发方案 + 踩坑实录
│   ├── esp-dl-yolo-plan.md      # ESP-DL + YOLO 技术方案
│   ├── yolo-practical-guide.md  # 标注 → 训练 → 部署实操指南
│   ├── ESP32-S3CAM原理图.pdf     # 开发板原理图
│   └── 参考程序/                 # 官方参考程序
├── sdkconfig.defaults           # 项目默认 Kconfig 配置
└── CMakeLists.txt               # 顶层 CMake 构建文件
```

## 架构

```
app_main()
  ├─ nvs_flash_init()                → NVS 存储初始化
  ├─ wifi_init_sta()                 → WPA2-PSK 连接 (WiFi 事件回调)
  ├─ mqtt_handler_init()             → MQTT 客户端 (异步连接, 自动重连)
  ├─ ds18b20_init(GPIO1)             → 温度传感器初始化
  ├─ fish_detector_init()            → 检测引擎初始化 (帧差法 or YOLO)
  ├─ xTaskCreate(temperature_task)    → 温度任务 (5s, prio 3)
  │     └─ ds18b20_read_temperature → mqtt_publish (temperature topic)
  ├─ xTaskCreate(camera_task)        → 摄像头任务 (60s, prio 2)
  │     └─ camera_capture → fish_detector_analyze
  │         → mqtt_publish (fish_status + image topics)
  └─ 主循环 (60s)
        └─ mqtt_publish (status topic — uptime, free_heap, RSSI)
```

## 组件依赖图

```
main.c
  ├── ds18b20.{h,c}         ← 纯 GPIO bit-bang (无外部依赖)
  ├── camera_handler.{h,c}  ← esp32-camera (managed component)
  ├── mqtt_handler.{h,c}    ← esp_mqtt (ESP-IDF 内置)
  ├── fish_detector.{h,c}   ← esp_jpeg (managed)
  │                          ← esp_dl (managed, 仅 YOLO 模式)
  └── FreeRTOS              ← 内置 (任务调度)
```

## 调试日志示例

### 帧差法模式

```
I (7342) camera: Detected OV5640 camera
I (8482) camera: Camera initialized successfully!
I (10492) smart-fisher: 📷 Photo: 800x600, 15420 bytes, ts=123456ms
I (10520) fish-detector: 🐟 Detection: 3 fish, moderate (score=12%, motion_pixels=900) [65 ms]
I (10530) mqtt: Published to smart-fisher/A1B2C3/fish_status
I (10531) mqtt: Published binary to smart-fisher/A1B2C3/image (15420 bytes)
I (13762) smart-fisher: 🌡️  Water Temperature: 29.00 °C
```

### YOLO 模式

```
I (5000) fish-detector: YOLO model loaded successfully
I (5000) fish-detector: YOLO detector ready. Input buffer: 150528 bytes (224x224x3)
I (65000) fish-detector: 🐟 Detection: 3 fish, active (score=65%)
         [jpeg=52ms, resize=18ms, infer=120ms, total=195ms]
```

## 关键经验

| 类别 | 经验 |
|------|------|
| **GPIO 冲突** | 1-Wire 和 SCCB (I²C) 不能共用 GPIO。DS18B20 GPIO4 → GPIO1 |
| **引脚映射** | ESP32-S3 摄像头引脚布局非常规——必须从原理图/参考程序验证 |
| **ESP-IDF v6.0** | `driver/xxx.h` 拆分，需显式 `PRIV_REQUIRES esp_driver_xxx` |
| **PSRAM** | 摄像头帧缓冲必须在 PSRAM。R16N8 的 8MB Octal PSRAM 经 SPI 访问 |
| **电源** | OV5640 + ESP32-S3 + WiFi 峰值 600mA。USB 2.0 口 (500mA) 供电不足 |
| **双引擎** | 帧差法零依赖开箱即用，YOLO 需 PC 端训练模型。API 不变, `#ifdef` 切换 |

## 文档索引

| 文档 | 说明 |
|------|------|
| [`docs/brd.md`](docs/brd.md) | 产品需求简述 |
| [`docs/development-plan.md`](docs/development-plan.md) | 完整开发方案 + 摄像头调试实录 |
| [`docs/esp-dl-yolo-plan.md`](docs/esp-dl-yolo-plan.md) | YOLO 技术选型、架构设计、内存预算 |
| [`docs/yolo-practical-guide.md`](docs/yolo-practical-guide.md) | 标注工具 → 训练命令 → 部署代码 全流程 |

## 参考资源

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [esp32-camera 组件](https://github.com/espressif/esp32-camera)
- [ESP-Detection](https://github.com/espressif/esp-detection) — YOLO 一键训练+部署
- [ESP-DL 文档](https://docs.espressif.com/projects/esp-dl/)
- [EMQX 公共 MQTT Broker](https://www.emqx.com/zh/mqtt/public-mqtt5-broker)
