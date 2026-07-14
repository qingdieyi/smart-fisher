# CLAUDE.md — Smart Fisher 项目手册

## 项目简介

Smart Fisher 是基于 ESP32-S3-CAM R16N8 的鱼缸智能监控系统。采集水温 (DS18B20) 和照片 (OV5640)，通过帧差法或 YOLO 神经网络检测鱼的数量和活跃度，数据经 WiFi + MQTT 上报到公共 Broker。

## 硬件平台

- **MCU**: ESP32-S3 (Xtensa LX7 @ 240MHz)
- **Flash**: 16MB | **PSRAM**: 8MB Octal SPI
- **摄像头**: OV5640 (500万像素 DVP 接口, 硬件 JPEG 编码)
- **温度传感器**: DS18B20 (1-Wire, 12-bit, ±0.0625°C)
- **关键引脚**: DS18B20 DATA → GPIO1 (4.7kΩ 上拉至 3.3V)

## 构建命令

```bash
# 配置 (一次性)
idf.py set-target esp32s3

# 编译
idf.py build

# 烧录 (按住 BOOT → RESET → 松 RESET → 松 BOOT)
idf.py -p COM8 flash

# 串口监控
idf.py -p COM8 monitor

# 菜单配置
idf.py menuconfig
# → Smart Fisher Configuration
#   → WiFi SSID / Password
#   → Fish Detection Method (Frame Diff / YOLO)
```

## 源码导航

```
main/
├── main.c                  [787 行] 入口 + WiFi STA + 任务调度 + MQTT 总控
├── ds18b20.{h,c}           [119+740 行] 1-Wire bit-bang 驱动 (纯 GPIO, 零依赖)
├── camera_handler.{h,c}    [167+666 行] OV5640 封装 (依赖 esp32-camera)
├── mqtt_handler.{h,c}      [120+250 行] MQTT 异步发布 (依赖 esp_mqtt)
├── fish_detector.{h,c}     [130+780 行] 鱼群检测双引擎 (依赖 esp_jpeg, 可选 esp_dl)
├── CMakeLists.txt          构建配置 + 条件模型嵌入
├── idf_component.yml       组件依赖 (esp32-camera, esp-dl)
├── Kconfig.projbuild       menuconfig 配置项
└── models/
    └── README.md           模型文件放置说明
```

## 架构

```
app_main()
  ├─ nvs_flash_init()
  ├─ wifi_init_sta()          → EventGroup: WIFI_CONNECTED_BIT
  ├─ xEventGroupWaitBits()    → 阻塞等待 WiFi
  ├─ mqtt_handler_init()      → 异步连接 broker.emqx.io:1883
  ├─ ds18b20_init(GPIO1)      → 1-Wire 设备检测 + 12-bit 配置
  ├─ fish_detector_init()     → 帧差法 OR YOLO (编译时选择)
  ├─ xTaskCreate(temp_task,   prio=3, stack=4096)
  │   └─ 每 5s: ds18b20_read → mqtt_publish(temperature)
  ├─ xTaskCreate(camera_task, prio=2, stack=8192)
  │   └─ 每 60s: camera_capture → fish_detector_analyze → mqtt_publish
  └─ while(1) 每 60s: mqtt_publish(status)
```

## MQTT Topics

| Topic | QoS | Payload | 间隔 |
|-------|-----|---------|------|
| `smart-fisher/{MAC}/temperature` | 0 | `{"temperature":25.50,"unit":"celsius","timestamp":...}` | 5s |
| `smart-fisher/{MAC}/fish_status` | 0 | `{"fish_count":3,"motion_score":12,"activity":"moderate","timestamp":...}` | 60s |
| `smart-fisher/{MAC}/image` | 1 | JPEG binary | 60s |
| `smart-fisher/{MAC}/status` | 1 | `{"status":"online","uptime_s":...,"free_heap":...,"wifi_rssi":...}` | 60s |

设备 ID (`{MAC}`) 由 `esp_read_mac(WIFI_STA)` 后 3 字节组成，如 `A1B2C3`。

## 检测引擎切换

```bash
# 当前默认: 帧差法 (零依赖, 开箱即用)
# 切换到 YOLO:
idf.py menuconfig → Smart Fisher Configuration
    → Fish Detection Method → YOLO Object Detection

# 前提条件:
#   1. main/models/espdet_pico_fish.espdl 存在
#   2. idf_component.yml 中 espressif/esp-dl 已取消注释
#   3. 详见 docs/yolo-practical-guide.md
```

## FreeRTOS 任务优先级

| 任务 | 优先级 | 栈 | 周期 | 原因 |
|------|--------|-----|------|------|
| temp_task | 3 (最高) | 4096 | 5s | DS18B20 时序敏感, 不能被抢占 |
| camera_task | 2 | 8192 | 60s | JPEG 解码 + 检测需要更大栈 |
| main (app_main) | 1 | 系统 | 60s | 设备状态上报 |

## 编码约定

- **命名**: `模块名_动作名()` 例: `ds18b20_read_temperature()`, `mqtt_publish()`
- **错误处理**: 返回 `esp_err_t`, 检查返回值, 失败不崩溃 (优雅降级)
- **日志**: `ESP_LOGI/E/W/D(TAG, "format", args...)` — TAG 是模块名
- **内存**: PSRAM 用于大 buffer (帧缓冲, 模型输入), 内部 SRAM 用于任务栈
- **头文件**: `#pragma once` + `extern "C"` 块
- **注释**: 中文解释 + Java 类比, 面向 Java 转 C 的开发者

## 常见修改场景

### 修改 WiFi 凭证
```c
// main/main.c (或 sdkconfig.defaults)
#define WIFI_SSID      "MyWiFi"
#define WIFI_PASSWORD  "MyPassword"
```

### 调整拍照间隔
```c
// main/main.c
#define CAMERA_CAPTURE_INTERVAL_MS  300000  // 5 分钟
```

### 更换 MQTT Broker
```c
// main/mqtt_handler.c
#define MQTT_BROKER_URI  "mqtt://my-broker.com:1883"
```

### 调整帧差法灵敏度
```c
// main/fish_detector.c (帧差法部分)
#define MOTION_THRESHOLD    28    // 调低=更敏感, 调高=更严格
#define MIN_FISH_AREA       3     // 最小连通域像素数
```

### 调整 YOLO 检测阈值
```bash
idf.py menuconfig → Smart Fisher Configuration
    → YOLO Confidence Threshold (default 0.50)
    → YOLO NMS IoU Threshold (default 0.45)
```

## 依赖关系

```
main
├── esp_wifi          (IDF 内置)
├── nvs_flash         (IDF 内置)
├── esp_driver_gpio   (IDF 内置, v6.0 拆分)
├── esp_timer         (IDF 内置)
├── esp_mqtt          (IDF 内置)
├── esp_jpeg          (managed, 自动下载)
├── esp32-camera      (managed, espressif/esp32-camera ^2.0.0)
└── esp_dl            (managed, espressif/esp-dl ^3.3.0 — 仅 YOLO 模式)
```

## 文档索引

| 文档 | 说明 |
|------|------|
| `README.md` | 项目概览 + 快速开始 |
| `docs/brd.md` | 产品需求简述 |
| `docs/development-plan.md` | 完整开发方案 + 摄像头调试实录 |
| `docs/esp-dl-yolo-plan.md` | YOLO 技术选型 + 架构设计 + 内存预算 |
| `docs/yolo-practical-guide.md` | 标注 → 训练 → 部署实操指南 |
| `docs/ESP32-S3CAM原理图.pdf` | 开发板原理图 |
| `docs/参考程序/` | 官方 esp32-camera 参考程序 |
