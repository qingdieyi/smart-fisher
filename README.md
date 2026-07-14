# Smart Fisher — 鱼缸智能监控系统

基于 ESP32-S3-CAM R16N8 的鱼缸水温监测 + 摄像头拍照系统，数据通过 WiFi 上报。

## 硬件

| 物料 | 型号 |
|------|------|
| 开发板 | ESP32-S3-CAM R16N8 (16MB Flash, 8MB Octal PSRAM) |
| 摄像头 | OV5640 (500万像素, 板载 FPC 排线) |
| 温度传感器 | DS18B20 防水探头 (1-Wire 协议) |

### 接线

**DS18B20**：

| DS18B20 引脚 | ESP32-S3-CAM | 说明 |
|-------------|-------------|------|
| VCC (红) | 3.3V | |
| GND (黑) | GND | |
| DATA (黄) | **GPIO1** | 必须加 4.7kΩ 上拉电阻到 3.3V |

> ⚠ DS18B20 最初接在 GPIO4，但 GPIO4 也是摄像头 SCCB SDA (I²C) 引脚。两个协议共用同一 GPIO 会导致信号互相干扰。改为 GPIO1 后问题解决。

**OV5640 摄像头**：板载 FPC 排线连接，无需额外接线。引脚映射详见 `docs/development-plan.md`。

## 功能

### 已完成 (Phase 1 ~ 5)

- ✅ WiFi WPA2-PSK 连接 (自动重连, 最多 5 次失败后重启)
- ✅ DS18B20 温度采集 (12-bit 精度, 0.0625°C, 每 5 秒)
- ✅ OV5640 摄像头拍照 (800×600 JPEG, 硬件编码, 每 60 秒)
- ✅ MQTT 数据上报 (broker.emqx.io 公共 Broker, JSON + JPEG)
- ✅ 鱼群检测 — 帧差法运动检测 (连通域分析, 计数 + 活跃度)

### MQTT Topics

| Topic | QoS | 说明 |
|-------|-----|------|
| `smart-fisher/{id}/temperature` | 0 | 水温 JSON |
| `smart-fisher/{id}/fish_status` | 0 | 鱼群状态 JSON |
| `smart-fisher/{id}/image` | 1 | 鱼缸照片 JPEG |
| `smart-fisher/{id}/status` | 1 | 设备状态 JSON (retained) |

## 快速开始

### 环境要求

- ESP-IDF v6.0
- Windows: `C:\Users\22847\esp\v6.0\esp-idf`
- 串口: COM8

### 编译 & 烧录

```bash
cd smart-fisher

# 首次编译（需要网络下载 esp32-camera 组件）
idf.py build

# 烧录（按住 BOOT → 按 RESET → 松 RESET → 松 BOOT 进入下载模式）
idf.py -p COM8 flash

# 查看串口日志
idf.py -p COM8 monitor
```

### 配置 WiFi

编辑 `sdkconfig.defaults`：
```ini
CONFIG_EXAMPLE_WIFI_SSID="你的WiFi名"
CONFIG_EXAMPLE_WIFI_PASSWORD="你的WiFi密码"
```

或通过 menuconfig：
```bash
idf.py menuconfig → Example Configuration
```

## 项目结构

```
smart-fisher/
├── main/
│   ├── main.c                   # 入口 — WiFi + 温度任务 + 摄像头任务 + MQTT
│   ├── ds18b20.h / ds18b20.c    # DS18B20 1-Wire 位拆裂驱动（零外部依赖）
│   ├── camera_handler.h / .c    # OV5640 摄像头封装（依赖 esp32-camera 组件）
│   ├── mqtt_handler.h / .c      # MQTT 数据上报（依赖 esp_mqtt）
│   ├── fish_detector.h / .c     # 鱼群帧差法检测（依赖 esp_jpeg）
│   ├── idf_component.yml        # 组件依赖声明 (espressif/esp32-camera ^2.0.0)
│   ├── CMakeLists.txt           # 源文件 + 编译依赖
│   └── Kconfig.projbuild        # menuconfig 配置项
├── docs/
│   ├── brd.md                     # 原始需求文档
│   ├── development-plan.md        # 完整开发方案 + 调试实录
│   ├── esp-dl-yolo-plan.md        # ESP-DL + YOLO 鱼群检测方案
│   ├── yolo-practical-guide.md    # 标注→训练→部署实操指南
│   ├── ESP32-S3CAM原理图.pdf       # 开发板原理图
│   └── 参考程序/                   # 官方参考程序 (BOARD_ESP32S3_WROOM)
├── sdkconfig.defaults         # 项目默认 Kconfig 配置
└── CMakeLists.txt             # 顶层 CMake
```

## 架构

```
app_main()
  ├─ nvs_flash_init()              → NVS 存储初始化
  ├─ wifi_init_sta()               → WPA2-PSK 连接
  ├─ mqtt_handler_init()           → MQTT 客户端 (broker.emqx.io:1883)
  ├─ ds18b20_init(GPIO1)           → 温度传感器初始化
  ├─ fish_detector_init()          → 鱼群检测器初始化
  ├─ xTaskCreate(temp_task)         → 温度任务 (5s, prio 3) + MQTT 上报
  ├─ xTaskCreate(camera_task)       → 摄像头任务 (60s, prio 2) + 鱼群检测 + MQTT 上报
  └─ 主循环 — 定期上报设备状态 (60s)
```

## 调试日志示例

```
I (7342) camera: Detected OV5640 camera
I (8482) camera: Camera initialized successfully!
I (10492) smart-fisher: 📷 Photo: 800x600, 15420 bytes, ts=123456ms
I (10520) fish-detector: 🐟 Detection: 3 fish, activity=moderate (score=12%, motion_pixels=900) [65 ms]
I (10530) mqtt: Published to smart-fisher/A1B2C3/fish_status (98 bytes, msg_id=12345)
I (10531) mqtt: Published binary to smart-fisher/A1B2C3/image (15420 bytes, msg_id=12346)
I (13762) smart-fisher: 🌡️  Water Temperature: 29.00 °C
```

## 关键经验

- **GPIO 冲突**：1-Wire 和 I²C 不能共用同一 GPIO。DS18B20 从 GPIO4 移到 GPIO1
- **引脚映射**：ESP32-S3 摄像头引脚不可猜测——必须从原理图或参考程序验证
- **ESP-IDF v6.0**：`driver/xxx.h` 需要显式 `PRIV_REQUIRES esp_driver_xxx`
- **电源**：OV5640 + ESP32-S3 + WiFi 峰值电流约 600mA，建议 USB 3.0 口或独立 5V/2A 供电

## 参考资源

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [esp32-camera 组件](https://github.com/espressif/esp32-camera)
- [EMQX 公共 MQTT Broker](https://www.emqx.com/zh/mqtt/public-mqtt5-broker)
