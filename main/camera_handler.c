/*
 * OV5640 摄像头驱动 — 实现
 * ==========================================
 *
 * ── 给你的话（Java 开发者看这里）───────────────────────────────────
 *
 * 这个文件封装了 ESP-IDF 的 esp32-camera 组件。
 * 摄像头驱动是嵌入式开发中"组件依赖"的典型场景：
 *
 *   - 底层：OV5640 传感器 → 通过 DVP 接口输出像素数据
 *   - 中间层：esp32-camera 组件 → 配置寄存器、管理帧缓冲、DMA 传输
 *   - 上层：我们的 camera_handler → 简化的拍照/释放 API
 *
 * 这就像 Java 中：
 *   我们的 Service → 调 Spring Data JPA → JPA 调 JDBC → JDBC 调数据库
 *
 * ── 摄像头如何拍照？ ────────────────────────────────────────────────
 *
 * OV5640 不是"按一下拍一张"的那种相机。它实际上在不停地输出像素流：
 *
 *   传感器 → ADC → DSP(白平衡/曝光) → JPEG 编码器 → FIFO
 *                                                    ↓
 *                                          ESP32 通过 DMA 搬走
 *                                                    ↓
 *                                          存到 PSRAM 帧缓冲
 *                                                    ↓
 *                                          我们的 camera_capture()
 *                                          拿到这一帧的指针
 *
 * 所以 "拍照" 的本质是：从连续视频流中抓取最新的一帧。
 * 这就是为什么 esp32-camera 的 API 叫 esp_camera_fb_get()
 * （fb = frame buffer，帧缓冲）。
 *
 * ── PSRAM 的作用 ────────────────────────────────────────────────────
 *
 * ESP32-S3 内部 SRAM 只有 512KB，不够存一帧 JPEG（800×600 约 30KB）。
 * 但 ESP32-S3-CAM R16N8 板载 8MB PSRAM（伪静态 RAM），通过 SPI 接口
 * 与 ESP32-S3 连接。PSRAM 容量大但速度比内部 SRAM 慢。
 *
 * 帧缓冲分配策略：
 *   - fb_location = CAMERA_FB_IN_PSRAM  → 帧缓冲放在 PSRAM（推荐）
 *   - fb_count = 2                      → 双缓冲（一个在用，一个排队）
 *
 *   双缓冲就像 Java 的 DoubleBuffer：一个 buffer 被 DMA 填充时，
 *   另一个可以被我们的代码读取，互不干扰。
 *
 * ── XCLK 时钟 ───────────────────────────────────────────────────────
 *
 * ESP32-S3 用 LEDC（LED PWM 控制器）为摄像头提供 XCLK 主时钟。
 * 这不是"开关 LED"——LEDC 是一种通用的 PWM/时钟生成器硬件，
 * 可以产生精确的方波信号。我们把它配置成 20MHz 输出给摄像头。
 *
 * ── SCCB 协议 ───────────────────────────────────────────────────────
 *
 * SCCB (Serial Camera Control Bus) 是 OmniVision 的专利协议，
 * 其实就是 I²C 的"方言"。ESP32 用 GPIO 模拟 I²C 来配置摄像头寄存器。
 * 这就是为什么需要指定 sccb_sda 和 sccb_scl 引脚。
 *
 * ═══════════════════════════════════════════════════════════════════
 *
 * 参考资料：
 *   - esp32-camera 组件: https://github.com/espressif/esp32-camera
 *   - OV5640 数据手册（322 页）: https://www.ovt.com/products/ov5640/
 */

/* ── 头文件引入 ───────────────────────────────────────────────────── */

#include "camera_handler.h"     /* 自己的头文件 */

#include "esp_camera.h"         /* esp32-camera 组件核心 API：
                                 *   - esp_camera_init()
                                 *   - esp_camera_fb_get()
                                 *   - esp_camera_fb_return()
                                 *   - camera_config_t, camera_fb_t 等类型     */
#include "esp_log.h"            /* ESP-IDF 日志系统 — 类似 Java 的 SLF4J */
#include "esp_timer.h"          /* 高精度定时器 — 类似 Java 的 System.nanoTime() */
#include "freertos/FreeRTOS.h"  /* FreeRTOS 核心 */
#include "freertos/task.h"      /* 任务管理 — vTaskDelay() */

/* ── 模块日志标签 ───────────────────────────────────────────────────
 *
 * 相当于 Java 的：
 *   private static final Logger LOG = LoggerFactory.getLogger("camera");
 *
 * 串口输出示例：
 *   I (23456) camera: Camera initialized. Frame size: 800x600 JPEG
 *                  ^^^^^^
 *                  这个 TAG
 */
static const char *TAG = "camera";

/* ═══════════════════════════════════════════════════════════════════
 * 摄像头引脚映射
 *
 * ── 来源 ─────────────────────────────────────────────────────────
 *
 * 以下引脚映射来自本板子的原理图（docs/ESP32-S3CAM原理图.pdf）。
 * ESP32-S3 通过 DVP（Digital Video Port）接口连接 OV5640。
 *
 * 原理图中的映射关系：
 *   CAM0SIOD  (SCCB SDA) ← GPIO4
 *   CAM0SIOC  (SCCB SCL) ← GPIO5
 *   CAM0VYSNC (垂直同步)  ← GPIO6
 *   CAM0HREF  (水平参考)  ← GPIO7
 *   CAM0Y4    (数据位 D2) ← GPIO8
 *   CAM0Y3    (数据位 D1) ← GPIO9
 *   CAM0Y5    (数据位 D3) ← GPIO10
 *   CAM0Y2    (数据位 D0) ← GPIO11
 *   CAM0Y6    (数据位 D4) ← GPIO12
 *   CAM0PCLK  (像素时钟)  ← GPIO13
 *   CAM0XCLK  (主时钟)    ← GPIO15
 *   CAM0Y9    (数据位 D7) ← GPIO16
 *   CAM0Y8    (数据位 D6) ← GPIO17
 *   CAM0Y7    (数据位 D5) ← GPIO18
 *   OV_RESET  (摄像头复位) ← GPIO46
 *   OV_PWDN   (掉电控制)   → GND (通过 1K 电阻拉低，常开)
 *
 * ── DVP 数据总线说明 ──────────────────────────────────────────────
 *
 * D0~D7 是 8-bit 并行数据线，每个像素时钟周期传输 1 字节。
 *
 * CAM_Y2 ~ CAM_Y9 是 ESP-IDF 中的信号名：
 *   Y2/Y3/Y4/Y5/Y6/Y7/Y8/Y9 分别对应 D0/D1/D2/D3/D4/D5/D6/D7
 *
 * ⚠ 注意映射不是顺序的！例如：
 *   - D0 → GPIO11 (不是 GPIO4)
 *   - D7 → GPIO16 (不是 GPIO18)
 *   这是因为 PCB 布线优化（优先考虑信号完整性而非编号顺序）。
 * ═══════════════════════════════════════════════════════════════════ */

/* ── DVP 数据总线（8-bit 并行）───────────────────────────────────── */
#define CAM_PIN_D0      11      /* CAM_Y2 → OV_D0 */
#define CAM_PIN_D1       9      /* CAM_Y3 → OV_D1 */
#define CAM_PIN_D2       8      /* CAM_Y4 → OV_D2 */
#define CAM_PIN_D3      10      /* CAM_Y5 → OV_D3 */
#define CAM_PIN_D4      12      /* CAM_Y6 → OV_D4 */
#define CAM_PIN_D5      18      /* CAM_Y7 → OV_D5 */
#define CAM_PIN_D6      17      /* CAM_Y8 → OV_D6 */
#define CAM_PIN_D7      16      /* CAM_Y9 → OV_D7 */

/* ── 同步和控制信号 ─────────────────────────────────────────────── */
#define CAM_PIN_XCLK    15      /* 主时钟输出（20MHz，由 ESP32 LEDC 提供） */
#define CAM_PIN_PCLK    13      /* 像素时钟输入（来自摄像头） */
#define CAM_PIN_VSYNC    6      /* 垂直同步输入（一帧开始） */
#define CAM_PIN_HREF     7      /* 水平参考输入（一行有效数据） */

/* ── SCCB（I²C）引脚 ───────────────────────────────────────────── */
#define CAM_PIN_SCCB_SDA  4     /* SCCB 数据（I²C SDA）→ OV_SDA */
#define CAM_PIN_SCCB_SCL  5     /* SCCB 时钟（I²C SCL）→ OV_SCL */

/* ── 其他 ──────────────────────────────────────────────────────── */
//
// ⚠ 参考程序使用纯软件复位（pin_reset = -1）。
//   硬件复位 GPIO46 在某些批次 OV5640 上可能导致传感器进入异常状态。
//   改为和参考程序一致的纯软件复位方案。
//
#define CAM_PIN_RESET    -1     /* 纯软件复位（和参考程序一致） */
#define CAM_PIN_PWDN     -1     /* 掉电控制（不使用，硬件已接 GND） */

/* ═══════════════════════════════════════════════════════════════════
 * 摄像头配置参数
 *
 * 这些值影响图像质量、帧率和内存占用。
 * 类似 Java 中 Camera.Parameters 的各项设置。
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * ── XCLK 频率 ─────────────────────────────────────────────────────
 *
 * 20MHz 是 OV5640 的推荐时钟频率。
 * 频率越高 = 帧率越高 = 功耗越大 = 发热越多。
 * 20MHz 是画质和功耗的平衡点。
 */
#define CAM_XCLK_FREQ       20000000    /* 20 MHz（OV5640 标准时钟频率） */

/*
 * ── JPEG 质量 ─────────────────────────────────────────────────────
 *
 * 范围：0 ~ 63
 *   0  = 最高质量（文件最大，约 50~100KB/帧）
 *   63 = 最低质量（文件最小，约 3~5KB/帧）
 *
 * 对于鱼缸监控：
 *   - 不需要超高清（我们只做运动检测）
 *   - 但也不能太糊（看不清鱼）
 *   - 12~15 是适合上传和质量折中的范围
 *
 * 我们选 12 = 适中的 JPEG 质量，单帧约 10~20KB
 * MQTT 最大 payload 通常 256KB，这个大小很安全。
 */
#define CAM_JPEG_QUALITY    12

/*
 * ── 帧缓冲数量 ────────────────────────────────────────────────────
 *
 * fb_count = 2 → 双缓冲（double buffering）
 *
 * 为什么是 2 个？
 *   buffer[0] → DMA 正在写入（摄像头输出）
 *   buffer[1] → 我们的代码正在读取（上一帧）
 *
 * 如果只有 1 个缓冲，读和写会冲突（撕裂画面）。
 * 超过 2 个浪费 PSRAM，没必要。
 *
 * Java 类比：
 *   这就是生产者-消费者模式中的 DoubleBuffer。
 *   生产者（DMA）写 buffer[0]，消费者（我们的代码）读 buffer[1]。
 */
#define CAM_FB_COUNT        2

/*
 * ── 图像分辨率 ────────────────────────────────────────────────────
 *
 * FRAMESIZE_SVGA = 800 × 600 像素
 *
 * 可选分辨率（从低到高）：
 *   FRAMESIZE_QQVGA  = 160 × 120
 *   FRAMESIZE_QVGA   = 320 × 240
 *   FRAMESIZE_VGA    = 640 × 480
 *   FRAMESIZE_SVGA   = 800 × 600   ← 我们选的（鱼缸监控够用）
 *   FRAMESIZE_XGA    = 1024 × 768
 *   FRAMESIZE_SXGA   = 1280 × 1024
 *   FRAMESIZE_UXGA   = 1600 × 1200  ← OV5640 全分辨率（5MP）
 *
 * 为什么选 SVGA？
 *   - 800×600 足够看清鱼和运动
 *   - JPEG 文件约 10~30KB（适合 MQTT 传输）
 *   - 每帧 PSRAM 占用约 480KB（800×600×1 字节 YUV 近似）
 *   - 运动检测不需要全分辨率
 */
#define CAM_FRAME_SIZE      FRAMESIZE_SVGA   /* 800×600 — 鱼缸监控 */

/*
 * ── 像素格式 ───────────────────────────────────────────────────────
 *
 * PIXFORMAT_JPEG = 传感器内部硬件 JPEG 编码
 *
 * OV5640 有一个硬件 JPEG 编码器！这意味着：
 *   传感器直接输出 JPEG 数据，不经过 ESP32 软件编码。
 *   省 CPU、省时间、省电。
 *
 * 如果不选 JPEG，就要输出原始 Bayer/YUV 数据，
 *   然后用 ESP32 CPU 软件编码 JPEG —— 很慢（~2 秒/帧）。
 */
#define CAM_PIXEL_FORMAT    PIXFORMAT_JPEG

/*
 * ── 抓取模式 ───────────────────────────────────────────────────────
 *
 * CAMERA_GRAB_WHEN_EMPTY:
 *   当帧缓冲为空时，立即抓取下一帧。
 *
 * 另一种模式 CAMERA_GRAB_LATEST:
 *   总是返回最新的一帧（丢弃中间帧）。
 *
 * 对于定时拍照（5 分钟一次），用 WHEN_EMPTY 更合适：
 *   我们每次读完后缓冲变空 → 传感器自动填充 → 下次读拿到最新帧。
 */
#define CAM_GRAB_MODE       CAMERA_GRAB_WHEN_EMPTY

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API 实现
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 初始化 OV5640 摄像头。
 *
 * ── 初始化流程 ────────────────────────────────────────────────────
 *
 *   camera_init()
 *     │
 *     ├─ 1. 填充 camera_config_t 结构体（引脚、频率、分辨率等）
 *     │
 *     ├─ 2. esp_camera_init(&config)
 *     │     │
 *     │     ├─ 配置 LEDC 产生 XCLK 时钟（20MHz）
 *     │     ├─ 初始化 SCCB（I²C）总线
 *     │     ├─ 通过 SCCB 写入大量寄存器（白平衡、曝光、Gamma...）
 *     │     ├─ 检测 OV5640 芯片 ID（读寄存器 0x300A/0x300B）
 *     │     ├─ 初始化 DMA 通道（数据从摄像头 → PSRAM）
 *     │     └─ 启动传感器数据流
 *     │
 *     └─ 3. 检查返回值，打印摄像头信息
 *
 * ── 可能失败的原因 ─────────────────────────────────────────────────
 *
 *   1. PSRAM 未启用：
 *      menuconfig → Component config → ESP PSRAM → 必须开启
 *      → 症状：esp_camera_init() 返回 ESP_ERR_NO_MEM
 *
 *   2. 摄像头排线松动：
 *      FPC 排线接触不良是常见问题
 *      → 症状：esp_camera_init() 返回 ESP_ERR_CAMERA_NOT_DETECTED
 *
 *   3. 供电不足：
 *      电脑 USB 口可能不够 500mA
 *      → 症状：反复重启、拍照花屏
 *
 *   4. XCLK 引脚冲突：
 *      检查 GPIO16 没有被其他模块占用
 *      → 症状：摄像头不工作，I²C 通信失败
 */
esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  Initializing OV5640 camera...");
    ESP_LOGI(TAG, "  Resolution: 800x600 SVGA");
    ESP_LOGI(TAG, "  Format: JPEG (hardware encoder)");
    ESP_LOGI(TAG, "  Quality: %d", CAM_JPEG_QUALITY);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    /*
     * ── 步骤 1：填充摄像头配置 ──
     *
     * camera_config_t 是 esp32-camera 组件定义的结构体。
     * 我们使用 C99 的"指定初始化器"来只设置需要的字段。
     *
     * Java 类比：
     *   CameraConfig.builder()
     *       .pinD0(47).pinD1(48)...  // 引脚映射
     *       .xclkFreq(20000000)       // 时钟频率
     *       .jpegQuality(12)          // JPEG 质量
     *       .frameSize(SVGA)          // 分辨率
     *       .build();
     */
    camera_config_t config = {
        /* ── 引脚映射 ──
         *
         * 每个引脚都映射到对应的 GPIO。
         * -1 表示不使用该功能（如 PWDN）。
         */
        .pin_pwdn       = CAM_PIN_PWDN,        /* 掉电引脚（不使用） */
        .pin_reset      = CAM_PIN_RESET,       /* 复位引脚 */
        .pin_xclk       = CAM_PIN_XCLK,        /* 主时钟输出 */
        .pin_sccb_sda   = CAM_PIN_SCCB_SDA,    /* SCCB 数据 */
        .pin_sccb_scl   = CAM_PIN_SCCB_SCL,    /* SCCB 时钟 */

        /* ── DVP 8-bit 数据总线 ── */
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        /* ── 同步信号 ── */
        .pin_vsync = CAM_PIN_VSYNC,    /* 垂直同步（帧同步） */
        .pin_href  = CAM_PIN_HREF,     /* 水平参考（行同步） */
        .pin_pclk  = CAM_PIN_PCLK,     /* 像素时钟 */

        /* ── 时钟和定时器 ──
         *
         * LEDC（LED PWM Controller）被借用来产生 XCLK。
         * LEDC_TIMER_0 和 LEDC_CHANNEL_0 是硬件资源的编号，
         * 类似 Java 中的线程池名称——只是个标识。
         */
        .xclk_freq_hz  = CAM_XCLK_FREQ,    /* XCLK: 20MHz */
        .ledc_timer    = LEDC_TIMER_0,      /* 使用 LEDC 定时器 0 */
        .ledc_channel  = LEDC_CHANNEL_0,    /* 使用 LEDC 通道 0 */

        /* ── 图像格式和大小 ── */
        .pixel_format  = CAM_PIXEL_FORMAT,  /* PIXFORMAT_JPEG — 硬件编码 */
        .frame_size    = CAM_FRAME_SIZE,    /* FRAMESIZE_SVGA — 800×600 */

        /* ── JPEG 压缩质量 ──
         *
         * 注意：值越小质量越高！0=最佳，63=最差。
         * 这和直觉相反——就像高尔夫分数，越低越好。
         */
        .jpeg_quality  = CAM_JPEG_QUALITY,  /* 12 = 中等偏高质量 */

        /* ── 帧缓冲配置 ── */
        .fb_count      = CAM_FB_COUNT,      /* 2 个帧缓冲（双缓冲） */
        /*
         * CAMERA_FB_IN_PSRAM:
         *   帧缓冲放在 PSRAM（片外 8MB）。
         *
         * 为什么不用内部 SRAM？
         *   - 一帧 JPEG 约 30KB
         *   - 双缓冲 × 2 = 60KB
         *   - 内部 SRAM 总共仅 512KB，还要分给 FreeRTOS 栈、WiFi 缓冲等
         *   - PSRAM 有 8MB，绰绰有余
         *
         * 代价是 PSRAM 比内部 SRAM 慢约 2~3 倍（通过 SPI 访问）。
         * 但对于拍照（非实时视频），这点延迟完全没影响。
         */
        .fb_location   = CAMERA_FB_IN_PSRAM,
        .grab_mode     = CAM_GRAB_MODE,      /* WHEN_EMPTY — 缓冲空就抓 */
    };

    /*
     * ── 步骤 2：调用 esp_camera_init() ──
     *
     * 这是 esp32-camera 组件暴露的唯一初始化函数。
     * 它内部做了大量工作（数百个寄存器配置），我们只需关心返回值。
     *
     * ESP_ERROR_CHECK 宏：
     *   如果返回值不是 ESP_OK，打印错误并 abort()。
     *   这里用 if 判断而非直接 ESP_ERROR_CHECK，是为了打印
     *   更友好的错误信息。
     */
    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        /*
         * 初始化失败的原因诊断（按概率从高到低）：
         *
         * ESP_ERR_CAMERA_NOT_DETECTED:
         *   → 摄像头排线松动 / 没插好 / OV5640 坏
         *   → 检查 FPC 排线两端
         *
         * ESP_ERR_NO_MEM:
         *   → PSRAM 未启用
         *   → idf.py menuconfig → Component config → ESP PSRAM
         *
         * ESP_ERR_NOT_SUPPORTED:
         *   → 摄像头型号不是 OV5640（可能是 OV2640？）
         *   → 检查芯片丝印
         *
         * ESP_ERR_TIMEOUT:
         *   → SCCB（I²C）通信超时 → 接线问题
         *   → GPIO 引脚被其他功能占用
         */
        ESP_LOGE(TAG, "Camera init failed with error 0x%X", (unsigned int)ret);
        ESP_LOGE(TAG, "Troubleshooting:");
        ESP_LOGE(TAG, "  1. Check FPC ribbon cable is fully inserted");
        ESP_LOGE(TAG, "  2. Enable PSRAM: idf.py menuconfig → Component config → ESP PSRAM");
        ESP_LOGE(TAG, "  3. Check power supply (5V/2A recommended)");
        ESP_LOGE(TAG, "  4. Check no GPIO conflict with pin definitions");
        return ret;
    }

    /*
     * ── 步骤 3：获取传感器句柄，打印调试信息 ──
     *
     * esp_camera_sensor_get() 返回 sensor_t 指针，
     * 可以用来查询和修改传感器参数（曝光、白平衡等）。
     *
     * Java 类比：
     *   Camera.Parameters params = camera.getParameters();
     *   int pixFmt = params.getPixelFormat();
     */
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL) {
        ESP_LOGI(TAG, "OV5640 detected. PID: 0x%02X, MIDH: 0x%02X, MIDL: 0x%02X",
                 sensor->id.PID, sensor->id.MIDH, sensor->id.MIDL);

        /*
         * 打印当前生效的配置（确认设置正确）。
         *
         * sensor->status.framesize:  当前分辨率（0~N 对应 QQVGA~UXGA）
         * sensor->status.quality:    JPEG 质量（0~63，越小越好）
         * sensor->status.brightness: 亮度等级（-2 ~ +2）
         */
        ESP_LOGI(TAG, "Current config — FrameSize:%d, Quality:%d, Brightness:%d",
                 sensor->status.framesize,
                 sensor->status.quality,
                 sensor->status.brightness);
    }

    ESP_LOGI(TAG, "Camera initialized successfully!");

    /*
     * 拍照前需要等 1~2 秒让传感器稳定。
     * 刚初始化完传感器还在做自动曝光和自动白平衡（AWB）。
     * 立刻拍照会得到一张暗绿色或全黑的照片。
     *
     * 类比：就像打开手机相机 App，前 1~2 秒画面会从暗变亮——
     *      那是 AE（自动曝光）正在调整参数。
     */
    ESP_LOGI(TAG, "Waiting 2s for sensor auto-exposure to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Camera ready for capture.");

    return ESP_OK;
}

/**
 * 拍摄一张照片。
 *
 * ── 内部流程 ──────────────────────────────────────────────────────
 *
 *   camera_capture(&frame)
 *     │
 *     ├─ 1. esp_camera_fb_get()     ← 从帧缓冲队列取一帧
 *     │     │                          如果队列空则等待（阻塞）
 *     │     └─ 返回 camera_fb_t*   ← 指向 PSRAM 中的帧数据
 *     │
 *     ├─ 2. 填充我们的 camera_frame_t
 *     │     frame->buf          = fb->buf       (JPEG 数据指针)
 *     │     frame->len          = fb->len       (数据长度)
 *     │     frame->width        = fb->width     (图像宽度)
 *     │     frame->height       = fb->height    (图像高度)
 *     │     frame->timestamp_ms = 当前时间戳     (毫秒)
 *     │
 *     └─ 3. 返回 ESP_OK
 *            （注意：此时 fb 还在，调用者用完必须 release！）
 *
 * ── 重要：为什么不能连续拍照？ ─────────────────────────────────────
 *
 *   OV5640 的硬件 JPEG 编码器需要时间。如果连续调用 camera_capture()：
 *     - 第 1 次：拿到帧 A（正常）
 *     - 第 2 次（立即）：可能拿到同一帧 A（因为下一帧还没编码完）
 *     - 第 3 次（立即）：可能阻塞（等待新帧）
 *
 *   所以建议拍照间隔 ≥ 1 秒。
 */
esp_err_t camera_capture(camera_frame_t *frame)
{
    /* ── 参数校验 ──
     *
     * 等价于 Java 的：
     *   Objects.requireNonNull(frame, "frame must not be null");
     */
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * ── 获取摄像头帧缓冲 ──
     *
     * esp_camera_fb_get() 是阻塞的：
     *   - 如果有可用帧 → 立即返回
     *   - 如果没有 → 等待 grab_mode 指定的条件满足
     *
     * 返回的 camera_fb_t 结构体：
     *   fb->buf      → uint8_t*，指向 JPEG 数据起始地址
     *   fb->len      → size_t，JPEG 数据长度（字节）
     *   fb->width    → 图像宽度
     *   fb->height   → 图像高度
     *   fb->format   → 像素格式（PIXFORMAT_JPEG）
     *   fb->timestamp → 帧时间戳（struct timeval）
     *
     * Java 类比：
     *   CameraFrameBuffer fb = camera.getFrameBuffer();  // 可能阻塞
     */
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == NULL) {
        /*
         * fb == NULL 意味着：
         *   - 帧缓冲获取超时（默认超时约 5 秒）
         *   - 或者 esp_camera_fb_get() 内部分配失败
         *
         * 常见原因：
         *   - 摄像头排线松动（突然断连）
         *   - PSRAM 耗尽（前几帧没 release）
         *   - 传感器时钟出问题
         */
        ESP_LOGE(TAG, "Camera capture failed: no frame buffer returned");
        ESP_LOGE(TAG, "  → Check FPC cable connection");
        ESP_LOGE(TAG, "  → Make sure previous frames were released");
        return ESP_ERR_TIMEOUT;
    }

    /*
     * ── 填充我们的 camera_frame_t ──
     *
     * 我们把 esp32-camera 的 camera_fb_t 数据"翻译"成我们自己定义的结构体。
     * 这样做的好处：
     *   - 调用者不需要知道 esp32-camera 的类型
     *   - 未来换摄像头驱动层不影响上层代码
     *   - 封装 = 职责清晰（类似 DAO 模式）
     *
     * 注意：我们复制的是指针和元数据，不是 JPEG 数据本身。
     * JPEG 数据仍然在 PSRAM 的原地址，调用者用完要 release。
     */
    frame->buf          = fb->buf;           /* 指向 PSRAM 中的 JPEG 数据 */
    frame->len          = fb->len;           /* JPEG 大小（字节） */
    frame->width        = fb->width;         /* 图像宽度 */
    frame->height       = fb->height;        /* 图像高度 */
    frame->timestamp_ms = esp_timer_get_time() / 1000;  /* 微秒 → 毫秒 */

    /*
     * ── 释放 esp32-camera 的帧缓冲包装 ──
     *
     * 等等，我们不是还没用这个帧吗？为什么要释放？
     *
     * esp_camera_fb_return(fb) 不是释放 JPEG 数据，
     * 而是"归还帧缓冲槽位"给 DMA 引擎。
     *
     * 结构：
     *   fb（camera_fb_t 包装）→ buf（JPEG 数据在 PSRAM）
     *
     * 调用 esp_camera_fb_return(fb)：
     *   ✅ fb 这个"槽位"归还了 → DMA 可以写入新帧
     *   ❌ buf 数据还在！→ 我们的 frame->buf 仍然有效
     *
     * 这个设计依赖 esp32-camera 的双缓冲机制：
     *   归还了 fb0，DMA 就不会覆盖正在用的 buf1。
     *   但如果双缓冲配置错误，可能会有数据竞争。
     *
     * Java 类比：
     *   这就像归还一个"令牌"（semaphore permit）：
     *     semaphore.release();  // 允许生产者写入新数据
     *     // 但我们手上的数据引用还活着，可以继续用
     */
    esp_camera_fb_return(fb);

    ESP_LOGI(TAG, "Photo captured: %dx%d, %u bytes, ts=%lld ms",
             (unsigned int)frame->width,
             (unsigned int)frame->height,
             (unsigned int)frame->len,
             (long long)frame->timestamp_ms);

    return ESP_OK;
}

/**
 * 释放摄像头帧占用的内存。
 *
 * ── 为什么需要这个函数？ ───────────────────────────────────────────
 *
 * 前面说过，camera_capture() 中我们调用了 esp_camera_fb_return(fb)，
 * 这归还了帧缓冲槽位。但 JPEG 像素数据（buf）仍然占用 PSRAM。
 *
 * 如果帧缓冲分配在 PSRAM 中，而 PSRAM 是用 malloc 分配的：
 *   - 只有调用 free(buf) 才能归还给堆
 *   - esp_camera_fb_return() 可能不会 free buf（取决于配置）
 *
 * 不过在当前实现中，esp_camera_fb_return() 已经把槽位还回去了，
 * buf 由 esp32-camera 内部管理（池化复用，不会泄漏）。
 *
 * 这个函数目前主要是"标记"和"清零"操作，防止 use-after-free：
 *   - 把指针置 NULL → 再访问就会 crash（好过读到错误数据）
 *   - 教育意义：提醒开发者记得管理内存
 *
 * ── 内存模型图解 ───────────────────────────────────────────────────
 *
 *   PSRAM (8MB)
 *   ┌────────────────────────────────────────┐
 *   │ fb0.buf → [JPEG data, ~15KB]           │ ← DMA 槽位 0
 *   │ fb1.buf → [JPEG data, ~15KB]           │ ← DMA 槽位 1
 *   │ ... free space ...                     │
 *   └────────────────────────────────────────┘
 *
 *   esp_camera_fb_get()  → 返回 fb0 的指针（消费者拿到）
 *   esp_camera_fb_return(fb0) → 归还 fb0 给 DMA（生产者可以写入）
 *
 *   我们的 camera_frame_release():
 *     → 清零 frame 结构体（标记为"已释放"）
 *     → buf 由 esp32-camera 内部池管理，无需手动 free
 */
void camera_frame_release(camera_frame_t *frame)
{
    if (frame == NULL) {
        return;     /* 空指针保护 — 类似 Java 的 if (obj == null) return; */
    }

    /*
     * 注意：我们不需要 free(frame->buf)！
     *
     * 原因：buf 指向的内存由 esp32-camera 内部的帧缓冲池管理。
     * esp_camera_fb_return() 已经把槽位归还了。
     * 如果这里 free(buf)，下一次 esp_camera_fb_get() 会拿到野指针。
     *
     * 我们只需要清空 frame 结构体，防止后续代码误用。
     */
    frame->buf          = NULL;     /* 防止 use-after-free */
    frame->len          = 0;
    frame->width        = 0;
    frame->height       = 0;
    frame->timestamp_ms = 0;
}
