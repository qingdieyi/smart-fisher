/*
 * Smart Fisher — 鱼缸智能监控系统 主程序
 * ==============================================
 *
 * ── 给 Java 开发者的话 ─────────────────────────────────────────────
 *
 * 如果你从 Java/Spring Boot 转过来，以下是一些关键区别：
 *
 *   概念                  Java / Spring              C / ESP-IDF
 *   ─────────────────────────────────────────────────────────────────
 *   程序入口              public static void main    void app_main(void)
 *   并发模型              多线程 + 线程池             FreeRTOS 任务 (Task)
 *   线程间通信            BlockingQueue / Future     EventGroup / Queue
 *   依赖注入              @Autowired / @Bean          手动 init，顺序调用
 *   日志                  SLF4J / logback             ESP_LOGx 宏
 *   配置                  application.yml             Kconfig / menuconfig
 *   构建工具              Maven / Gradle              CMake + idf.py
 *   包管理                Maven Central               ESP-IDF Component Registry
 *   异常处理              try-catch / throw           返回错误码 (esp_err_t)
 *   内存管理              GC 自动回收                 手动管理（全局/静态变量或 malloc）
 *
 * ── FreeRTOS 任务 vs Java 线程 ─────────────────────────────────────
 *
 *   FreeRTOS 任务是抢占式的，类似 Thread，但有一些区别：
 *     - 没有 Thread.stop() —— 任务自己 return 或 vTaskDelete() 自己
 *     - 任务优先级：数字越大优先级越高（Java 是越小越高，注意反过来）
 *     - 栈大小需要手动指定（Java 栈自动增长）
 *     - 没有垃圾回收 —— 任务用完的内存需要 free() 或干脆用静态分配
 *
 * ── 当前阶段 ────────────────────────────────────────────────────────
 *
 *   ✅ Phase 1 完成 — WiFi (WPA2-PSK) + DS18B20 温度采集
 *   ⬜ Phase 2 — 摄像头 (OV5640)
 *   ⬜ Phase 3 — MQTT 上报
 *   ⬜ Phase 4 — 鱼群检测
 */

/* ── 头文件引入 ─────────────────────────────────────────────────── */

#include <stdio.h>            /* 标准 I/O（printf 等），这里主要和 ESP_LOGI 配合 */

/* FreeRTOS 相关 — 实时操作系统的核心 API */
#include "freertos/FreeRTOS.h"       /* FreeRTOS 核心类型和宏 */
#include "freertos/task.h"           /* 任务创建/管理：xTaskCreate, vTaskDelay */
#include "freertos/event_groups.h"   /* 事件组：任务间同步的一种方式 */

/* ESP-IDF 组件 */
#include "esp_wifi.h"         /* WiFi 驱动（STA/AP/扫描等） */
#include "esp_event.h"        /* ESP-IDF 事件循环（类似 Android 的 EventBus / BroadcastReceiver） */
#include "esp_log.h"          /* 日志系统 */
#include "esp_system.h"       /* 系统级操作（esp_restart 重启等） */
#include "nvs_flash.h"        /* NVS = Non-Volatile Storage（类似 Android 的 SharedPreferences）
                               * ESP32 用 Flash 的一部分来模拟"键值对存储"，
                               * WiFi 凭证就存在这里 */
#include "esp_netif.h"        /* 网络接口抽象层（类似 Java 的 NetworkInterface） */

/* 我们自己写的模块 */
#include "ds18b20.h"           /* DS18B20 温度传感器驱动 */
#include "camera_handler.h"    /* OV5640 摄像头驱动 */

/* ═══════════════════════════════════════════════════════════════════
 * 配置常量
 *
 * 在 Java 中通常会放在 application.yml 里。
 * 在 ESP-IDF 中，编译时配置用 #define 或 Kconfig（menuconfig 界面）。
 * 这里用 #define 写死，因为 WiFi 凭证不会频繁改动。
 *
 * 如果你想做成可配置的（通过 idf.py menuconfig 修改），
 * 可以把这些改成 CONFIG_EXAMPLE_WIFI_SSID（在 Kconfig.projbuild 中定义）。
 * ═══════════════════════════════════════════════════════════════════ */

#define WIFI_SSID      "yang"           /* WiFi 名称（2.4GHz 频段） */
#define WIFI_PASSWORD  "yang123456"     /* WiFi 密码（至少 8 位） */
#define WIFI_MAX_RETRY 5                /* 重连次数上限，超过后自动重启 */

/*
 * ESP32-S3-CAM R16N8 引脚分配说明：
 *
 * 开发板的大部分 GPIO 已经被摄像头占用了。
 * GPIO4 是少数空闲的引脚之一，适合接 DS18B20。
 *
 * 注意：GPIO4 同时也是板载 LED 闪光灯的控制引脚。
 * 后期开启摄像头时，可能需要改用其他空闲引脚（如 GPIO5/GPIO6）。
 */
#define TEMP_SENSOR_GPIO  GPIO_NUM_1    /* DS18B20 数据线接 GPIO1（GPIO4 已被摄像头 SCCB SDA 占用） */

/*
 * 摄像头拍照间隔（毫秒）。
 *
 * 开发阶段用 60 秒（方便调试），
 * 正式部署改为 300000（5 分钟），减少功耗和上传流量。
 */
#define CAMERA_CAPTURE_INTERVAL_MS  60000   /* 60 秒 = 1 分钟 */

/* ═══════════════════════════════════════════════════════════════════
 * 全局变量
 *
 * 在 Java 中，这些通常是类的 static 字段。
 * 在 C 的嵌入式项目里，简单场景下用文件级 static 变量就够了。
 *
 * static 在 C 文件级的作用：
 *   限制变量的可见范围为当前 .c 文件。
 *   类似 Java 的 private 修饰符 —— 其他 .c 文件看不到这个变量。
 * ═══════════════════════════════════════════════════════════════════ */

/* 日志标签 —— 类似 Java 的 Logger name */
static const char *TAG = "smart-fisher";

/*
 * WiFi 事件组 — FreeRTOS 的"条件变量"
 *
 * EventGroup 是 FreeRTOS 提供的一种任务间同步机制。
 * 每个 bit 代表一个事件的状态。
 * 一个任务可以阻塞等待某个 bit 被设置（类似 Java 的 CountDownLatch.await()）。
 * 另一个任务在事件发生时设置对应的 bit（类似 CountDownLatch.countDown()）。
 *
 * 我们用它来通知其他任务"WiFi 连上了"。
 */
static EventGroupHandle_t wifi_event_group;

/*
 * EventGroup 中的 bit 定义。
 * BIT0 = (1 << 0) = 0b0001
 * 一个 32-bit 的 EventGroup 可以表达 24 种不同的事件（高 8 位保留）。
 * 我们只需要一个事件：WiFi 是否连上。
 */
const int WIFI_CONNECTED_BIT = BIT0;

/* WiFi 重连计数器 —— 记录断线重试了几次 */
static int wifi_retry_count = 0;

/* ═══════════════════════════════════════════════════════════════════
 * WiFi 事件处理回调
 *
 * ESP-IDF 的 WiFi 子系统采用事件驱动模型。工作流程：
 *
 *   1. 注册回调函数（类似 Android 的 registerReceiver）
 *   2. WiFi 底层状态变化时，向事件循环发送事件
 *   3. 事件循环调用我们注册的回调
 *
 * ── 事件类型 ─────────────────────────────────────────────────────
 *
 *   WIFI_EVENT_STA_START       → WiFi STA 模式已启动，可以开始连接
 *   WIFI_EVENT_STA_DISCONNECTED → 断线了（原因在 event_data 里）
 *   IP_EVENT_STA_GOT_IP        → 已获取 IP 地址（这时才算真正能上网）
 *
 * Java 类比：
 *   @EventListener
 *   public void onWifiEvent(WifiEvent event) {
 *       switch (event.getType()) {
 *           case STA_START: wifi.connect(); break;
 *           case DISCONNECTED: reconnect(); break;
 *           case GOT_IP: onConnected(event.getIp()); break;
 *       }
 *   }
 * ═══════════════════════════════════════════════════════════════════ */

static void wifi_event_handler(void *arg,                 /* 注册时传的自定义参数 */
                               esp_event_base_t event_base, /* 事件大类（WIFI_EVENT / IP_EVENT）*/
                               int32_t event_id,            /* 具体事件 ID */
                               void *event_data)            /* 事件附带数据 */
{
    /* ── 事件：WiFi STA 模式已就绪 ── */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /*
         * WiFi 驱动启动完成，可以开始扫描和连接了。
         * 类似："网卡驱动加载完毕，自动尝试连接"。
         */
        esp_wifi_connect();
    }

    /* ── 事件：WiFi 断线 ── */
    else if (event_base == WIFI_EVENT
             && event_id == WIFI_EVENT_STA_DISCONNECTED) {

        /* event_data 包含了断线原因（信号差？密码错？AP 重启？） */
        wifi_event_sta_disconnected_t *ev =
            (wifi_event_sta_disconnected_t *)event_data;

        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();        /* 立即重连 */
            wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected (reason=%d), retrying (%d/%d)...",
                     ev->reason, wifi_retry_count, WIFI_MAX_RETRY);
            /*
             * ESP_LOGW = WARNING 级别日志
             * 类似 Java 的 log.warn("WiFi disconnected, retrying {}/{}", retry, max)
             *
             * 格式说明符：
             *   %d — int (带符号整数)
             *   %s — char* (字符串)
             *   %u — unsigned int
             */
        } else {
            /*
             * 重试太多次了，直接重启开发板。
             * 这就像 Spring Boot 的 actuator/restart。
             * 重启后 nvs_flash_init() 会记住之前的配置，所以 WiFi 凭证还在。
             */
            ESP_LOGE(TAG, "WiFi max retries reached. Rebooting...");
            esp_restart();             /* 软重启 —— 不掉电，MCU 重新从 app_main() 开始 */
        }

        /*
         * 清除 CONNECTED_BIT。
         * 这样等待 WiFi 的任务（如 temperature_task）会阻塞，
         * 直到 WiFi 重新连上后再次设置这个 bit。
         */
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }

    /* ── 事件：获取到 IP 地址 ── */
    else if (event_base == IP_EVENT
             && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;

        wifi_retry_count = 0;    /* 重置重试计数器 */

        /*
         * 设置 CONNECTED_BIT，通知所有等待的任务："网络 OK 了！"
         * 类似 CountDownLatch.countDown()。
         */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        /*
         * IPSTR 和 IP2STR 是 ESP-IDF 的宏，用于打印 IP 地址：
         *   IPSTR   = "%d.%d.%d.%d"（格式化字符串）
         *   IP2STR  = 把 ip_addr_t 拆成 4 个 uint8_t 参数
         *
         * 例：IP:192.168.1.100
         */
        ESP_LOGI(TAG, "WiFi connected. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * WiFi 初始化（Station 模式 — 连接路由器）
 *
 * ESP32 有两种 WiFi 模式：
 *   STA (Station)：  连接到一个已有的 WiFi 路由器（类似手机连家里的 WiFi）
 *   AP (Access Point)：自己作为热点让其他设备连接
 *
 * 我们只需要 STA 模式。
 *
 * Java 类比：
 *   这相当于在 Spring Boot 中配置一个 @Bean HttpClient
 *   + 注册 connection listener。
 * ═══════════════════════════════════════════════════════════════════ */

static void wifi_init_sta(void)
{
    /*
     * ── 步骤 1：初始化 TCP/IP 协议栈 ──
     *
     * esp_netif_init()     → 初始化网络接口抽象层
     * esp_event_loop_create_default() → 创建默认事件循环（类似 Android Looper.prepareMainLooper()）
     * esp_netif_create_default_wifi_sta() → 创建 WiFi STA 网络接口
     *
     * 这些调用顺序很重要，类似 Spring 中 Bean 的初始化顺序。
     */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* ── 步骤 2：初始化 WiFi 硬件 ── */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    /*
     * WIFI_INIT_CONFIG_DEFAULT() 是一个宏，返回默认的 WiFi 初始化配置。
     * 类似 Java 的 Builder 模式的 .defaults().build()。
     */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ── 步骤 3：注册事件回调 ──
     *
     * 告诉 ESP-IDF："当 WiFi 或 IP 状态变化时，调用我的回调函数"。
     *
     * ESP_EVENT_ANY_ID = 监听这个大类下的所有事件
     * 类似：
     *   eventBus.register(WifiEvent.class, this::wifi_event_handler);
     *   eventBus.register(IpEvent.class, this::wifi_event_handler);
     */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                     ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                     IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* ── 步骤 4：配置并启动 WiFi ── */
    wifi_config_t wifi_config = {
        /*
         * C 语言的"指定初始化器"（Designated Initializer）—— C99 特性。
         * 只初始化你关心的字段，其他字段自动归零。
         *
         * Java 类比：
         *   WifiConfig.builder()
         *       .ssid("yang")
         *       .password("yang123456")
         *       .authMode(WPA2_PSK)
         *       .pmf(new PmfConfig(true, false))
         *       .build();
         */
        .sta = {
            .ssid = WIFI_SSID,         /* 网络名称 */
            .password = WIFI_PASSWORD, /* 网络密码 */

            /*
             * threshold.authmode: 最低接受的认证模式
             * WIFI_AUTH_WPA2_PSK = 我们要求至少 WPA2 级别的加密。
             *
             * 如果你的路由器是 WPA3，ESP32-S3 也支持，
             * 但需要额外配置 SAE（Simultaneous Authentication of Equals）。
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            /*
             * PMF (Protected Management Frames) = 保护管理帧。
             * 802.11w 标准，防止 deauth 攻击（有人故意踢你下线）。
             *   .capable = true    → 我们支持 PMF
             *   .required = false  → 但不强制要求（兼容旧路由器）
             */
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    /*
     * esp_wifi_set_mode(WIFI_MODE_STA)
     *   → 设置为"客户端"模式（连接路由器的模式，而非自己当热点）
     *
     * esp_wifi_set_config() → 把上面填好的配置传给 WiFi 驱动
     *
     * esp_wifi_start() → 启动 WiFi 硬件（开始耗电了）
     *   启动后会自动触发 WIFI_EVENT_STA_START 事件，
     *   我们的 wifi_event_handler 会收到并调用 esp_wifi_connect()。
     */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA initialized. Connecting to SSID: %s", WIFI_SSID);
}

/* ═══════════════════════════════════════════════════════════════════
 * 摄像头拍照任务
 *
 * 这个任务负责：
 *   1. 等待 WiFi 连接
 *   2. 初始化 OV5640 摄像头（配置寄存器、启动数据流）
 *   3. 每隔 CAMERA_CAPTURE_INTERVAL_MS 毫秒拍一张照片
 *   4. 打印照片信息（大小、分辨率、时间戳）
 *
 * ── 为什么任务优先级低于温度任务？ ─────────────────────────────────
 *
 *   优先级：temp_task=3, camera_task=2
 *
 *   拍照涉及大量数据搬运（DMA + PSRAM 写入），耗时较长（~200ms）。
 *   如果摄像头优先级高于温度任务，可能在读温度时被中断，
 *   导致 1-Wire 时序出错（DS18B20 对时序精度要求高）。
 *
 *   所以让温度任务优先级更高：测温优先，拍照靠后。
 *
 * ── 栈大小为什么是 8KB？ ──────────────────────────────────────────
 *
 *   温度任务栈 4KB 就够了（只做 GPIO 操作）。
 *   摄像头任务需要 8KB 因为：
 *     - esp_camera_fb_get() 内部有深调用栈（传感器驱动 → DMA 管理）
 *     - camera_frame_t 结构体（虽然用堆/PSRAM，但函数调用也要栈）
 *     - ESP_LOGI 等日志函数也有栈开销
 *   8KB 是比较安全的值。如果栈溢出，日志会看到 "Stack canary" 错误。
 *
 * Java 类比：
 *   @Scheduled(fixedDelay = 60000)
 *   public void capturePhoto() {
 *       CameraFrame frame = camera.takePicture();
 *       log.info("Photo: {} bytes", frame.getSize());
 *   }
 * ═══════════════════════════════════════════════════════════════════ */

static void camera_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Camera task started");

    /*
     * ── 第 1 步：等待 WiFi 连接 ──
     *
     * 摄像头的初始化涉及 PSRAM 分配（需要系统就绪）。
     * 统一在拿到 WIFI_CONNECTED_BIT 后再初始化所有外设，
     * 可以避免竞态条件（race condition）。
     */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /*
     * ── 第 2 步：初始化摄像头 ──
     *
     * camera_init() 是阻塞的（内部有 vTaskDelay(2000) 等传感器稳定）。
     * 整个过程约 3~5 秒 —— 大部分时间在等传感器自动曝光收敛。
     *
     * 如果初始化失败，摄像头任务进入错误循环：
     *   - 每 30 秒重试一次（给传感器时间恢复）
     *   - 不影响温度任务（独立任务，各自运行）
     *
     * 这种"优雅降级"设计在嵌入式系统中非常重要：
     *   摄像头坏了 ≠ 整个鱼缸监控坏了
     *   温度采集仍然正常，串口日志记录错误供诊断。
     */
    esp_err_t ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed (err=0x%X). "
                 "Camera task will retry every 30s.", (unsigned int)ret);

        /*
         * ── 错误恢复循环 ──
         *
         * 和温度任务的设计哲学一样：失败了不崩溃，定期重试。
         * 30 秒重试间隔足够长，不会刷屏日志；
         * 也足够短，用户插好排线后半分钟内就能恢复。
         */
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(30000));  /* 等 30 秒 */
            ESP_LOGI(TAG, "Retrying camera init...");
            ret = camera_init();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Camera recovered successfully!");
                break;  /* 跳出重试循环，进入拍照循环 */
            }
            ESP_LOGW(TAG, "Still failing (err=0x%X), will retry...",
                     (unsigned int)ret);
        }
    }

    /*
     * ── 第 3 步：拍照循环 ──
     *
     * 定时拍照 → 打印信息 → 释放帧 → 延时 → 重复。
     *
     * 当前阶段（Phase 2）只做拍照 + 串口输出，
     * 后续 Phase 4 会加上 MQTT 上传，
     * Phase 5 会加上鱼群检测。
     */
    while (1) {
        /*
         * ── 拍照 ──
         *
         * camera_capture() 是阻塞的：
         *   - 等传感器曝光完成
         *   - 读 JPEG 字节流
         *   - 填充 camera_frame_t
         * 整个过程约 100~300ms。
         */
        camera_frame_t frame;
        ret = camera_capture(&frame);

        if (ret == ESP_OK) {
            /*
             * ── 打印照片信息 ──
             *
             * 后续阶段会改为：
             *   mqtt_publish_image(frame.buf, frame.len, frame.timestamp_ms);
             *
             * 现在只是串口输出，验证拍照功能正常。
             *
             * 输出示例：
             *   📷 Photo: 800x600, 15420 bytes, ts=123456ms
             */
            ESP_LOGI(TAG, "📷 Photo: %dx%d, %u bytes, ts=%lld ms",
                     (unsigned int)frame.width,
                     (unsigned int)frame.height,
                     (unsigned int)frame.len,
                     (long long)frame.timestamp_ms);

            /*
             * ── 释放帧缓冲 ──
             *
             * 忘记调用 camera_frame_release() 的后果：
             *   - 第 1 次拍照：正常
             *   - 第 2 次拍照：正常（双缓冲机制）
             *   - 第 3 次拍照：可能阻塞或失败（PSRAM 帧缓冲耗尽）
             *
             * 在嵌入式 C 中，内存泄漏不会像 Java 那样最终被 GC 回收。
             * 8MB PSRAM 看起来很大，但忘记释放 = 肯定会累积泄漏。
             *
             * Java 类比：
             *   frame.release();  // !!! 忘记这行 = 内存泄漏 !!!
             */
            camera_frame_release(&frame);
        } else {
            /*
             * ── 拍照失败处理 ──
             *
             * 运行中的偶发失败通常是瞬时干扰（DMA 总线冲突等），
             * 不尝试重新初始化，等下一个周期自动恢复。
             * 如果连续失败 → 看门狗会重启 → 重新走完整初始化。
             */
            ESP_LOGW(TAG, "Photo capture failed (err=0x%X), will retry next cycle.",
                     (unsigned int)ret);
        }

        /*
         * ── 等待下一个拍照周期 ──
         *
         * vTaskDelay 让任务进入 Blocked 状态，释放 CPU 给其他任务。
         * 这不是忙等（busy-wait）—— CPU 同时执行温度读取、WiFi 维护等。
         */
        vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_INTERVAL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 温度采集任务
 *
 * 这是一个 FreeRTOS 任务（Task），它会作为一个独立的执行流运行。
 *
 * 与 Java 线程的关键区别：
 *
 *   特性              Java Thread              FreeRTOS Task
 *   ──────────────────────────────────────────────────────────────
 *   创建              new Thread(r).start()    xTaskCreate(func, name, stack, ...)
 *   入口方法          run()                    一个普通的 C 函数
 *   栈大小            自动增长（-Xss 默认 1M）    手动指定（4096 字节）
 *   休眠              Thread.sleep(ms)         vTaskDelay(ticks)
 *   优先级            Thread.setPriority()     创建时指定（数字越大优先级越高）
 *   终止              Thread.interrupt()       函数 return 或 vTaskDelete(NULL)
 *   返回值            Future.get()             无（通过 Queue/EventGroup 通信）
 * ═══════════════════════════════════════════════════════════════════ */

static void temperature_task(void *pvParameters)
{
    /*
     * 从参数中提取 GPIO 引脚号。
     *
     * pvParameters 是 void* 类型（类似 Java 的 Object），需要强制转换。
     * (uintptr_t) 先把指针转成整数——这是 C 语言的标准做法，
     * 因为直接从指针转枚举在某些编译器上会报警告。
     */
    gpio_num_t pin = (gpio_num_t)(uintptr_t)pvParameters;
    float temperature = 0.0f;

    ESP_LOGI(TAG, "Temperature task started on GPIO %d", (int)pin);

    /*
     * 阻塞等待 WiFi 连接就绪。
     *
     * xEventGroupWaitBits() 类似 Java 的：
     *   wifiConnectedLatch.await();  // 阻塞直到 latch 归零
     *
     * 参数说明：
     *   wifi_event_group    — 要等待的事件组
     *   WIFI_CONNECTED_BIT  — 等待这个 bit 被设置
     *   pdFALSE             — 等待完成后不清除 bit
     *   pdTRUE              — 必须所有 bit 都设置才返回
     *   portMAX_DELAY       — 无限等待（等价于 await() 不带超时）
     *
     * 如果 WiFi 还没连上，这个任务会在这里一直阻塞，
     * 直到 wifi_event_handler 调用了 xEventGroupSetBits(WIFI_CONNECTED_BIT)。
     */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /*
     * ── 主循环 ──
     *
     * 嵌入式任务的常见模式：while(1) + vTaskDelay()
     * 类似：
     *   @Scheduled(fixedDelay = 5000)  // Spring 的定时任务
     *   public void readTemperature() { ... }
     *
     * 与 Java 定时任务的区别：
     *   - 没有框架帮你管理调度，需要自己写 while(1) 循环
     *   - vTaskDelay 会出让 CPU 给其他任务，不是忙等（busy-wait）
     *   - 如果要把数据发给手机看，这里会 publish 到 MQTT（后续阶段实现）
     */
    while (1) {
        /*
         * 调用我们写的 DS18B20 驱动读取温度。
         *
         * 返回值检查（类似 Java 的 try-catch 或者 Optional 模式）：
         *   - ESP_OK             → 正常，temperature 有有效值
         *   - ESP_ERR_TIMEOUT    → 传感器通信超时（线松了？）
         *   - ESP_ERR_INVALID_CRC → 数据校验失败（电磁干扰？线太长？）
         */
        esp_err_t ret = ds18b20_read_temperature(pin, &temperature);

        if (ret == ESP_OK) {
            /*
             * %.2f = 浮点数，保留 2 位小数
             * 输出示例：🌡️  Water Temperature: 25.50 °C
             */
            ESP_LOGI(TAG, "🌡️  Water Temperature: %.2f °C", temperature);
        } else {
            /*
             * 读取失败不崩溃 —— 这次读不到，5秒后再试。
             * 这体现了嵌入式系统的"容错设计"：单个操作的失败
             * 不应该导致整个系统挂掉。
             *
             * 0x%X = 以十六进制打印错误码
             * 输出示例：Temperature read failed (err=0x103), retrying...
             */
            ESP_LOGW(TAG, "Temperature read failed (err=0x%X), retrying...",
                     (unsigned int)ret);
        }

        /*
         * 阻塞当前任务 5 秒。
         *
         * pdMS_TO_TICKS(5000)：把毫秒转成 FreeRTOS tick 数。
         * 默认配置：1 tick = 10ms → 5000ms = 500 ticks
         *
         * 注意：vTaskDelay 让任务进入 Blocked 状态整整 5 秒。
         * 期间 CPU 可以执行其他任务（如 WiFi 重连、后续的 MQTT 等）。
         * 这就叫 "非阻塞延时"。
         */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 程序入口 — void app_main(void)
 *
 * 这是 ESP-IDF 的 "public static void main(String[] args)"。
 *
 * 区别：
 *   - 没有参数（argc/argv 在嵌入式上没有意义）
 *   - 返回值是 void —— 程序不应该"退出"，而是永远运行
 *   - 如果 app_main() return 了，ESP-IDF 会自动重启开发板
 *
 * main 函数的工作顺序非常重要，类似 Spring Boot 的启动流程：
 *   1. 初始化底层（NVS、WiFi）
 *   2. 连接网络
 *   3. 初始化外设（传感器）
 *   4. 启动业务任务
 * ═══════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    esp_err_t ret;  /* ESP-IDF 函数的标准返回类型，类似 Java 的异常 */

    /* ── 启动 Logo ── */
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  Smart Fisher — Fish Tank Monitor");
    ESP_LOGI(TAG, "  Board: ESP32-S3-CAM R16N8");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    /* ═════════════════════════════════════════════════════════════
     * 阶段 1：NVS 初始化
     *
     * NVS (Non-Volatile Storage) = 非易失性存储。
     * ESP32 的 Flash 存储中划分出一块区域，用作键值对存储。
     *
     * 类比：
     *   - Android SharedPreferences
     *   - Redis / SQLite 的简化版
     *
     * WiFi 库需要 NVS 来存储：
     *   - WiFi SSID 和密码
     *   - 上次连接的 AP 信息（加速下次连接）
     *   - 其他 WiFi 相关的校准数据
     *
     * 为什么需要处理 ESP_ERR_NVS_NO_FREE_PAGES？
     *   如果是新板子或 Flash 被擦除过，NVS 分区是空的。
     *   这时需要先格式化（erase），再重新初始化。
     *   类似：如果 SQLite 数据库文件不存在，先 CREATE DATABASE。
     * ═════════════════════════════════════════════════════════════ */

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES
        || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /*
         * NVS 分区损坏或版本不兼容，擦除后重新初始化。
         * 擦除意味着之前存的 WiFi 凭证会丢失（仅开发阶段无所谓）。
         */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  /* 如果这次还失败，直接 abort */

    /* ═════════════════════════════════════════════════════════════
     * 阶段 2：WiFi 连接
     *
     * 创建 EventGroup → 初始化 WiFi → 等待获取 IP
     *
     * EventGroup 在这里充当"启动屏障"：
     *   app_main 等 WiFi 连上之后，才继续初始化后面的传感器。
     * ═════════════════════════════════════════════════════════════ */

    wifi_event_group = xEventGroupCreate();  /* 类似 new CountDownLatch(1) */
    wifi_init_sta();                          /* 类似 startWifiService() */

    ESP_LOGI(TAG, "Waiting for WiFi connection...");

    /*
     * 阻塞等待 WiFi 连通。
     *
     * 注意：portMAX_DELAY = 无限等待（没有超时）。
     * 如果路由器关机或者密码错了，程序会永远卡在这里。
     * 生产环境中应该加超时 + 降级处理（比如用蓝牙配网）。
     *
     * 目前这样做是合理的：没有网络，后面的工作都白做。
     */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* ═════════════════════════════════════════════════════════════
     * 阶段 3：传感器初始化
     * ═════════════════════════════════════════════════════════════ */

    ret = ds18b20_init(TEMP_SENSOR_GPIO);
    if (ret != ESP_OK) {
        /*
         * 传感器初始化失败，打印错误但程序继续运行。
         *
         * 为什么不像 Java 那样抛出异常？
         *   在嵌入式系统中，"一个外设坏了"不应该让整个系统停摆。
         *   传感器可能晚点才插上（热插拔），WiFi 连接和后续的摄像头
         *   功能都不依赖温度传感器。
         *
         * 这是一种"优雅降级"（Graceful Degradation）的设计理念：
         *   传感器坏了 → 日志报错 → 温度任务会持续重试 → 但程序不挂
         */
        ESP_LOGE(TAG, "DS18B20 init failed. Check wiring and GPIO %d.",
                 (int)TEMP_SENSOR_GPIO);
    }

    /* ═════════════════════════════════════════════════════════════
     * 阶段 4：启动业务任务
     *
     * xTaskCreate() = 创建并启动一个 FreeRTOS 任务
     *
     * 参数说明（按顺序）：
     *   1. temperature_task   — 任务函数（相当于 Java 的 Runnable.run()）
     *   2. "temp_task"        — 任务名称（用于调试和 ps 命令查看）
     *   3. 4096               — 栈大小（字节），类似 Thread 构造函数中的 stackSize
     *                           4096 字节对于温度读取来说足够了。
     *                           如果不够 → StackOverflow → 看门狗复位
     *   4. (void*)(uintptr_t)TEMP_SENSOR_GPIO — 传给任务的参数（GPIO 引脚号）
     *   5. 3                  — 优先级（数字越大优先级越高，0 最低）
     *   6. NULL               — 任务句柄（不需要保存，传 NULL）
     *
     * Java 类比：
     *   Thread tempThread = new Thread(
     *       () -> temperatureTask.run(),
     *       "temp_task"
     *   );
     *   tempThread.setPriority(Thread.NORM_PRIORITY);  // Java 里越低越优先
     *   tempThread.start();
     * ═════════════════════════════════════════════════════════════ */

    xTaskCreate(temperature_task, "temp_task", 4096,
                (void *)(uintptr_t)TEMP_SENSOR_GPIO, 3, NULL);

    /*
     * ── 创建摄像头任务 ──
     *
     * 栈大小 8192 字节（8KB）—— 因为 camera_capture() 调用链较深
     * （esp_camera_fb_get → DMA 管理 → JPEG 解码），需要更多栈空间。
     *
     * 优先级 2 —— 低于温度任务（3），测温比拍照更重要。
     * 参数传 NULL —— 摄像头任务不需要额外参数。
     */
    xTaskCreate(camera_task, "camera_task", 8192,
                NULL, 2, NULL);

    ESP_LOGI(TAG, "System ready. Monitoring fish tank...");

    /* ═════════════════════════════════════════════════════════════
     * 阶段 5：主循环（空闲）
     *
     * app_main() 本身也是一个 FreeRTOS 任务（系统自动创建的"main task"）。
     * 它不能 return（return 后系统会重启）。
     *
     * 所以这里放一个无限循环，定期做一些"主任务级别"的事情。
     * 目前是空的（只延迟），后续可以加入：
     *   - 监控 free heap（剩余内存）
     *   - 监控任务栈使用情况
     *   - 看门狗喂狗
     *   - OTA 升级检查
     *
     * vTaskDelay(10000) = 让出 CPU 10 秒，什么都不做
     * ═════════════════════════════════════════════════════════════ */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
