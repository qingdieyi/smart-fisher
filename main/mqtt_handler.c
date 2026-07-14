/*
 * MQTT 数据上报模块 — 实现
 * ==========================================
 *
 * ── 给你的话（Java 开发者看这里）───────────────────────────────────
 *
 * 本模块封装 ESP-IDF 的 esp_mqtt 客户端。工作流程：
 *
 *   1. 从 ESP32 MAC 地址生成唯一 device_id
 *   2. 创建 MQTT 客户端 → 连接公共 Broker
 *   3. 其他任务调用 mqtt_publish() 发送数据
 *   4. 断线自动重连（ESP-MQTT 内置机制）
 *
 * ── MQTT vs HTTP ──────────────────────────────────────────────────
 *
 *   特性              MQTT                     HTTP
 *   ──────────────────────────────────────────────────────────────
 *   架构              发布/订阅（Broker）      请求/响应（Server）
 *   连接              长连接（TCP 持久）       短连接（每次请求新连接）
 *   消息方向          双向（Pub + Sub）         单向（Client → Server）
 *   开销              极小（2 字节头）          较大（HTTP Headers）
 *   适合              物联网传感器数据          网页 API、文件下载
 *   端口              1883 (TCP) / 8883 (TLS)  80 / 443
 *
 * ── Broker 选择 ───────────────────────────────────────────────────
 *
 *   开发/测试用公共 Broker（免费，无需注册）：
 *     - broker.emqx.io:1883       (EMQX 公共测试)
 *     - test.mosquitto.org:1883   (Mosquitto 公共测试)
 *     - broker.hivemq.com:1883    (HiveMQ 公共测试)
 *
 *   生产环境建议自建或使用云服务。
 *
 * ── ESP-MQTT 内部机制 ─────────────────────────────────────────────
 *
 *   esp_mqtt 客户端内部有一个事件循环和一个发送队列。
 *   - esp_mqtt_client_enqueue() 把消息放入队列（非阻塞）
 *   - 后台任务从队列取消息，通过 TCP socket 发送
 *   - 连接断开时消息留在队列中，重连后自动续传
 *   - 这就是为什么我们的 publish 函数不等待发送完成
 *
 *   Java 类比：
 *     这就是一个内置了重连逻辑的 BlockingQueue + 消费者线程。
 */

/* ── 头文件引入 ───────────────────────────────────────────────────── */

#include "mqtt_handler.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"            /* esp_read_mac() — 读取 MAC 地址 */
#include "esp_system.h"         /* esp_get_free_heap_size() 等 */
#include "mqtt_client.h"        /* ESP-IDF MQTT 客户端 API */

/* ── 模块日志标签 ─────────────────────────────────────────────────── */
static const char *TAG = "mqtt";

/* ═══════════════════════════════════════════════════════════════════
 * 配置常量
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * MQTT Broker 地址。
 *
 * 使用 EMQX 公共测试服务器（免费，无需注册）。
 * 格式: mqtt://hostname:port 或 mqtts://hostname:port (TLS)
 *
 * 如果定义了 CONFIG_BROKER_URI（来自 sdkconfig），则使用该值。
 * 否则使用默认公共 Broker。
 */
#ifdef CONFIG_BROKER_URI
#define MQTT_BROKER_URI  CONFIG_BROKER_URI
#else
#define MQTT_BROKER_URI  "mqtt://broker.emqx.io:1883"
#endif

/*
 * Topic 前缀。
 * 所有 topic 都以此为前缀，加上 device_id 形成完整路径。
 *
 * 例：smart-fisher/A1B2C3/temperature
 */
#define MQTT_TOPIC_PREFIX  "smart-fisher"

/* ═══════════════════════════════════════════════════════════════════
 * 全局变量
 * ═══════════════════════════════════════════════════════════════════ */

static esp_mqtt_client_handle_t mqtt_client = NULL;  /* MQTT 客户端句柄 */
static char device_id[16] = {0};                      /* 设备 ID（基于 MAC） */
static char topic_prefix[64] = {0};                   /* "smart-fisher/{device_id}" */
static bool connected = false;                         /* 连接状态 */

/* ═══════════════════════════════════════════════════════════════════
 * 内部辅助函数
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 基于 ESP32 MAC 地址生成设备 ID。
 *
 * 取 MAC 地址最后 3 字节，转为大写十六进制字符串。
 * 例如 MAC = AA:BB:CC:DD:EE:FF → device_id = "DDEEFF"
 *
 * 为什么只用后 3 字节？
 *   - 前 3 字节是厂商 OUI（Espressif 的固定前缀），所有 ESP32 都相同
 *   - 后 3 字节是设备唯一编号，足够在同网段内区分设备
 */
static void generate_device_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(device_id, sizeof(device_id), "%02X%02X%02X",
             mac[3], mac[4], mac[5]);
    snprintf(topic_prefix, sizeof(topic_prefix), "%s/%s",
             MQTT_TOPIC_PREFIX, device_id);
    ESP_LOGI(TAG, "Device ID: %s (MAC: %02X:%02X:%02X:%02X:%02X:%02X)",
             device_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * 构建完整的 MQTT topic 路径。
 *
 * 把 topic_prefix + "/" + suffix 组装成完整路径。
 * 例如: "smart-fisher/A1B2C3" + "/" + "temperature"
 *     = "smart-fisher/A1B2C3/temperature"
 *
 * @param suffix   topic 后缀（如 "temperature", "image"）
 * @param buf      [出参] 输出 buffer
 * @param buf_size buffer 大小
 */
static void build_topic(const char *suffix, char *buf, size_t buf_size)
{
    snprintf(buf, buf_size, "%s/%s", topic_prefix, suffix);
}

/* ═══════════════════════════════════════════════════════════════════
 * MQTT 事件处理回调
 *
 * ESP-MQTT 通过事件驱动的机制工作。连接状态变化时，
 * 客户端内部触发事件，我们在这里处理。
 *
 * Java 类比：
 *   @EventListener
 *   public void onMqttEvent(MqttEvent event) {
 *       switch (event.getType()) {
 *           case CONNECTED: onConnected(); break;
 *           case DISCONNECTED: onDisconnected(); break;
 *       }
 *   }
 * ═══════════════════════════════════════════════════════════════════ */

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    /*
     * ESP-MQTT 的事件数据结构。
     * event_id 表示事件类型（连接/断开/数据到达等）。
     */
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        /*
         * 已连接到 Broker。
         * 此时可以开始发布消息了。
         */
        connected = true;
        ESP_LOGI(TAG, "MQTT connected to broker");
        ESP_LOGI(TAG, "Topics: %s/temperature", topic_prefix);
        ESP_LOGI(TAG, "        %s/fish_status", topic_prefix);
        ESP_LOGI(TAG, "        %s/image", topic_prefix);
        ESP_LOGI(TAG, "        %s/status", topic_prefix);
        break;

    case MQTT_EVENT_DISCONNECTED:
        /*
         * 与 Broker 断开连接。
         * ESP-MQTT 会自动重连，我们只需更新状态标记。
         * 断开期间的消息会暂存在内部队列中。
         */
        connected = false;
        ESP_LOGW(TAG, "MQTT disconnected (auto-reconnect enabled)");
        break;

    case MQTT_EVENT_ERROR:
        /*
         * MQTT 通信出错（如 DNS 解析失败、连接被拒等）。
         * 内部自动重试，我们只记日志。
         */
        ESP_LOGE(TAG, "MQTT error occurred");
        break;

    case MQTT_EVENT_PUBLISHED:
        /*
         * 一条消息被 Broker 确认（QoS 1/2 时才有）。
         * 不需要做额外处理，只打 DEBUG 日志。
         */
        ESP_LOGD(TAG, "MQTT message acknowledged (msg_id=%d)", event->msg_id);
        break;

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API 实现
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t mqtt_handler_init(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  Initializing MQTT client...");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    /* ── 步骤 1：生成设备 ID ── */
    generate_device_id();

    /* ── 步骤 2：配置 MQTT 客户端 ──
     *
     * ESP-IDF v6.0 的 MQTT 配置使用嵌套结构体。
     * broker.address.uri 是 Broker 的连接地址。
     *
     * 其他可配置项（都使用默认值）：
     *   - .session.keepalive = 120s（心跳间隔）
     *   - .network.disable_auto_reconnect = false（自动重连）
     *   - .network.reconnect_timeout_ms = 10000（重连间隔）
     */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
        },
        .credentials = {
            .client_id = device_id,     /* 用设备 MAC 后 3 字节作为 Client ID */
        },
    };

    /*
     * esp_mqtt_client_init() 创建客户端实例。
     * 注意：此时还没连接，只是分配内存和初始化内部状态。
     *
     * Java 类比：
     *   MqttClient client = new MqttClient(brokerUri, clientId);
     */
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client (out of memory?)");
        return ESP_ERR_NO_MEM;
    }

    /*
     * ── 步骤 3：注册事件回调 ──
     *
     * 类似之前的 WiFi 事件回调，告诉 ESP-MQTT：
     *   "当连接状态变化时，调用我的回调函数"。
     *
     * ESP_EVENT_ANY_ID = 监听所有 MQTT 事件类型。
     */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    /*
     * ── 步骤 4：启动 MQTT 客户端 ──
     *
     * esp_mqtt_client_start() 开始连接流程（非阻塞）：
     *   1. 创建后台任务
     *   2. DNS 解析 Broker 地址
     *   3. TCP 三次握手
     *   4. MQTT CONNECT 握手
     *
     * 整个过程在后台进行，函数立即返回。
     */
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT client started. Broker: %s", MQTT_BROKER_URI);
    ESP_LOGI(TAG, "Device ID: %s", device_id);

    return ESP_OK;
}

esp_err_t mqtt_publish(const char *topic, const char *data, int len, int qos)
{
    if (mqtt_client == NULL || topic == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len <= 0) {
        len = strlen(data);
    }

    if (len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * esp_mqtt_client_enqueue() 把消息放入内部发送队列（非阻塞）。
     *
     * 参数：
     *   - topic: MQTT 主题路径
     *   - data:  消息内容
     *   - len:   消息长度
     *   - qos:   服务质量（0/1/2）
     *   - retain: 是否保留消息（Broker 会记住最后一条 retained 消息，
     *             新订阅者一订阅就能收到。温度数据设为 0（不保留），
     *             设备状态设为 1（保留）。）
     *   - msg_id: 输出消息 ID（用于跟踪 QoS 1/2 的确认），不需要则传 NULL
     *
     * 返回值：
     *   -1 = 失败（未连接或队列满）
     *   其他 = 消息 ID（成功入队）
     */
    int msg_id = esp_mqtt_client_enqueue(mqtt_client, topic, data, len,
                                         qos, 0, NULL);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Failed to enqueue message to topic: %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Published to %s (%d bytes, msg_id=%d)", topic, len, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_publish_binary(const char *topic, const uint8_t *data,
                               size_t len, int qos)
{
    if (mqtt_client == NULL || topic == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int msg_id = esp_mqtt_client_enqueue(mqtt_client, topic,
                                         (const char *)data, len,
                                         qos, 0, NULL);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Failed to enqueue binary message to topic: %s (%u bytes)",
                 topic, (unsigned int)len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Published binary to %s (%u bytes, msg_id=%d)",
             topic, (unsigned int)len, msg_id);
    return ESP_OK;
}

bool mqtt_is_connected(void)
{
    return connected;
}

const char *mqtt_get_device_id(void)
{
    return device_id;
}
