/*
 * MQTT 数据上报模块 — 头文件
 * ==========================================
 *
 * ── 这是什么？ ─────────────────────────────────────────────────────
 *
 * 本模块封装 ESP-IDF 的 esp_mqtt 组件，提供简洁的 MQTT 发布 API。
 * 数据上报到公共 MQTT Broker（如 broker.emqx.io），任何 MQTT 客户端
 * 都可以订阅查看鱼缸数据。
 *
 * ── MQTT 是什么？ ─────────────────────────────────────────────────
 *
 * MQTT (Message Queuing Telemetry Transport) 是物联网领域最常用的
 * 轻量级消息协议。架构是"发布/订阅"模式：
 *
 *   ESP32 (Publisher)  ──发布──→  MQTT Broker  ←──订阅──  手机 App
 *                                   (服务器)
 *   发布: "smart-fisher/xxx/temperature" → 25.5°C
 *   订阅方收到: {"temperature": 25.5, ...}
 *
 * 类比 Java：
 *   这就相当于 Kafka / RabbitMQ 的 Topic，但没有那么重。
 *   MQTT 专为低带宽、不可靠网络设计，很适合嵌入式。
 *
 * ── Topic 设计 ─────────────────────────────────────────────────────
 *
 *   smart-fisher/{device_id}/temperature   — 水温数据 (JSON)
 *   smart-fisher/{device_id}/fish_status   — 鱼群状态 (JSON)
 *   smart-fisher/{device_id}/image         — 鱼缸照片 (JPEG binary)
 *   smart-fisher/{device_id}/status        — 设备状态 (JSON)
 *
 *   device_id 由 ESP32 MAC 地址后 3 字节生成，例如 "A1B2C3"。
 *
 * ── QoS 说明 ───────────────────────────────────────────────────────
 *
 *   - QoS 0: 最多发一次（不保证送达，适合温度这种定期上报的数据）
 *   - QoS 1: 至少发一次（保证送达但可能重复，适合照片）
 *   - QoS 2: 恰好发一次（最严格，开销最大）
 *
 *   温度/状态 → QoS 0（丢了等 5 秒下一包）
 *   照片     → QoS 1（确保送达）
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════
 * API 函数
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 初始化 MQTT 客户端并连接到 Broker。
 *
 * 这个函数做的事情：
 *   1. 基于 MAC 地址生成 device_id
 *   2. 创建 MQTT 客户端实例
 *   3. 注册连接/断线事件回调
 *   4. 启动连接（非阻塞，后台自动重连）
 *
 * @return  ESP_OK              — MQTT 客户端创建成功
 *          ESP_ERR_NO_MEM      — 内存不足
 *          ESP_ERR_INVALID_ARG — 配置参数无效
 *
 * Java 类比：
 *   MqttClient client = MqttClient.builder()
 *       .serverUri("mqtt://broker.emqx.io:1883")
 *       .clientId("smart-fisher-A1B2C3")
 *       .build();
 *   client.connectAsync();
 *
 * 注意：
 *   - 必须在 WiFi 连接成功后调用（需要网络）
 *   - 连接是异步的，函数返回不代表已连上
 *   - 断线后会自动重连（ESP-MQTT 内置）
 */
esp_err_t mqtt_handler_init(void);

/**
 * 发布 JSON 文本到指定 topic。
 *
 * @param topic  MQTT topic（例如 "smart-fisher/A1B2C3/temperature"）
 * @param data   JSON 字符串
 * @param len    数据长度（字节），传 0 则自动用 strlen()
 * @param qos    QoS 等级（0/1/2），温度数据推荐 0，重要数据推荐 1
 * @return       ESP_OK              — 已加入发送队列
 *               ESP_ERR_INVALID_ARG — 参数无效
 *               ESP_FAIL            — MQTT 未连接或队列满
 *
 * Java 类比：
 *   mqttClient.publish(topic, jsonPayload.getBytes(), qos, false);
 */
esp_err_t mqtt_publish(const char *topic, const char *data, int len, int qos);

/**
 * 发布二进制数据到指定 topic（用于上传 JPEG 照片）。
 *
 * @param topic  MQTT topic
 * @param data   二进制数据指针
 * @param len    数据长度（字节）
 * @param qos    QoS 等级，照片推荐 1
 * @return       ESP_OK / ESP_FAIL
 */
esp_err_t mqtt_publish_binary(const char *topic, const uint8_t *data, size_t len, int qos);

/**
 * 检查 MQTT 是否已连接到 Broker。
 *
 * @return true  — 已连接，可以发布
 *         false — 未连接（正在重连或网络不通）
 */
bool mqtt_is_connected(void);

/**
 * 获取本设备的 MQTT device ID。
 *
 * @return device_id 字符串（如 "A1B2C3"），由 MAC 地址生成。
 *         注意：返回的是静态 buffer，不要 free。
 */
const char *mqtt_get_device_id(void);

#ifdef __cplusplus
}
#endif
