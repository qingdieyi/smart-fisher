/*
 * DS18B20 数字温度传感器 — 驱动头文件
 * ==========================================
 *
 * ── 这是什么？ ─────────────────────────────────────────────────────
 *
 * DS18B20 是 Maxim Integrated（现属 Analog Devices）生产的数字温度传感器。
 * 它用一根数据线就能通信（"1-Wire 总线"），不需要 I²C 或 SPI 那种多线协议。
 *
 * ── 给 Java 开发者的类比 ───────────────────────────────────────────
 *
 *    Java 概念                         C / 嵌入式 对应
 *    ───────────────────────────────────────────────────
 *    interface TemperatureSensor       头文件 (.h) — 定义 API 契约
 *    class Ds18b20Driver               源文件 (.c) — 具体实现
 *    new Ds18b20Driver(GPIO4)         ds18b20_init(GPIO_NUM_4)
 *    sensor.readTemperature()          ds18b20_read_temperature()
 *    throws IOException               返回 esp_err_t 错误码
 *
 * ── 接线要求 ───────────────────────────────────────────────────────
 *
 *   DS18B20 引脚       ESP32-S3-CAM
 *   ─────────────────────────────────
 *   VCC (红线)   →    3.3V
 *   GND (黑线)   →    GND
 *   DATA (黄线)  →    GPIO4
 *
 *   ⚠ 必须在 DATA 和 3.3V 之间焊接一个 4.7kΩ 的上拉电阻！
 *     原因：1-Wire 总线靠电阻上拉来维持高电平"空闲"状态。
 *     忘了这个电阻 → 传感器检测不到。
 *
 *   💡 推荐买"防水探头款"DS18B20（不锈钢封装 + 3米线），直接放入鱼缸。
 *
 * ── 精度 ───────────────────────────────────────────────────────────
 *
 *   我们使用 12-bit 分辨率模式：
 *     - 分辨率：0.0625°C（每 1/16 度一跳）
 *     - 量程：-55°C ~ +125°C
 *     - 每次测温耗时：750ms（芯片内部 ADC 转换时间）
 *
 * ── 安全 ───────────────────────────────────────────────────────────
 *
 *   本驱动使用纯软件 GPIO 位拆裂（bit-bang），不依赖任何第三方组件。
 *   在不支持联网下载 ESP-IDF 组件的情况下也能编译。
 */

#pragma once  /* ← 等价于 Java 的包级导入保护，防止头文件被重复 #include */

#include <stdint.h>       /* C 语言的标准整数类型：uint8_t, int16_t 等 */
#include "driver/gpio.h"  /* ESP-IDF 的 GPIO（引脚）控制 API */
#include "esp_err.h"      /* ESP-IDF 的错误码类型：esp_err_t */

#ifdef __cplusplus
extern "C" {
#endif
/*
 * extern "C" 的作用：
 *   C++ 编译器会对函数名做 "name mangling"（把函数名编码成包含参数类型的长字符串）。
 *   加上 extern "C" 告诉 C++ 编译器："这段代码按 C 语言的规则编译和链接"。
 *   这样 .c 文件（C 编译器）和 .cpp 文件（C++ 编译器）就能互相调用。
 *   对于纯 C 项目来说，这个块是空操作。
 */

/* ──────────────────────────────────────────────────────────────────
 * API 函数
 *
 * 嵌入式 C 的命名约定：模块名_动作名()
 * 类似 Java 的 TemperatureSensor.read()，这里写成 ds18b20_read_temperature()
 * ────────────────────────────────────────────────────────────────── */

/**
 * 初始化 DS18B20 传感器。
 *
 * 这个函数做的事情（类似 Java 构造函数的职责）：
 *   1. 配置 GPIO 引脚为输入模式 + 内部上拉
 *   2. 发送 1-Wire 复位脉冲，检查传感器是否在线
 *   3. 将传感器配置为 12-bit 精度模式
 *
 * @param pin  数据线连接的 GPIO 引脚号（我们用的是 GPIO_NUM_4）
 * @return     ESP_OK               — 传感器初始化成功
 *             ESP_ERR_NOT_FOUND    — 没找到传感器，检查接线和上拉电阻
 *             ESP_ERR_TIMEOUT      — 通信超时
 *
 * Java 类比：
 *   Ds18b20Driver driver = new Ds18b20Driver();
 *   driver.init(GPIO4);  // throws WiringException if not found
 */
esp_err_t ds18b20_init(gpio_num_t pin);

/**
 * 读取当前温度。
 *
 * 完整的测温流程：
 *   1. 发送 "开始转换" 命令（0x44）
 *   2. 等待 750ms（12-bit ADC 转换需要时间，类似等一个慢的 HTTP 响应）
 *   3. 发送 "读暂存器" 命令（0xBE），读取 9 字节原始数据
 *   4. CRC 校验（类似 TCP 的 checksum，确保数据没损坏）
 *   5. 计算温度值：原始值 × 0.0625 = °C
 *
 * @param pin    数据线连接的 GPIO 引脚号
 * @param temp_c [出参] 温度值（摄氏度），通过指针返回
 * @return       ESP_OK              — 读取成功
 *               ESP_ERR_TIMEOUT     — 通信超时
 *               ESP_ERR_INVALID_CRC — 数据校验失败（可能是线太长或干扰）
 *
 * Java 类比：
 *   float temperature = driver.readTemperature();  // returns °C, throws on error
 *   在 C 中，我们用返回值表达成功/失败，用指针参数表达实际数据。
 *   这类似 Java 的：
 *     Optional<Float> readTemperature()  ← 返回值同时包含成功/失败和数据
 *   或者：
 *     Result<Float, Error> readTemperature()
 */
esp_err_t ds18b20_read_temperature(gpio_num_t pin, float *temp_c);

#ifdef __cplusplus
}
#endif
