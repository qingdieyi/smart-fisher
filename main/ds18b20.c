/*
 * DS18B20 数字温度传感器 — 驱动实现
 * ==========================================
 *
 * ── 给你的话（Java 开发者看这里）──────────────────────────────────
 *
 * 这个文件实现了用 ESP32-S3 的普通 GPIO 引脚来和 DS18B20 通信。
 * 关键技术叫 "bit-banging"（位拆裂）：不用专用硬件控制器，而是用软件
 * 手动控制引脚的高低电平，在精确的时间点上读/写数据。
 *
 * 这相当于在 Java 里不用 HttpClient，而是自己写 Socket 字节流解析。
 *
 * ── 1-Wire 协议简介 ───────────────────────────────────────────────
 *
 * 1-Wire 是 Dallas Semiconductor 发明的单线通信协议。顾名思义：
 *   只用一根数据线（+ 地线），就能实现双向通信！
 *
 * 总线上的设备有两种角色：
 *   - Master（主机）：ESP32，负责发起所有通信
 *   - Slave（从机）：DS18B20，只响应主机的命令
 *
 * 物理层靠一个上拉电阻（4.7kΩ）维持高电平。通信时，主机把线拉低来发
 * 起始信号，然后按精确的时间窗口读/写 bit。
 *
 * ── 1-Wire 与 I²C/SPI 的对比 ──────────────────────────────────────
 *
 *   特性       1-Wire             I²C              SPI
 *   ─────────────────────────────────────────────────────
 *   数据线数    1 根               2 根(SDA+SCL)    3 根+
 *   速度        慢(15kbps)         中(100-400k)     快(>10M)
 *   距离        远(可达100m+)      短(PCB级别)      短(PCB)
 *   多设备      支持(每个有唯一ID)  支持(7-bit地址)  支持(CS片选)
 *   适合场景    远程传感器         板级芯片通信      高速外设
 *
 * ── 为什么关中断？ ─────────────────────────────────────────────────
 *
 * 1-Wire 的 bit 级别时序要求精确到微秒级。如果 FreeRTOS 在通信中途
 * 切换到另一个任务，就会破坏时序，导致数据错误。
 *
 *   Java 类比：
 *     这就像你在一个线程中执行 synchronized 块，阻止其他线程打断你。
 *     portDISABLE_INTERRUPTS() ≈ ReentrantLock.lock()
 *     portENABLE_INTERRUPTS()  ≈ ReentrantLock.unlock()
 *
 *   区别：关中断比 Java 锁更"暴力"——它连操作系统内核的调度器都暂停了。
 *   所以关中断的时间越短越好（本驱动中每次只关几十微秒）。
 *
 * ── 温度读数流程 ──────────────────────────────────────────────────
 *
 *   ds18b20_read_temperature() 执行以下步骤：
 *
 *   ┌──────────────┐
 *   │ 1. 复位+检测  │  → 拉低 500us → 释放 → 等待传感器拉低（应答）
 *   │    ow_reset() │
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 2. 发 SKIP_ROM│  → 0xCC：跳过 ROM 地址匹配（总线上只有一个设备）
 *   │    + CONVERT_T│  → 0x44：启动温度转换（传感器内部 ADC 开始工作）
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 3. 等待 750ms │  → vTaskDelay() 阻塞当前任务，让出 CPU
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 4. 复位       │  → 重新开始一次通信
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 5. 发 SKIP_ROM│
 *   │  + READ_SCR  │  → 0xBE：读取 9 字节暂存器（温度数据 + CRC）
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 6. CRC 校验  │  → 对比计算出的 CRC 和读到的 CRC
 *   └──────┬───────┘
 *          ▼
 *   ┌──────────────┐
 *   │ 7. 计算温度   │  → int16_t raw = (MSB<<8)|LSB; temp = raw × 0.0625
 *   └──────────────┘
 *
 * 参考资料：
 *   - DS18B20 数据手册: https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
 *   - 1-Wire 软件实现指南: https://www.analog.com/en/resources/technical-articles/1wire-communication-through-software.html
 */

/* ── 头文件引入 ─────────────────────────────────────────────────────
 *
 * C 语言的 #include 类似于 Java 的 import，但有本质区别：
 *   Java import: 编译时引用，.class 文件中不复制代码
 *   C #include:  预处理阶段直接把头文件内容"粘贴"进来
 *                 （可以理解为编译器在编译前先做了一次文本替换）
 *
 * 所以 C 需要用 "include guard"（#pragma once）防止重复粘贴。
 */
#include "ds18b20.h"        /* 自己的头文件 —— 声明和实现要匹配 */

#include "esp_log.h"        /* ESP-IDF 日志系统 —— 类似 Java 的 SLF4J / log4j：
                             *   ESP_LOGE() = log.error()
                             *   ESP_LOGW() = log.warn()
                             *   ESP_LOGI() = log.info()
                             *   ESP_LOGD() = log.debug()     */
#include "esp_rom_sys.h"    /* ROM 中的系统函数，提供 esp_rom_delay_us()
                             * 这是微秒级延时函数，比 vTaskDelay 更精确。
                             * vTaskDelay 最小粒度是 1 个 FreeRTOS tick（通常 10ms），
                             * 而 esp_rom_delay_us 可以精确到 1 微秒。              */
#include "freertos/FreeRTOS.h"  /* FreeRTOS 核心 —— 实时操作系统的"JVM" */
#include "freertos/portmacro.h" /* FreeRTOS 移植层 —— portDISABLE_INTERRUPTS() 等 */
#include "freertos/task.h"      /* FreeRTOS 任务管理 —— vTaskDelay() 等 */

/* ── 模块日志标签 ───────────────────────────────────────────────────
 *
 * 相当于 Java 的：
 *   private static final Logger LOGGER = LoggerFactory.getLogger("ds18b20");
 *
 * 串口输出时会显示：I (12345) ds18b20: Initializing DS18B20 on GPIO 4
 *                       ^^^^^  ^^^^^^^^^
 *                       时间戳  这个 TAG
 */
static const char *TAG = "ds18b20";

/* ═══════════════════════════════════════════════════════════════════
 * 1-Wire 协议命令字
 *
 * 这些是 DS18B20 芯片手册里定义的"指令码"。
 * 主机通过发送特定的字节来告诉传感器要做什么。
 * 类似于 HTTP 的 GET/POST，或者 SQL 的 SELECT/INSERT。
 * ═══════════════════════════════════════════════════════════════════ */

/* ── ROM 命令（用于识别/寻址总线上的设备） ──────────────────────── */

/* 1-Wire 总线上可以挂多个设备，每个设备有唯一的 64-bit ROM ID。
 * 但我们的鱼缸只有一个传感器，所以可以用 SKIP_ROM 跳过地址匹配。
 * 这相当于广播："总线上的所有人听着！"（反正只有一个人）            */

#define DS18B20_CMD_SEARCH_ROM    0xF0  /* 搜索总线上的所有设备 ID */
#define DS18B20_CMD_READ_ROM      0x33  /* 读取单个设备的 64-bit ROM ID */
#define DS18B20_CMD_MATCH_ROM     0x55  /* 指定要和哪个设备通信（通过 64-bit ID）*/
#define DS18B20_CMD_SKIP_ROM      0xCC  /* 跳过地址匹配，直接对"所有"设备说话 */
#define DS18B20_CMD_ALARM_SEARCH  0xEC  /* 搜索处于报警状态的设备 */

/* ── 功能命令（告诉传感器做什么） ───────────────────────────────── */

#define DS18B20_CMD_CONVERT_T         0x44  /* 开始温度转换（≈ 执行 SQL SELECT） */
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E  /* 写暂存器（配置分辨率等） */
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE  /* 读暂存器（获取温度原始值） */
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48  /* 将暂存器复制到 EEPROM（掉电保存）*/
#define DS18B20_CMD_RECALL_E2         0xB8  /* 从 EEPROM 恢复出厂设置 */
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4  /* 查询供电模式（寄生供电 vs 外部供电）*/

/* ═══════════════════════════════════════════════════════════════════
 * 1-Wire 时序参数（单位：微秒 μs）
 *
 * 1-Wire 协议的底层是靠精确的时间窗口来区分 0 和 1 的。
 * 每个 bit 的传输占用一个 "时隙"（time slot），时长 60~120μs。
 *
 * 主机（ESP32）控制所有时序：
 *   - 要发 0：把线拉低并保持 60~120μs
 *   - 要发 1：把线拉低只保持 1~15μs，然后释放让电阻拉高
 *   - 要读 bit：把线拉低 1~15μs，释放，在 15μs 内采样
 *
 * 类比 Java：这就像用 Thread.sleep() 做精确的纳秒级延时来控制
 *            Socket 的 TX/RX 线 —— 通常我们会用硬件 UART 来做这事，
 *            但 DS18B20 用了一个非标准协议，所以只能软件模拟。
 * ═══════════════════════════════════════════════════════════════════ */

#define OW_RESET_LOW_US   500  /* 复位：主机拉低 500μs（手册要求 ≥480μs）*/
#define OW_RESET_WAIT_US  70   /* 复位：释放后等待 70μs 再采样应答脉冲 */
#define OW_SLOT_START_US  2    /* 每个时隙开始时拉低 2μs（手册要求 1~15μs）*/
#define OW_WRITE0_LOW_US  62   /* 写 0：拉低 62μs（手册要求 60~120μs）*/
#define OW_WRITE1_LOW_US  5    /* 写 1：只拉低 5μs（手册要求 1~15μs）*/
#define OW_RECOVERY_US    3    /* 时隙之间的恢复时间 */
#define OW_SAMPLE_US      10   /* 读时隙：拉低 2μs 后，再等 10μs 采样 */
#define OW_CONVERT_MS     750  /* 12-bit 温度转换时间（毫秒）*/

/* ═══════════════════════════════════════════════════════════════════
 * GPIO 辅助函数
 *
 * 这些是封装了 ESP-IDF GPIO API 的内联函数。
 *
 * Java 类比：
 *   这些就是 private helper method，比如：
 *     private void setPinHigh(int pin) { digitalWrite(pin, HIGH); }
 *
 * 写成 static inline 的原因：
 *   - static：只在当前 .c 文件可见（≈ Java 的 private）
 *   - inline：建议编译器把函数体直接嵌入调用处，省去函数调用的开销
 *     （在亚微秒级时序控制中，函数调用的几条指令开销也很大）
 * ═══════════════════════════════════════════════════════════════════ */

static inline void ow_set_pin_output(gpio_num_t pin)
{
    /*
     * 把引脚设为输出模式。
     * 输出模式 = ESP32 主动控制引脚电平（高或低）。
     * 类似 Java 的 socket.getOutputStream()。
     */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static inline void ow_set_pin_input(gpio_num_t pin)
{
    /*
     * 把引脚设为输入模式（高阻态）。
     * 输入模式 = ESP32"放手"，让外部电路（上拉电阻或传感器）决定电平。
     * 类似 Java 的 socket.getInputStream()。
     *
     * 为什么要切输入模式？
     *   → 省电、不干扰总线、让上拉电阻把线拉高（1-Wire 的"空闲"状态）
     *
     * 为什么要切来切去？
     *   → 1-Wire 是半双工协议（同一根线既读又写），
     *     需要不断在"说话"（输出）和"听"（输入）之间切换。
     *     类似对讲机：按着按钮说话（输出），松开按钮听（输入）。
     */
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

static inline void ow_set_pin_low(gpio_num_t pin)
{
    /* 把引脚拉低（0V）*/
    gpio_set_level(pin, 0);
}

static inline void ow_set_pin_high(gpio_num_t pin)
{
    /* 把引脚拉高（3.3V）*/
    gpio_set_level(pin, 1);
}

static inline int ow_get_pin_level(gpio_num_t pin)
{
    /* 读取引脚当前电平（返回 0 或 1）*/
    return gpio_get_level(pin);
}

/* ═══════════════════════════════════════════════════════════════════
 * 1-Wire 底层通信函数
 *
 * 以下函数实现了 1-Wire 协议的三个基本操作：
 *   ow_reset()      — 复位 + 检测设备是否存在
 *   ow_write_bit()  — 发送 1 个 bit
 *   ow_read_bit()   — 接收 1 个 bit
 *
 * 然后在它们基础上构建字节级操作：
 *   ow_write_byte() — 发送 1 个字节（循环 8 次 ow_write_bit）
 *   ow_read_byte()  — 接收 1 个字节（循环 8 次 ow_read_bit）
 *
 * 类似网络协议栈的：物理层(bit) → 数据链路层(byte) → 应用层(温度值)
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 1-Wire 复位 + 设备检测。
 *
 * 这是每次通信的"敲门"动作。流程：
 *   1. 主机把线拉低 500μs  →  "喂，有人在吗？"
 *   2. 主机释放线，切换到输入模式
 *   3. 等待 70μs
 *   4. 采样：如果传感器在线，它会把线拉低（presence pulse，应答脉冲）
 *      持续 60~240μs
 *   5. 如果采样到低电平 → 有设备；高电平 → 没设备
 *
 *   ⏱ 时间线（单位：μs）：
 *   ──┐     ┌──────────────────────────
 *     └─────┘  ← 500μs 低电平
 *           ↑                     ↑
 *           主机拉低               采样点（70μs 后）
 *           (如果传感器在线，这里线还是低的)
 *
 * @return ESP_OK (0)        = 检测到设备
 *         ESP_ERR_NOT_FOUND = 没检测到设备
 */
static esp_err_t ow_reset(gpio_num_t pin)
{
    int presence;

    /*
     * ⚠ 关中断！
     * 复位时序要求主机精确控制 500μs 的低电平。
     * 如果中途被 FreeRTOS 任务切换打断（比如 WiFi 中断来了），
     * 实际低电平时间可能变成 500μs + 1ms，传感器会认为超时。
     * 关中断能保证我们在这几十微秒内独占 CPU。
     */
    portDISABLE_INTERRUPTS();

    /* ── 阶段 1：发出复位脉冲（拉低 500μs） ── */
    ow_set_pin_low(pin);          /* 先设电平 */
    ow_set_pin_output(pin);       /* 再切方向 → 电平被驱动到引脚上 */
    esp_rom_delay_us(OW_RESET_LOW_US);  /* 保持 500μs（精确硬件延时） */

    /* ── 阶段 2：释放总线，等待传感器应答 ── */
    ow_set_pin_high(pin);         /* 先设高电平（其实输入模式不需要，但好习惯）*/
    ow_set_pin_input(pin);        /* 切输入 → 上拉电阻把线拉到 3.3V */
    esp_rom_delay_us(OW_RESET_WAIT_US);  /* 等待 70μs 让传感器反应 */

    /* ── 阶段 3：采样应答脉冲 ── */
    presence = ow_get_pin_level(pin);    /* 读电平：0=传感器在线，1=离线 */

    portENABLE_INTERRUPTS();      /* 可以恢复中断了 */

    /* 等待应答窗口结束（总共约 480μs 后才算完整的一次复位） */
    esp_rom_delay_us(OW_RESET_LOW_US - OW_RESET_WAIT_US);

    if (presence == 0) {
        return ESP_OK;            /* 传感器拉低了线 → 在线！ */
    }
    return ESP_ERR_NOT_FOUND;     /* 线还是高的 → 没人应答 */
}

/**
 * 发送 1 个 bit 到 1-Wire 总线。
 *
 * 1-Wire 用不同长度的低电平来区分 0 和 1：
 *
 *   写 0：┌──────────┐                   写 1：┌─┐
 *         │ 60~120μs │                         │1~15μs│
 *         │   低电平  │                         │低电平 │
 *   ──────┘          └────────────      ────────┘      └────────────
 *
 *   理解：低电平持续时间长 = 0，低电平持续时间短 = 1。
 *   有点像莫尔斯电码：长按 = 划(dash)，短按 = 点(dot)。
 *
 * @param bit 要发送的 bit 值（0 或 1）
 */
static void ow_write_bit(gpio_num_t pin, uint8_t bit)
{
    portDISABLE_INTERRUPTS();

    /* ── 每个时隙都以拉低开始 ── */
    ow_set_pin_low(pin);
    ow_set_pin_output(pin);
    esp_rom_delay_us(OW_SLOT_START_US);  /* 起始低脉冲（2μs，手册要求 ≥1μs）*/

    if (bit) {
        /* ── 写 1：短低脉冲后立即释放 ──
         * 总低电平时间 = OW_SLOT_START_US = 2μs（远小于 15μs 上限）
         * 释放后上拉电阻把线拉高，传感器看到"短脉冲"= bit 1             */
        ow_set_pin_high(pin);
        ow_set_pin_input(pin);
        esp_rom_delay_us(OW_WRITE0_LOW_US);  /* 等待剩余时隙走完 */
    } else {
        /* ── 写 0：长低脉冲 ──
         * 总低电平时间 = OW_SLOT_START_US + OW_WRITE0_LOW_US ≈ 64μs
         * 传感器看到"长脉冲"= bit 0                                   */
        esp_rom_delay_us(OW_WRITE0_LOW_US);
        ow_set_pin_high(pin);
        ow_set_pin_input(pin);
        esp_rom_delay_us(OW_SLOT_START_US);  /* 小延时让总线稳定 */
    }

    portENABLE_INTERRUPTS();
    esp_rom_delay_us(OW_RECOVERY_US);  /* 时隙间隔恢复时间 */
}

/**
 * 从 1-Wire 总线读取 1 个 bit。
 *
 * 读取时隙中，主机只发一个短低脉冲表示"我要读了"，然后立即释放线，
 * 让传感器来控制电平：
 *
 *   读时隙：  ┌─┐    ← 主机拉低 2μs
 *            │ │
 *   ─────────┘ └────────────────────
 *              ↑                    ↑
 *              │                    │
 *         传感器保持低 = bit 0    传感器释放 → 电阻拉高 = bit 1
 *         （传感器在"说" 0）     （传感器在"说" 1）
 *
 *   主机在 OW_SAMPLE_US（10μs）处采样。
 *
 * @return 读取到的 bit 值（0 或 1）
 */
static uint8_t ow_read_bit(gpio_num_t pin)
{
    uint8_t bit;

    portDISABLE_INTERRUPTS();

    /* ── 起始低脉冲（告诉传感器"我要读了"） ── */
    ow_set_pin_low(pin);
    ow_set_pin_output(pin);
    esp_rom_delay_us(OW_SLOT_START_US);  /* 2μs */

    /* ── 释放线，让传感器来驱动 ── */
    ow_set_pin_high(pin);
    ow_set_pin_input(pin);
    esp_rom_delay_us(OW_SAMPLE_US);      /* 等 10μs 让线路电平稳定 */

    /* ── 采样！ ── */
    bit = ow_get_pin_level(pin);         /* 读传感器的输出 */

    portENABLE_INTERRUPTS();

    /* 等待剩余时隙走完（总共 60~120μs）*/
    esp_rom_delay_us(OW_WRITE0_LOW_US - OW_SAMPLE_US);
    esp_rom_delay_us(OW_RECOVERY_US);

    return bit;
}

/**
 * 发送 1 个字节（8 bit），低位先发（LSB first）。
 *
 * 1-Wire 协议规定数据按 LSB（Least Significant Bit，最低有效位）优先发送。
 * 比如要发 0x41 (0100 0001)：
 *   bit 0 先发 → 1
 *   bit 1     → 0
 *   bit 2     → 0
 *   ...
 *   bit 7 最后 → 0
 *
 * Java 类比：
 *   OutputStream.write(byte) — 但这里一次只发一个 bit
 */
static void ow_write_byte(gpio_num_t pin, uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit(pin, byte & 0x01);  /* 取最低位发送 */
        byte >>= 1;                       /* 右移一位，准备发下一个 bit */
    }
}

/**
 * 接收 1 个字节（8 bit），低位先收（LSB first）。
 *
 * 收到的第一个 bit 放入 byte 的最低位，然后不断左移。
 *
 * Java 类比：
 *   InputStream.read() — 返回一个 byte（0-255）
 */
static uint8_t ow_read_byte(gpio_num_t pin)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;                      /* 先右移，给新 bit 腾位置 */
        if (ow_read_bit(pin)) {          /* 如果读到 1 */
            byte |= 0x80;                /* 把最高位（bit7）设为 1 */
        }
    }
    return byte;
}

/* ═══════════════════════════════════════════════════════════════════
 * Dallas CRC-8 校验
 *
 * DS18B20 用 CRC-8 来验证数据传输的正确性。
 *
 * CRC 是什么？
 *   循环冗余校验（Cyclic Redundancy Check）。简单理解：
 *     把一串数据当成一个大整数，除以一个固定的"生成多项式"，
 *     余数就是 CRC 值。
 *   发送方附上 CRC → 接收方重新计算 CRC → 对比是否一致。
 *
 * Java 类比：
 *   这就像计算一个 hash code 来校验数据完整性。
 *   类似 Java 的 MessageDigest，但更轻量（只有一个字节）。
 *
 * 多项式：x^8 + x^5 + x^4 + 1 (Dallas 1-Wire 标准)
 *   二进制表示：100110001
 *   反射后：    10001100 = 0x8C
 *
 * 注意：DS18B20 的 CRC 校验范围是暂存器的前 8 个字节（byte 0~7），
 *       不包含 byte 8 本身（byte 8 就是 CRC 值）。
 * ═══════════════════════════════════════════════════════════════════ */

static uint8_t ow_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;

    while (len--) {
        uint8_t byte = *data++;          /* 取下一个字节 */

        for (int i = 0; i < 8; i++) {
            /*
             * 如果 crc 的最低位和 byte 的最低位不同 → mix=1
             * （相当于 CRC 多项式除法中当前位需要 XOR 操作）
             */
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;                    /* CRC 寄存器右移 */
            if (mix) {
                crc ^= 0x8C;              /* XOR 反射多项式 */
            }
            byte >>= 1;                   /* 数据字节也右移 */
        }
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API 实现
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 初始化 DS18B20。
 *
 * 完整流程：
 *   1. 配置 GPIO
 *   2. 检测传感器存在
 *   3. 配置 12-bit 分辨率
 *
 * 关于 GPIO 配置的说明：
 *   - GPIO_MODE_INPUT: 默认输入模式，不主动驱动电平
 *   - GPIO_PULLUP_ENABLE: 启用 ESP32 内部上拉电阻（~45kΩ）
 *     注意：内部上拉电阻值较大（45kΩ），只作为辅助。
 *     外部仍需焊接 4.7kΩ 电阻才能稳定通信！
 *   - GPIO_PULLDOWN_DISABLE: 不使用下拉电阻
 */
esp_err_t ds18b20_init(gpio_num_t pin)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing DS18B20 on GPIO %d", (int)pin);

    /*
     * 配置 GPIO 引脚。
     *
     * .pin_bit_mask = (1ULL << pin)
     *   这是一个位掩码（bitmask）。例如 GPIO4：
     *     1ULL << 4 = 0b0001 0000 = 16
     *   ESP-IDF 用位掩码来选择要配置哪些引脚。
     *   类似 Java EnumSet<GpioPin>。
     *
     * ULL 后缀 = unsigned long long（64-bit 无符号整数）
     * 因为 ESP32-S3 有 49 个 GPIO，需要 64-bit 才能表示。
     */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     /* 启用内部上拉（~45kΩ）*/
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* 不用下拉 */
        .intr_type = GPIO_INTR_DISABLE,        /* 不用中断 */
    };

    /*
     * ESP_ERROR_CHECK() 宏：
     *   如果 gpio_config() 返回非 ESP_OK，立即 abort。
     *   类似 Java 的：
     *     Objects.requireNonNull(value, "GPIO config failed");
     *   或者更准确地说，类似 assert 但在 Release 模式下也生效。
     */
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* 确保总线空闲（上拉电阻会把线拉到高电平）*/
    ow_set_pin_high(pin);

    /* ── 检测设备是否存在 ── */
    ret = ow_reset(pin);
    if (ret != ESP_OK) {
        /*
         * 没找到传感器！可能原因：
         *   1. 忘了焊 4.7kΩ 上拉电阻
         *   2. 接线顺序搞反了
         *   3. 杜邦线接触不良
         *   4. 传感器坏了
         *
         * 注意：这里用 ESP_LOGE（Error 级别）而非 assert 崩溃。
         * 设备可能在后续插上传感器后恢复正常（热插拔）。
         */
        ESP_LOGE(TAG, "No DS18B20 device found on GPIO %d. "
                 "Check wiring: DATA → GPIO%d, VCC → 3.3V, GND → GND, "
                 "with 4.7kΩ pull-up between DATA and 3.3V.",
                 (int)pin, (int)pin);
        return ret;
    }

    /* ── 配置传感器 ── */
    /*
     * 发送三个命令：
     *   1. SKIP_ROM (0xCC) — "嘿，总线上的设备听我说"（不对特定设备寻址）
     *   2. WRITE_SCRATCHPAD (0x4E) — "我要写配置了"
     *   3. 三个字节的配置数据：
     *      - TH = 0x00（高温报警阈值，我们不关心）
     *      - TL = 0x00（低温报警阈值，我们不关心）
     *      - Config = 0x7F（分辨率设为 12-bit）
     *
     * Config 寄存器各位的含义：
     *   Bit 7: 0
     *   Bit 6: 1  ─┐
     *   Bit 5: 1   ├─ R1, R0 = 11 → 12-bit 分辨率
     *   Bit 4~0: 11111（保留，固定为 1）
     *
     *   0x7F = 0111 1111
     *
     * 不同分辨率对比：
     *   R1R0  分辨率    转换时间    精度
     *   00    9-bit     93.75ms    0.5°C
     *   01    10-bit    187.5ms    0.25°C
     *   10    11-bit    375ms      0.125°C
     *   11    12-bit    750ms      0.0625°C  ← 我们选的
     */
    ow_write_byte(pin, DS18B20_CMD_SKIP_ROM);
    ow_write_byte(pin, DS18B20_CMD_WRITE_SCRATCHPAD);
    ow_write_byte(pin, 0x00);   /* TH register (alarm high trigger — unused) */
    ow_write_byte(pin, 0x00);   /* TL register (alarm low trigger — unused) */
    ow_write_byte(pin, 0x7F);   /* Configuration register: 12-bit resolution */

    ESP_LOGI(TAG, "DS18B20 initialized successfully (12-bit, ±0.0625°C)");
    return ESP_OK;
}

/**
 * 读取温度（°C）。
 *
 * ── 暂存器（Scratchpad）布局 ──────────────────────────────────────
 *
 * DS18B20 内部有一个 9 字节的 SRAM 叫"暂存器"，
 * 温度数据就存在这里：
 *
 *   Byte 0: Temperature LSB (低 8 位)
 *   Byte 1: Temperature MSB (高 8 位，高 4 位是符号扩展)
 *   Byte 2: TH register (报警上限)
 *   Byte 3: TL register (报警下限)
 *   Byte 4: Configuration register (分辨率配置)
 *   Byte 5: Reserved (0xFF)
 *   Byte 6: Reserved
 *   Byte 7: Reserved (0x10)
 *   Byte 8: CRC (前 8 字节的 CRC-8 校验值)
 *
 * ── 温度数据格式（12-bit 模式） ──────────────────────────────────
 *
 *   Bit 15 14 13 12 11 10 9 8  7 6 5 4 3 2 1 0
 *        S  S  S  S  S  2⁶ 2⁵ 2⁴ 2³ 2² 2¹ 2⁰ 2⁻¹ 2⁻² 2⁻³ 2⁻⁴
 *        └─符号位(拷贝)─┘  └──── 整数部分 ────┘ └─── 小数部分 ───┘
 *
 *   S = 符号位（0=正温度，1=负温度），高 4 位全是 S。
 *   温度值 = 原始值 × 0.0625°C
 *
 *   例 1：+25.0625°C → raw = 0x0191 = 401 → 401 × 0.0625 = 25.0625
 *   例 2：-0.5°C     → raw = 0xFFF8 = -8  → -8 × 0.0625 = -0.5
 *   例 3：+85°C（上限）→ raw = 0x0550 = 1360 → 1360 × 0.0625 = 85.0
 */
esp_err_t ds18b20_read_temperature(gpio_num_t pin, float *temp_c)
{
    /* 参数校验 —— 等价于 Java 的 Objects.requireNonNull(temp_c) */
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ═══════════════════════════════════════════════════════════════
     * 第 1 步：启动温度转换（CONVERT_T）
     *
     * 发送 0x44 命令后，DS18B20 内部的 ADC 开始工作。
     * 转换期间传感器不响应任何通信。
     * 我们需要等 750ms（12-bit 模式）。
     *
     * Java 类比：
     *   Future<Float> future = sensor.startConversion();
     *   // ... wait 750ms ...
     *   float temp = future.get();
     * ═══════════════════════════════════════════════════════════════ */

    if (ow_reset(pin) != ESP_OK) {
        ESP_LOGW(TAG, "Reset failed before CONVERT_T");
        return ESP_ERR_TIMEOUT;
    }
    ow_write_byte(pin, DS18B20_CMD_SKIP_ROM);
    ow_write_byte(pin, DS18B20_CMD_CONVERT_T);

    /*
     * vTaskDelay() = FreeRTOS 的任务延时
     *
     * 与 Thread.sleep() 的重要区别：
     *   Thread.sleep(750)  → 阻塞当前线程 750ms，JVM 可能用 spin-wait
     *   vTaskDelay(750ms)   → 当前 FreeRTOS 任务进入 Blocked 状态，
     *                         调度器切换到其他就绪任务（比如 WiFi 任务）。
     *                         CPU 不会空转浪费电！
     *
     * pdMS_TO_TICKS() 把毫秒转换成 FreeRTOS 的 tick 数。
     * 默认 tick rate = 100Hz → 1 tick = 10ms → 750ms = 75 ticks
     */
    vTaskDelay(pdMS_TO_TICKS(OW_CONVERT_MS));

    /* ═══════════════════════════════════════════════════════════════
     * 第 2 步：读取暂存器（READ_SCRATCHPAD）
     *
     * 重新复位 → 发 SKIP_ROM → 发 READ_SCRATCHPAD → 读 9 个字节
     * ═══════════════════════════════════════════════════════════════ */

    if (ow_reset(pin) != ESP_OK) {
        ESP_LOGW(TAG, "Reset failed before READ_SCRATCHPAD");
        return ESP_ERR_TIMEOUT;
    }
    ow_write_byte(pin, DS18B20_CMD_SKIP_ROM);
    ow_write_byte(pin, DS18B20_CMD_READ_SCRATCHPAD);

    /* 读取 9 字节暂存器 */
    uint8_t sp[9];
    for (int i = 0; i < 9; i++) {
        sp[i] = ow_read_byte(pin);
    }

    /* ═══════════════════════════════════════════════════════════════
     * 第 3 步：CRC 校验
     *
     * 对前 8 字节计算 CRC，与第 9 字节（sp[8]）对比。
     * 不匹配 = 数据传输中发生了 bit 翻转（电磁干扰、线太长等）。
     *
     * 为什么不用重试？简单的重试策略可以加在调用方（main.c 的循环里）。
     * ═══════════════════════════════════════════════════════════════ */

    if (ow_crc8(sp, 8) != sp[8]) {
        ESP_LOGW(TAG, "Scratchpad CRC mismatch: calc=0x%02X, recv=0x%02X",
                 ow_crc8(sp, 8), sp[8]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ═══════════════════════════════════════════════════════════════
     * 第 4 步：计算温度
     *
     *   raw = (sp[1] << 8) | sp[0]
     *       = (高字节 << 8) + 低字节
     *       = int16_t 有符号整数
     *
     *   temp = raw × 0.0625
     *
     * 为什么用 int16_t 而不是 uint16_t？
     *   → 低温时（< 0°C），DS18B20 返回补码表示的负值。
     *     int16_t 可以正确表示 -55°C ~ +125°C 的范围。
     * ═══════════════════════════════════════════════════════════════ */

    int16_t raw = (sp[1] << 8) | sp[0];

    /*
     * 0.0625f = 1/16
     * 在 C 中写 0.0625f（float 字面量，后缀 f）而不是 0.0625（double）。
     * ESP32-S3 有硬件单精度 FPU，float 运算很快。
     */
    *temp_c = raw * 0.0625f;

    /*
     * ESP_LOGD = DEBUG 级别日志。
     * 默认不显示，需要 idf.py menuconfig → Log output → 把日志级别调成 Debug。
     * 类似 Java 的 log.debug()，默认不输出。
     */
    ESP_LOGD(TAG, "Raw=0x%04X, Temp=%.2f°C", (uint16_t)raw, (double)*temp_c);

    return ESP_OK;
}
