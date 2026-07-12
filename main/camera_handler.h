/*
 * OV5640 摄像头驱动 — 头文件
 * ==========================================
 *
 * ── 这是什么？ ─────────────────────────────────────────────────────
 *
 * OV5640 是 OmniVision 生产的 500 万像素 CMOS 图像传感器。
 * 它通过 DVP（数字视频并行接口）连接到 ESP32-S3。
 * 本模块封装了 ESP-IDF 的 esp32-camera 组件，提供更简洁的 API。
 *
 * ── 给 Java 开发者的类比 ───────────────────────────────────────────
 *
 *   Java 概念                         C / 嵌入式 对应
 *   ───────────────────────────────────────────────────
 *   CameraManager.open()             camera_init()
 *   camera.takePicture(callback)     camera_capture(&frame)
 *   Bitmap.recycle()                 camera_frame_release(&frame)
 *   Camera.Parameters                camera_config_t (sensor_t)
 *   FileOutputStream (JPEG)          frame->buf (已经是 JPEG 编码)
 *
 * ── 硬件连接 ───────────────────────────────────────────────────────
 *
 *   OV5640 已经焊接在 ESP32-S3-CAM 开发板上，无需额外接线。
 *   摄像头通过 8-bit DVP 接口连接到 ESP32-S3 的特定 GPIO：
 *
 *   信号         GPIO        说明
 *   ───────────────────────────────────────────
 *   SCCB_SDA     40         SCCB 数据（类似 I²C SDA）
 *   SCCB_SCL     39         SCCB 时钟（类似 I²C SCL）
 *   VSYNC         6         垂直同步信号
 *   HREF          7         水平参考信号
 *   PCLK         15         像素时钟
 *   XCLK         16         主时钟（由 ESP32 提供，20MHz）
 *   D0 ~ D7    47,48,45,   8-bit 并行数据总线
 *              13,12,11,
 *              17,41
 *   RESET        46         摄像头复位引脚
 *   PWDN         -1         掉电引脚（不使用）
 *
 * ── 供电说明 ───────────────────────────────────────────────────────
 *
 *   OV5640 工作时电流可达 200mA+，整板峰值 500mA+。
 *   必须使用 5V/2A USB 电源，电脑 USB 口可能供电不足导致：
 *     - 拍照花屏（褐色条纹）
 *     - ESP32 反复重启
 *     - WiFi 掉线
 *
 * ── 拍照流程（高级视角）────────────────────────────────────────────
 *
 *   1. camera_init()            → 配置寄存器（分辨率、格式、时钟等）
 *   2. camera_capture(&frame)   → 拍一张照片 → JPEG 数据在 frame.buf
 *   3. 使用 frame.buf 数据      → 存 SD 卡 / 上传 MQTT / 串口输出
 *   4. camera_frame_release()   → 归还帧缓冲（非常重要！内存有限）
 *
 * ── 照片参数 ───────────────────────────────────────────────────────
 *
 *   - 分辨率：SVGA (800×600) — 鱼缸监控不需要全分辨率
 *   - 格式：JPEG（压缩比约 15~20，单帧约 10~30KB）
 *   - 缓冲：使用 PSRAM（ESP32-S3-CAM 有 8MB PSRAM）
 *   - 拍照间隔：建议 ≥ 5 秒（给传感器曝光调整时间）
 */

#pragma once  /* 防止头文件被重复 #include */

#include <stdint.h>         /* uint8_t, size_t, int64_t 等 */
#include "esp_err.h"        /* ESP-IDF 错误码类型 */

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════
 * 数据结构
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 摄像头帧 —— 一次拍照的结果。
 *
 * Java 类比：
 *   这相当于 Android 的 Image 对象 + ByteBuffer 的组合。
 *   JPEG 数据已经编码好了，不需要再转换格式。
 */
typedef struct {
    uint8_t *buf;           /* JPEG 图像数据缓冲区（指针指向 PSRAM） */
    size_t   len;           /* JPEG 数据实际长度（字节） */
    size_t   width;         /* 图像宽度（像素） */
    size_t   height;        /* 图像高度（像素） */
    int64_t  timestamp_ms;  /* 拍照时间戳（毫秒，从启动开始算） */
} camera_frame_t;

/* ═══════════════════════════════════════════════════════════════════
 * API 函数
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 初始化 OV5640 摄像头。
 *
 * 这个函数做的事情（类似 Java 构造函数的职责）：
 *   1. 配置 DVP 引脚的 GPIO 映射
 *   2. 设置 SCCB（类似 I²C）和 XCLK 时钟
 *   3. 配置传感器寄存器（分辨率、JPEG 质量、曝光等）
 *   4. 初始化帧缓冲（在 PSRAM 中分配）
 *   5. 启动传感器数据流
 *
 * @return  ESP_OK               — 摄像头初始化成功，可以拍照
 *          ESP_ERR_CAMERA_NOT_DETECTED — 没检测到摄像头（检查排线）
 *          ESP_ERR_NO_MEM       — PSRAM 初始化失败（检查 PSRAM 配置）
 *          ESP_ERR_INVALID_ARG  — 引脚配置冲突
 *
 * Java 类比：
 *   Camera camera = Camera.open();  // throws CameraException
 *
 * 注意：
 *   - 必须确保 PSRAM 已在 menuconfig 中启用
 *   - GPIO4 和 LED 闪光灯共用，拍照时不要同时操作 GPIO4
 *   - 本函数会占用约 500KB+ PSRAM 作为帧缓冲
 */
esp_err_t camera_init(void);

/**
 * 拍摄一张照片。
 *
 * 内部流程：
 *   1. 等待摄像头数据流就绪
 *   2. 从传感器获取一帧 JPEG 数据
 *   3. 填充 camera_frame_t 结构体
 *   4. 返回（数据在 frame->buf，调用者负责后续处理）
 *
 * @param frame  [出参] 拍照结果，通过指针返回
 * @return       ESP_OK               — 拍照成功
 *               ESP_ERR_INVALID_STATE — 摄像头未初始化
 *               ESP_ERR_TIMEOUT      — 等待超时（传感器卡住了？）
 *               ESP_ERR_NO_MEM       — 帧缓冲耗尽（PSRAM 不够）
 *
 * Java 类比：
 *   CameraFrame frame = camera.takePicture();  // 阻塞直到照片就绪
 *
 * 重要：
 *   - 调用 camera_frame_release() 释放帧缓冲后再拍下一张
 *   - 不要在中断服务函数（ISR）中调用此函数
 *   - 拍照间隔至少 1 秒以上
 */
esp_err_t camera_capture(camera_frame_t *frame);

/**
 * 释放摄像头帧占用的内存。
 *
 * 为什么需要手动释放？
 *   嵌入式没有 GC（垃圾回收）！每次 camera_capture() 都会从
 *   PSRAM 中分配一块缓冲。如果不及时释放，PSRAM 会用完，
 *   后续拍照会失败。
 *
 *   Java 类比：
 *     这相当于 Bitmap.recycle() —— 显式归还 native 内存。
 *     或者 try-with-resources 的 .close()。
 *
 * @param frame  之前 camera_capture() 返回的帧
 *
 * Java 类比：
 *   frame.release();  // 归还像素缓冲
 */
void camera_frame_release(camera_frame_t *frame);

#ifdef __cplusplus
}
#endif
