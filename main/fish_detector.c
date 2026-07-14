/*
 * 鱼群检测模块 — 实现
 * ==========================================
 *
 * ── 给你的话（Java 开发者看这里）───────────────────────────────────
 *
 * 本模块实现了基于帧差法的鱼群运动检测。
 * 不需要机器学习，不需要训练数据，只用简单的图像处理技术。
 *
 * ── 帧差法算法详解 ────────────────────────────────────────────────
 *
 * 第 1 步：JPEG 解码
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 使用 esp_jpeg 组件将 JPEG 解码为 RGB565 图像。              │
 *   │ 缩放比 1/8：800×600 → 100×75，速度和精度平衡。             │
 *   │                                                              │
 *   │ esp_jpeg 内部使用 Tjpgd（Tiny JPEG Decompressor），          │
 *   │ 是一个轻量级解码器，专为嵌入式设计。                          │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 第 2 步：RGB565 → 灰度
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 从 RGB565 像素提取 R、G、B 分量，计算亮度值：               │
 *   │                                                              │
 *   │   Y = 0.299 × R + 0.587 × G + 0.114 × B                    │
 *   │                                                              │
 *   │ 这是 BT.601 标准亮度公式（人眼对绿色最敏感）。               │
 *   │ 灰度值范围 0~255。                                           │
 *   │                                                              │
 *   │ RGB565 格式解释：                                            │
 *   │   Bit 15..11:  R (5-bit)                                    │
 *   │   Bit 10..5:   G (6-bit)                                    │
 *   │   Bit 4..0:    B (5-bit)                                    │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 第 3 步：帧差
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 逐个像素比较当前帧和参考帧的灰度值：                         │
 *   │                                                              │
 *   │   diff(x,y) = |current(x,y) - reference(x,y)|               │
 *   │                                                              │
 *   │ 如果 diff(x,y) > MOTION_THRESHOLD → 该像素有运动。          │
 *   │                                                              │
 *   │ 阈值选择：太低 = 噪声，太高 = 漏检慢速鱼。                   │
 *   │ 25~30 是适合鱼缸场景的经验值。                               │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 第 4 步：连通域分析（Connected Component Labeling）
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 使用 Two-Pass 算法为相邻的运动像素分配相同的 label：         │
 *   │                                                              │
 *   │ Pass 1：扫描图像，给每个运动像素一个临时 label。            │
 *   │         如果邻居也有 label，使用邻居的 label。               │
 *   │         如果左右邻居 label 不同，记录等价关系。              │
 *   │                                                              │
 *   │ Pass 2：用等价关系表统一 label。                            │
 *   │         统计每个 label 的像素数 → 过滤太小的区域。           │
 *   │         剩余 label 数 ≈ 鱼的条数。                          │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * 第 5 步：活跃度评估
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ motion_score = (运动像素数 / 总像素数) × 100                │
 *   │                                                              │
 *   │ 映射到活跃度标签：                                           │
 *   │   motion_score < 5%   → "calm"     (鱼在休息)               │
 *   │   motion_score 5~15%  → "moderate" (正常游动)               │
 *   │   motion_score > 15%  → "active"   (非常活跃，可能喂食时)   │
 *   └──────────────────────────────────────────────────────────────┘
 */

/* ── 头文件引入 ───────────────────────────────────────────────────── */

#include "fish_detector.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"       /* esp_jpeg — JPEG 解码 API */

/* ── 模块日志标签 ─────────────────────────────────────────────────── */
static const char *TAG = "fish-detector";

/* ═══════════════════════════════════════════════════════════════════
 * 检测参数（可调）
 *
 * 这些值是根据 100×75 分辨率 + 鱼缸场景选的经验值。
 * 可以通过实验调整以获得更准确的检测结果。
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * JPEG 解码缩放比例。
 * 1/8 缩放：800×600 → 100×75 = 7500 像素。
 * 对于运动检测来说完全够用，而且速度快。
 */
#define DETECT_SCALE        JPEG_IMAGE_SCALE_1_8

/*
 * 运动检测阈值（0~255）。
 * 两个像素灰度值相差超过此值才认为是"运动"。
 *
 * 太低：环境光线波动会被误判为鱼
 * 太高：缓慢游动的鱼可能漏检
 * 25~30 是适合室内鱼缸的经验范围
 */
#define MOTION_THRESHOLD    28

/*
 * 最小鱼区域（像素数）。
 * 连通域小于此值的会被当作噪声过滤掉。
 *
 * 100×75 图像中：
 *   - 3 像素 ≈ 实际图像中 24×24 像素区域（放大 8 倍后）
 *   - 对应约 3~5cm 的鱼在 800×600 图像中的大小
 *   - 太小可能是气泡、饲料残渣等
 *   - 太大可能几条鱼重叠
 */
#define MIN_FISH_AREA       3

/*
 * 最多检测的鱼数量。
 * 限制 label 数组大小，节省内存。
 */
#define MAX_FISH_COUNT      20

/*
 * JPEG 解码工作缓冲区大小（字节）。
 * Tjpgd 需要一块 scratchpad 内存。
 * 3.1KB 是默认值（JD_FASTDECODE == 0），足够 100×75 解码。
 */
#define JPEG_WORK_BUF_SIZE  3200

/* ═══════════════════════════════════════════════════════════════════
 * 全局状态
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * 检测器状态标记。
 * has_reference = false 表示还没拍过第一张照片，下次拍照会存为参考帧。
 */
static bool has_reference = false;

/*
 * 当前帧灰度图 buffer（堆分配）。
 * 大小 = width × height = 100 × 75（取决于 JPEG 实际尺寸和缩放比）
 */
static uint8_t *current_gray = NULL;
static uint8_t *reference_gray = NULL;  /* 参考帧灰度图 */
static int img_width = 0;
static int img_height = 0;
static int img_pixels = 0;              /* width × height */

/*
 * 运动标记图和 label 图。
 * 在连通域分析阶段分配和释放。
 * - motion_map: 差分结果，1=运动，0=静止
 * - label_map: 连通域标记结果，0=背景，1~N=各连通域 ID
 */
static uint8_t *motion_map = NULL;
static uint16_t *label_map = NULL;

/*
 * JPEG 解码工作缓冲区（静态分配，避免碎片）。
 */
static uint8_t jpeg_work_buf[JPEG_WORK_BUF_SIZE];

/* ═══════════════════════════════════════════════════════════════════
 * 内部辅助函数
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * 从 RGB565 像素提取亮度（灰度）值。
 *
 * RGB565 位布局：
 *   Bit:  15 14 13 12 11  10  9  8  7  6  5   4  3  2  1  0
 *         └──── R ─────┘ └────── G ──────┘ └────── B ──────┘
 *              5-bit            6-bit             5-bit
 *
 * 亮度公式（BT.601）：
 *   Y = 0.299 × R + 0.587 × G + 0.114 × B
 *
 * 用整数运算避免浮点（ESP32-S3 有 FPU 但整数更快）：
 *   R8 = R5 << 3 | R5 >> 2      (5-bit → 8-bit 扩展)
 *   G8 = G6 << 2 | G6 >> 4      (6-bit → 8-bit 扩展)
 *   B8 = B5 << 3 | B5 >> 2      (5-bit → 8-bit 扩展)
 *   Y  = (77 × R8 + 150 × G8 + 29 × B8) >> 8
 *   （77/256 ≈ 0.301, 150/256 ≈ 0.586, 29/256 ≈ 0.113）
 */
static inline uint8_t rgb565_to_gray(uint16_t pixel)
{
    /* 提取各通道并扩展到 8-bit */
    uint8_t r = ((pixel >> 11) & 0x1F);   /* 5-bit R */
    uint8_t g = ((pixel >> 5)  & 0x3F);   /* 6-bit G */
    uint8_t b = (pixel         & 0x1F);   /* 5-bit B */

    r = (r << 3) | (r >> 2);   /* 扩展到 8-bit */
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);

    /* BT.601 亮度（整数近似）: Y = 0.299R + 0.587G + 0.114B */
    return (uint8_t)(((uint32_t)r * 77 + (uint32_t)g * 150 + (uint32_t)b * 29) >> 8);
}

/**
 * 将 RGB565 图像转换为灰度图。
 *
 * @param rgb565   输入的 RGB565 像素数组
 * @param gray     输出的灰度像素数组
 * @param count    像素总数
 */
static void convert_to_grayscale(const uint16_t *rgb565, uint8_t *gray, int count)
{
    for (int i = 0; i < count; i++) {
        gray[i] = rgb565_to_gray(rgb565[i]);
    }
}

/**
 * 帧差计算：比较当前帧和参考帧的每个像素。
 *
 * @param current   当前帧灰度图
 * @param reference 参考帧灰度图
 * @param motion    输出的运动标记图（1=运动，0=静止）
 * @param count     像素总数
 * @return          运动像素数量
 */
static int compute_frame_diff(const uint8_t *current, const uint8_t *reference,
                              uint8_t *motion, int count)
{
    int motion_pixels = 0;

    for (int i = 0; i < count; i++) {
        int diff = (int)current[i] - (int)reference[i];
        if (diff < 0) diff = -diff;   /* 绝对值 */

        if (diff > MOTION_THRESHOLD) {
            motion[i] = 1;
            motion_pixels++;
        } else {
            motion[i] = 0;
        }
    }

    return motion_pixels;
}

/**
 * 连通域分析 — Two-Pass 算法。
 *
 * Pass 1：扫描每个像素
 *   - 如果 motion_map[p] == 0 → 跳过
 *   - 如果 motion_map[p] == 1 → 检查左、上、左上、右上邻居的 label
 *     - 如果所有邻居都没有 label → 分配新 label
 *     - 如果邻居有 label → 使用邻居的 label
 *     - 如果有多个不同 label → 记录等价关系（它们属于同一个区域）
 *
 * Pass 2：统一 label
 *   - 用并查集（Union-Find）合并等价 label
 *   - 重新扫描，把 label 替换为根 label
 *
 * 最后阶段：统计每个最终 label 的像素数
 *   - 像素数 < MIN_FISH_AREA → 噪声，忽略
 *   - 像素数 >= MIN_FISH_AREA → 算作一条鱼
 *
 * @param motion     运动标记图
 * @param label      输出的 label 图
 * @param width      图像宽度
 * @param height     图像高度
 * @param max_labels 最大 label 数量（label 数组大小）
 * @return           检测到的鱼数量（过滤后剩余的区域数）
 */
static int find_connected_components(const uint8_t *motion, uint16_t *label,
                                     int width, int height, int max_labels)
{
    /*
     * ── 并查集（Union-Find）数据结构 ──
     *
     * parent[i] = i 的父节点
     * 初始 parent[i] = i（每个元素是独立的集合）
     *
     * 等价于 Java 的：
     *   Map<Integer, Integer> parent = new HashMap<>();
     */
    int parent[MAX_FISH_COUNT + 2];     /* +2 留余量，索引 0 不用 */
    int area[MAX_FISH_COUNT + 2];       /* 每个 label 的像素面积 */
    int next_label = 1;                  /* 下一个可用的 label（索引 0 表示背景）*/
    int max_label = max_labels + 1;

    /* 初始化 */
    memset(label, 0, width * height * sizeof(uint16_t));
    memset(area, 0, sizeof(area));
    for (int i = 0; i <= max_labels + 1; i++) {
        parent[i] = i;
    }

    /*
     * ── Pass 1：分配临时 label ──
     */
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;

            if (motion[idx] == 0) {
                continue;   /* 非运动像素，跳过 */
            }

            /*
             * 检查四个方向的邻居（8-连通）：
             *   左上 (x-1, y-1)    上 (x, y-1)    右上 (x+1, y-1)
             *    左 (x-1, y)
             *
             * 只检查已经扫描过的邻居（上方和左侧），
             * 因为扫描顺序是从左到右、从上到下。
             */
            int left_label  = (x > 0)           ? label[y * width + (x-1)]     : 0;
            int top_label   = (y > 0)           ? label[(y-1) * width + x]     : 0;
            int topleft_label  = (x > 0 && y > 0)      ? label[(y-1) * width + (x-1)] : 0;
            int topright_label = (x < width-1 && y > 0) ? label[(y-1) * width + (x+1)] : 0;

            /*
             * 收集所有邻居的 label（去 0）。
             */
            int neighbor_labels[4];
            int nl_count = 0;
            if (left_label > 0) neighbor_labels[nl_count++] = left_label;
            if (top_label > 0)  neighbor_labels[nl_count++] = top_label;
            if (topleft_label > 0)  neighbor_labels[nl_count++] = topleft_label;
            if (topright_label > 0) neighbor_labels[nl_count++] = topright_label;

            if (nl_count == 0) {
                /*
                 * 没有邻居 → 分配新 label。
                 */
                if (next_label > max_label) {
                    continue;   /* label 不够用了，丢弃后续像素 */
                }
                label[idx] = next_label;
                area[next_label] = 1;
                next_label++;
            } else {
                /*
                 * 有邻居 → 使用最小的邻居 label。
                 * 如果有多种 label，记录等价关系。
                 */
                int min_label = neighbor_labels[0];
                for (int i = 1; i < nl_count; i++) {
                    if (neighbor_labels[i] < min_label) {
                        min_label = neighbor_labels[i];
                    }
                }
                label[idx] = min_label;
                area[min_label]++;

                /*
                 * 合并等价 label（Union）。
                 * 找每个邻居 label 的根，统一到 min_label 的根下。
                 */
                for (int i = 0; i < nl_count; i++) {
                    int lbl = neighbor_labels[i];
                    /* 找 lbl 的根 */
                    while (parent[lbl] != lbl) {
                        lbl = parent[lbl];
                    }
                    /* 找 min_label 的根 */
                    int root = min_label;
                    while (parent[root] != root) {
                        root = parent[root];
                    }
                    /* 合并 */
                    if (lbl != root) {
                        parent[lbl] = root;
                    }
                }
            }
        }
    }

    /*
     * ── Pass 2：统一 label（路径压缩）──
     */
    for (int i = 0; i < width * height; i++) {
        if (label[i] > 0) {
            int lbl = label[i];
            while (parent[lbl] != lbl) {
                lbl = parent[lbl];
            }
            label[i] = lbl;
        }
    }

    /*
     * ── 统计最终区域 ──
     *
     * 重新统计每个根 label 的像素面积（Pass 1 中的统计被合并打乱了）。
     * 只保留面积 >= MIN_FISH_AREA 的区域。
     */
    memset(area, 0, sizeof(area));

    /* 重新计算每个 label 的面积 */
    for (int i = 0; i < width * height; i++) {
        if (label[i] > 0 && label[i] <= max_label) {
            area[label[i]]++;
        }
    }

    /*
     * 统计有效区域数（过滤噪声）。
     * 每个面积 >= MIN_FISH_AREA 的区域算作一条鱼。
     */
    int fish_count = 0;
    int valid_labels[MAX_FISH_COUNT + 2] = {0};

    for (int lbl = 1; lbl < next_label; lbl++) {
        /* 只统计根节点（parent[lbl] == lbl）的区域 */
        if (parent[lbl] == lbl && area[lbl] >= MIN_FISH_AREA) {
            valid_labels[lbl] = 1;
            fish_count++;
        }
    }

    /*
     * 可选：把非鱼的区域标记回 0（背景）。
     * 这对于调试很有用。生产环境可以跳过以节省时间。
     */
    for (int i = 0; i < width * height; i++) {
        if (label[i] > 0 && label[i] <= max_label) {
            if (!valid_labels[label[i]]) {
                label[i] = 0;   /* 过滤掉噪声 */
            }
        }
    }

    return fish_count;
}

/**
 * 获取活跃度标签。
 *
 * @param motion_score  运动分数（0~100）
 * @return              活跃度字符串
 */
static const char *get_activity_label(int motion_score)
{
    if (motion_score < 5) {
        return "calm";
    } else if (motion_score < 15) {
        return "moderate";
    } else {
        return "active";
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API 实现
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t fish_detector_init(void)
{
    ESP_LOGI(TAG, "Initializing fish detector (frame-differencing method)");
    ESP_LOGI(TAG, "Parameters: threshold=%d, min_area=%d, max_fish=%d",
             MOTION_THRESHOLD, MIN_FISH_AREA, MAX_FISH_COUNT);

    has_reference = false;
    img_width = 0;
    img_height = 0;
    img_pixels = 0;

    return ESP_OK;
}

esp_err_t fish_detector_analyze(const uint8_t *jpeg_data, size_t jpeg_len,
                                fish_result_t *result)
{
    if (jpeg_data == NULL || jpeg_len == 0 || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t t_start = esp_timer_get_time();

    /* 初始化结果 */
    memset(result, 0, sizeof(fish_result_t));
    result->activity = "unknown";

    /* ───────────────────────────────────────────────────────────────
     * 第 1 步：JPEG 解码 → RGB565
     * ─────────────────────────────────────────────────────────────── */

    /*
     * 先获取图像信息（宽度、高度、输出 buffer 大小）。
     * 这一步不解码，只读 JPEG 头部。
     */
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg_data,
        .indata_size = (uint32_t)jpeg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = DETECT_SCALE,
        .flags = { .swap_color_bytes = 0 },
        .advanced = {
            .working_buffer = jpeg_work_buf,
            .working_buffer_size = sizeof(jpeg_work_buf),
        },
    };

    esp_jpeg_image_output_t jpeg_out;
    esp_err_t ret = esp_jpeg_get_image_info(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get JPEG info (err=0x%X)", (unsigned int)ret);
        return ESP_FAIL;
    }

    int new_width = jpeg_out.width;
    int new_height = jpeg_out.height;
    int new_pixels = new_width * new_height;
    size_t outbuf_size = new_pixels * 2;   /* RGB565 = 2 bytes/pixel */

    ESP_LOGD(TAG, "Decoded image: %dx%d, output buffer: %u bytes",
             new_width, new_height, (unsigned int)outbuf_size);

    /*
     * 如果图像尺寸变了（不太可能但做个检查），重新分配 buffer。
     */
    if (new_pixels != img_pixels) {
        /* 释放旧 buffer */
        if (current_gray != NULL) { free(current_gray); current_gray = NULL; }
        if (reference_gray != NULL) { free(reference_gray); reference_gray = NULL; }
        if (motion_map != NULL) { free(motion_map); motion_map = NULL; }
        if (label_map != NULL) { free(label_map); label_map = NULL; }

        img_width = new_width;
        img_height = new_height;
        img_pixels = new_pixels;

        /* 分配新 buffer */
        current_gray = (uint8_t *)malloc(img_pixels);
        reference_gray = (uint8_t *)malloc(img_pixels);
        motion_map = (uint8_t *)malloc(img_pixels);
        label_map = (uint16_t *)malloc(img_pixels * sizeof(uint16_t));

        if (current_gray == NULL || reference_gray == NULL
            || motion_map == NULL || label_map == NULL) {
            ESP_LOGE(TAG, "Failed to allocate detection buffers (%d pixels)",
                     img_pixels);
            fish_detector_reset();
            return ESP_ERR_NO_MEM;
        }

        /* 清空灰度 buffer */
        memset(current_gray, 0, img_pixels);
        memset(reference_gray, 0, img_pixels);

        ESP_LOGI(TAG, "Detection buffers allocated: %dx%d (%d pixels)",
                 img_width, img_height, img_pixels);
    }

    /*
     * 分配 RGB565 临时输出 buffer。
     * 这个比较大（100×75×2 = 15KB），用堆分配。
     */
    uint16_t *rgb565_buf = (uint16_t *)malloc(outbuf_size);
    if (rgb565_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RGB565 buffer (%u bytes)",
                 (unsigned int)outbuf_size);
        return ESP_ERR_NO_MEM;
    }

    /* 执行 JPEG 解码 */
    jpeg_cfg.outbuf = (uint8_t *)rgb565_buf;
    jpeg_cfg.outbuf_size = (uint32_t)outbuf_size;

    ret = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed (err=0x%X)", (unsigned int)ret);
        free(rgb565_buf);
        return ESP_FAIL;
    }

    /* ───────────────────────────────────────────────────────────────
     * 第 2 步：RGB565 → 灰度
     * ─────────────────────────────────────────────────────────────── */
    convert_to_grayscale(rgb565_buf, current_gray, img_pixels);

    /* RGB565 buffer 用完了，可以释放 */
    free(rgb565_buf);
    rgb565_buf = NULL;

    /* ───────────────────────────────────────────────────────────────
     * 第 3 步：帧差（如果有参考帧）
     * ─────────────────────────────────────────────────────────────── */

    if (!has_reference) {
        /*
         * 第一帧：存为参考帧，不做分析。
         * 下次拍照时才能做差分。
         */
        memcpy(reference_gray, current_gray, img_pixels);
        has_reference = true;

        int64_t t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "Reference frame stored (%dx%d). "
                 "Motion detection will start from next capture. "
                 "Took %lld ms.",
                 img_width, img_height,
                 (long long)((t_end - t_start) / 1000));
        return ESP_OK;
    }

    /*
     * 比较当前帧和参考帧。
     */
    int motion_pixels = compute_frame_diff(current_gray, reference_gray,
                                           motion_map, img_pixels);

    /* ───────────────────────────────────────────────────────────────
     * 第 4 步：连通域分析 → 估算鱼的数量
     * ─────────────────────────────────────────────────────────────── */

    int fish_count = 0;
    if (motion_pixels >= MIN_FISH_AREA) {
        fish_count = find_connected_components(motion_map, label_map,
                                               img_width, img_height,
                                               MAX_FISH_COUNT);
    }

    /* ───────────────────────────────────────────────────────────────
     * 第 5 步：计算运动分数和活跃度
     * ─────────────────────────────────────────────────────────────── */

    int motion_score = (motion_pixels * 100) / img_pixels;   /* 百分比 */
    const char *activity = get_activity_label(motion_score);

    /* ───────────────────────────────────────────────────────────────
     * 更新参考帧（当前帧变为下次的参考帧）
     * ─────────────────────────────────────────────────────────────── */
    memcpy(reference_gray, current_gray, img_pixels);

    /* ───────────────────────────────────────────────────────────────
     * 输出结果
     * ─────────────────────────────────────────────────────────────── */

    result->fish_count = fish_count;
    result->motion_score = motion_score;
    result->activity = activity;

    int64_t t_end = esp_timer_get_time();
    ESP_LOGI(TAG, "🐟 Detection: %d fish, activity=%s (score=%d%%, "
             "motion_pixels=%d) [%lld ms]",
             fish_count, activity, motion_score, motion_pixels,
             (long long)((t_end - t_start) / 1000));

    return ESP_OK;
}

void fish_detector_reset(void)
{
    ESP_LOGI(TAG, "Resetting detector state");

    has_reference = false;

    if (reference_gray != NULL) {
        memset(reference_gray, 0, img_pixels);
    }
    if (current_gray != NULL) {
        memset(current_gray, 0, img_pixels);
    }
    if (motion_map != NULL) {
        memset(motion_map, 0, img_pixels);
    }
    if (label_map != NULL) {
        memset(label_map, 0, img_pixels * sizeof(uint16_t));
    }
}
