/*
 * 鱼群检测模块 — 实现（双引擎：帧差法 + YOLO/ESP-DL）
 * ================================================================
 *
 * ── 给你的话（Java 开发者看这里）───────────────────────────────────
 *
 * 本模块支持两种检测引擎，通过 Kconfig 在编译时选择：
 *
 *   方式 1: 帧差法 (SMART_FISHER_DETECTION_FRAME_DIFF)
 *     - 纯 CPU 像素比较，不需要任何模型文件
 *     - 快速 (~80ms)，低内存 (~40KB)
 *     - 只能检测运动中的鱼
 *
 *   方式 2: YOLO 目标检测 (SMART_FISHER_DETECTION_YOLO)
 *     - 基于 ESP-DL 推理引擎运行 YOLOv11-nano 模型
 *     - 可以检测静止的鱼，给出精确的 bounding box
 *     - 需要预训练的 .espdl 模型文件
 *     - 推理 ~120ms，PSRAM ~700KB
 *
 * 两种方式共用同一套 API（fish_detector.h），上层 main.c 无感知。
 *
 * ── YOLO 推理管线 ─────────────────────────────────────────────────
 *
 *   JPEG (800×600)
 *      │
 *      ▼
 *   esp_jpeg 解码 → RGB565 (200×150, 1/4 缩放)
 *      │
 *      ▼
 *   双线性插值 → RGB888 (224×224)      ← 模型输入尺寸
 *      │
 *      ▼
 *   ESP-DL 推理 (espdl_model_run)       ← 神经网络前向传播
 *      │
 *      ▼
 *   NMS (非极大值抑制)                   ← 过滤重叠框
 *      │
 *      ▼
 *   质心追踪 → 活跃度评估                ← 对比上一帧
 *      │
 *      ▼
 *   fish_result_t
 *
 * ── NMS 算法 ──────────────────────────────────────────────────────
 *
 *   目标检测模型会输出很多重叠的候选框。NMS 的作用是：
 *     输入: 50 个候选框（很多重叠）
 *     输出: 3 个最佳框（每条鱼一个）
 *
 *   算法:
 *     1. 按置信度从高到低排序所有候选框
 *     2. 取置信度最高的框 A → 加入"最终结果"
 *     3. 计算 A 与其他所有框的 IoU（交并比）
 *     4. 删除与 A 的 IoU > 阈值的框（它们是"同一个鱼"的重复检测）
 *     5. 回到第 2 步，直到没有候选框
 *
 *   IoU = 交集面积 / 并集面积
 *     IoU = 1.0 → 完全重叠（肯定是同一个目标）
 *     IoU = 0.0 → 完全不重叠
 *     阈值 0.45 → IoU > 45% 就视为重复
 *
 * ── 活跃度追踪 ────────────────────────────────────────────────────
 *
 *   对比当前帧和上一帧中每条鱼的 bounding box 质心位移：
 *     - 质心几乎不动 → 鱼在休息 → 低活跃度
 *     - 质心大幅移动 → 鱼在游动 → 高活跃度
 *
 *   使用最近邻匹配（Nearest Neighbor）关联两帧中的同一目标。
 */

/* ── 头文件引入 ───────────────────────────────────────────────────── */

#include "fish_detector.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"       /* esp_jpeg — JPEG 解码 API */

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO
#include "esp_dl.h"             /* ESP-DL — 神经网络推理引擎 */
#endif

/* ── 模块日志标签 ─────────────────────────────────────────────────── */
static const char *TAG = "fish-detector";

/* ═══════════════════════════════════════════════════════════════════
 * YOLO 配置常量（通过 Kconfig 可调）
 * ═══════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO

/* 模型输入尺寸（espdet_pico 标准：224×224） */
#define YOLO_INPUT_W        224
#define YOLO_INPUT_H        224
#define YOLO_INPUT_C        3       /* RGB */

/* 置信度阈值 — 低于此值的检测结果丢弃 */
#ifndef CONFIG_SMART_FISHER_YOLO_CONFIDENCE_THRESHOLD
#define YOLO_CONF_THRESHOLD 0.50f
#else
#define YOLO_CONF_THRESHOLD CONFIG_SMART_FISHER_YOLO_CONFIDENCE_THRESHOLD
#endif

/* NMS 的 IoU 阈值 — 高于此值的重叠框视为重复 */
#ifndef CONFIG_SMART_FISHER_YOLO_IOU_THRESHOLD
#define YOLO_IOU_THRESHOLD  0.45f
#else
#define YOLO_IOU_THRESHOLD  CONFIG_SMART_FISHER_YOLO_IOU_THRESHOLD
#endif

/* 最大检测目标数 */
#define YOLO_MAX_DETECTIONS 20

/* JPEG 解码缩放比（用于检测预处理） */
#define YOLO_JPEG_SCALE     JPEG_IMAGE_SCALE_1_4   /* 800×600/4 = 200×150 */

/* JPEG 解码工作缓冲区 */
#define JPEG_WORK_BUF_SIZE  3200

#endif /* CONFIG_SMART_FISHER_DETECTION_YOLO */

/* ═══════════════════════════════════════════════════════════════════
 * 帧差法配置常量
 * ═══════════════════════════════════════════════════════════════════ */

#ifndef CONFIG_SMART_FISHER_DETECTION_YOLO

#define DETECT_SCALE        JPEG_IMAGE_SCALE_1_8   /* 800/8=100, 600/8=75 */
#define MOTION_THRESHOLD    28
#define MIN_FISH_AREA       3
#define MAX_FISH_COUNT      20
#define JPEG_WORK_BUF_SIZE  3200

#endif /* !CONFIG_SMART_FISHER_DETECTION_YOLO */

/* ═══════════════════════════════════════════════════════════════════
 * 全局状态（两个引擎共用）
 * ═══════════════════════════════════════════════════════════════════ */

static uint8_t jpeg_work_buf[JPEG_WORK_BUF_SIZE];  /* JPEG 解码 scratchpad */

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO

/*
 * ── YOLO 引擎状态 ──────────────────────────────────────────────────
 */

/* 模型句柄 */
static void *yolo_model = NULL;

/* 模型输入 buffer: 224×224×3 = 150,528 bytes（PSRAM） */
static uint8_t *yolo_input = NULL;

/* 上一帧检测到的 bbox（用于活跃度追踪） */
typedef struct {
    float cx, cy;           /* bbox 质心坐标（归一化 0~1） */
    float w, h;             /* 宽高 */
} tracked_bbox_t;

static tracked_bbox_t prev_bboxes[YOLO_MAX_DETECTIONS];
static int prev_bbox_count = 0;

/*
 * 模型二进制数据（通过 CMake target_add_binary_data 嵌入固件）。
 *
 * 符号名由 CMake 自动生成：
 *   models/espdet_pico_fish.espdl
 *   → _binary_models_espdet_pico_fish_espdl_start
 *   → _binary_models_espdet_pico_fish_espdl_end
 *
 * 如果编译报 "undefined symbol"，检查：
 *   1. CMakeLists.txt 中有 target_add_binary_data 行
 *   2. 模型文件确实在 main/models/ 目录下
 *   3. 文件名中的 / 和 . 在符号名中被转为了 _
 */
extern const uint8_t
    _binary_models_espdet_pico_fish_espdl_start[] asm(
        "_binary_models_espdet_pico_fish_espdl_start");
extern const uint8_t
    _binary_models_espdet_pico_fish_espdl_end[] asm(
        "_binary_models_espdet_pico_fish_espdl_end");

#else /* CONFIG_SMART_FISHER_DETECTION_FRAME_DIFF */

/*
 * ── 帧差法引擎状态 ────────────────────────────────────────────────
 */

static bool has_reference = false;
static uint8_t *current_gray = NULL;
static uint8_t *reference_gray = NULL;
static uint8_t *motion_map = NULL;
static uint16_t *label_map = NULL;
static int img_width = 0;
static int img_height = 0;
static int img_pixels = 0;

#endif /* CONFIG_SMART_FISHER_DETECTION_FRAME_DIFF */

/* ═══════════════════════════════════════════════════════════════════
 * 内部辅助 — RGB565 → 灰度（帧差法用）
 * ═══════════════════════════════════════════════════════════════════ */

#ifndef CONFIG_SMART_FISHER_DETECTION_YOLO

static inline uint8_t rgb565_to_gray(uint16_t pixel)
{
    uint8_t r = ((pixel >> 11) & 0x1F);
    uint8_t g = ((pixel >> 5)  & 0x3F);
    uint8_t b = (pixel         & 0x1F);
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    return (uint8_t)(((uint32_t)r * 77 + (uint32_t)g * 150 + (uint32_t)b * 29) >> 8);
}

static void convert_to_grayscale(const uint16_t *rgb565, uint8_t *gray, int count)
{
    for (int i = 0; i < count; i++) {
        gray[i] = rgb565_to_gray(rgb565[i]);
    }
}

static int compute_frame_diff(const uint8_t *current, const uint8_t *reference,
                              uint8_t *motion, int count)
{
    int motion_pixels = 0;
    for (int i = 0; i < count; i++) {
        int diff = (int)current[i] - (int)reference[i];
        if (diff < 0) diff = -diff;
        if (diff > MOTION_THRESHOLD) {
            motion[i] = 1;
            motion_pixels++;
        } else {
            motion[i] = 0;
        }
    }
    return motion_pixels;
}

/*
 * ── 连通域分析（Two-Pass + Union-Find）───────────────────────────
 */
static int find_connected_components(const uint8_t *motion, uint16_t *label,
                                     int width, int height, int max_labels)
{
    int parent[MAX_FISH_COUNT + 2];
    int area[MAX_FISH_COUNT + 2];
    int next_label = 1;
    int max_label = max_labels + 1;

    memset(label, 0, width * height * sizeof(uint16_t));
    memset(area, 0, sizeof(area));
    for (int i = 0; i <= max_labels + 1; i++) parent[i] = i;

    /* Pass 1: assign temporary labels */
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            if (motion[idx] == 0) continue;

            int left_label  = (x > 0)          ? label[y * width + (x-1)]     : 0;
            int top_label   = (y > 0)          ? label[(y-1) * width + x]     : 0;
            int topleft_lbl = (x > 0 && y > 0) ? label[(y-1) * width+(x-1)]  : 0;
            int topright_lbl= (x<width-1&&y>0) ? label[(y-1) * width+(x+1)]  : 0;

            int neighbors[4]; int nc = 0;
            if (left_label > 0)   neighbors[nc++] = left_label;
            if (top_label > 0)    neighbors[nc++] = top_label;
            if (topleft_lbl > 0)  neighbors[nc++] = topleft_lbl;
            if (topright_lbl > 0) neighbors[nc++] = topright_lbl;

            if (nc == 0) {
                if (next_label > max_label) continue;
                label[idx] = next_label;
                area[next_label] = 1;
                next_label++;
            } else {
                int min_lbl = neighbors[0];
                for (int i = 1; i < nc; i++)
                    if (neighbors[i] < min_lbl) min_lbl = neighbors[i];
                label[idx] = min_lbl;
                area[min_lbl]++;
                for (int i = 0; i < nc; i++) {
                    int lbl = neighbors[i];
                    while (parent[lbl] != lbl) lbl = parent[lbl];
                    int root = min_lbl;
                    while (parent[root] != root) root = parent[root];
                    if (lbl != root) parent[lbl] = root;
                }
            }
        }
    }

    /* Pass 2: resolve equivalences */
    for (int i = 0; i < width * height; i++) {
        if (label[i] > 0) {
            int lbl = label[i];
            while (parent[lbl] != lbl) lbl = parent[lbl];
            label[i] = lbl;
        }
    }

    /* Count valid regions */
    memset(area, 0, sizeof(area));
    for (int i = 0; i < width * height; i++)
        if (label[i] > 0 && label[i] <= max_label) area[label[i]]++;

    int fish_count = 0;
    int valid[MAX_FISH_COUNT + 2] = {0};
    for (int lbl = 1; lbl < next_label; lbl++) {
        if (parent[lbl] == lbl && area[lbl] >= MIN_FISH_AREA) {
            valid[lbl] = 1;
            fish_count++;
        }
    }
    for (int i = 0; i < width * height; i++)
        if (label[i] > 0 && label[i] <= max_label && !valid[label[i]])
            label[i] = 0;

    return fish_count;
}

#endif /* !CONFIG_SMART_FISHER_DETECTION_YOLO */

/* ═══════════════════════════════════════════════════════════════════
 * 内部辅助 — YOLO NMS + 追踪
 * ═══════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO

/**
 * 从 RGB565 像素提取单通道并归一化到 [0,1]。
 *
 * ESP-DL 模型通常期望 float32 输入，范围 [0,1] 或 [0,255]。
 * espdet_pico 使用 INT8 量化，输入为 uint8 [0,255]。
 */
static inline void rgb565_to_rgb888(uint16_t pixel,
                                    uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = ((pixel >> 11) & 0x1F);
    *g = ((pixel >> 5)  & 0x3F);
    *b = (pixel         & 0x1F);
    *r = (*r << 3) | (*r >> 2);
    *g = (*g << 2) | (*g >> 4);
    *b = (*b << 3) | (*b >> 2);
}

/**
 * 双线性插值缩放：RGB565 源 → RGB888 目标。
 *
 * 把解码后的 JPEG 缩放到模型需要的 224×224。
 * 双线性插值比最近邻更平滑，有助于模型检测小目标。
 *
 * @param src       RGB565 源数据
 * @param src_w     源宽度
 * @param src_h     源高度
 * @param dst       RGB888 目标数据（dst_w × dst_h × 3）
 * @param dst_w     目标宽度
 * @param dst_h     目标高度
 */
static void resize_rgb565_to_rgb888_bilinear(
    const uint16_t *src, int src_w, int src_h,
    uint8_t *dst, int dst_w, int dst_h)
{
    float scale_x = (float)src_w / dst_w;
    float scale_y = (float)src_h / dst_h;

    for (int dy = 0; dy < dst_h; dy++) {
        float sy = dy * scale_y;
        int sy0 = (int)sy;
        int sy1 = (sy0 + 1 < src_h) ? sy0 + 1 : sy0;
        float fy = sy - sy0;

        for (int dx = 0; dx < dst_w; dx++) {
            float sx = dx * scale_x;
            int sx0 = (int)sx;
            int sx1 = (sx0 + 1 < src_w) ? sx0 + 1 : sx0;
            float fx = sx - sx0;

            /* 四个角点 (top-left, top-right, bottom-left, bottom-right) */
            uint8_t r00, g00, b00, r01, g01, b01;
            uint8_t r10, g10, b10, r11, g11, b11;

            rgb565_to_rgb888(src[sy0 * src_w + sx0], &r00, &g00, &b00);
            rgb565_to_rgb888(src[sy0 * src_w + sx1], &r01, &g01, &b01);
            rgb565_to_rgb888(src[sy1 * src_w + sx0], &r10, &g10, &b10);
            rgb565_to_rgb888(src[sy1 * src_w + sx1], &r11, &g11, &b11);

            /* 双线性插值 */
            float w00 = (1.0f - fx) * (1.0f - fy);
            float w01 = fx * (1.0f - fy);
            float w10 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int didx = (dy * dst_w + dx) * 3;
            dst[didx + 0] = (uint8_t)(r00 * w00 + r01 * w01 + r10 * w10 + r11 * w11);
            dst[didx + 1] = (uint8_t)(g00 * w00 + g01 * w01 + g10 * w10 + g11 * w11);
            dst[didx + 2] = (uint8_t)(b00 * w00 + b01 * w01 + b10 * w10 + b11 * w11);
        }
    }
}

/**
 * NMS (Non-Maximum Suppression) — 非极大值抑制。
 *
 * 输入: 原始检测框数组
 * 输出: 去重后的检测框数组（修改 in-place）
 *
 * 算法步骤：
 *   1. 按置信度降序排序
 *   2. 遍历：每个框与后续所有框算 IoU
 *   3. IoU > 阈值 → 标记为抑制
 *   4. 压缩数组（保留未抑制的框）
 *
 * @param boxes       检测框数组 [x1,y1,x2,y2,conf]（in-place）
 * @param count       输入框数量，输出后为去重后的数量
 * @param max_count   数组最大容量
 * @param iou_thresh  IoU 阈值
 */
static void nms_suppress(float *boxes, int *count, int max_count, float iou_thresh)
{
    int n = *count;
    if (n <= 1) return;

    /* 1. 按置信度降序排序（简单冒泡，n 很小） */
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (boxes[j * 5 + 4] > boxes[i * 5 + 4]) {
                /* Swap 5 floats */
                for (int k = 0; k < 5; k++) {
                    float tmp = boxes[i * 5 + k];
                    boxes[i * 5 + k] = boxes[j * 5 + k];
                    boxes[j * 5 + k] = tmp;
                }
            }
        }
    }

    /* 2. NMS 抑制 */
    uint8_t suppressed[YOLO_MAX_DETECTIONS];
    memset(suppressed, 0, sizeof(suppressed));

    for (int i = 0; i < n; i++) {
        if (suppressed[i]) continue;

        float ax1 = boxes[i * 5 + 0];
        float ay1 = boxes[i * 5 + 1];
        float ax2 = boxes[i * 5 + 2];
        float ay2 = boxes[i * 5 + 3];
        float a_area = (ax2 - ax1) * (ay2 - ay1);
        if (a_area <= 0) continue;

        for (int j = i + 1; j < n; j++) {
            if (suppressed[j]) continue;

            float bx1 = boxes[j * 5 + 0];
            float by1 = boxes[j * 5 + 1];
            float bx2 = boxes[j * 5 + 2];
            float by2 = boxes[j * 5 + 3];
            float b_area = (bx2 - bx1) * (by2 - by1);
            if (b_area <= 0) continue;

            /* IoU = intersection / union */
            float ix1 = (ax1 > bx1) ? ax1 : bx1;
            float iy1 = (ay1 > by1) ? ay1 : by1;
            float ix2 = (ax2 < bx2) ? ax2 : bx2;
            float iy2 = (ay2 < by2) ? ay2 : by2;
            float iw = ix2 - ix1;
            float ih = iy2 - iy1;

            if (iw > 0 && ih > 0) {
                float intersection = iw * ih;
                float union_area = a_area + b_area - intersection;
                float iou = intersection / union_area;

                if (iou > iou_thresh) {
                    suppressed[j] = 1;
                }
            }
        }
    }

    /* 3. 压缩数组（移除被抑制的框） */
    int new_count = 0;
    for (int i = 0; i < n; i++) {
        if (!suppressed[i]) {
            if (new_count != i) {
                memcpy(&boxes[new_count * 5], &boxes[i * 5], 5 * sizeof(float));
            }
            new_count++;
        }
    }
    *count = new_count;
}

/**
 * 基于 bbox 质心位移计算活跃度。
 *
 * 对当前帧的每个 bbox，找上一帧最近的 bbox，
 * 计算质心欧氏距离，累加后归一化为 0~100 的分数。
 *
 * @param cur_boxes  当前帧 bbox [cx,cy,w,h]
 * @param cur_count  当前帧数量
 * @param prev_boxes 上一帧 bbox
 * @param prev_count 上一帧数量
 * @return           活跃度分数 0~100
 */
static int compute_motion_score_bbox(
    const tracked_bbox_t *cur_boxes, int cur_count,
    const tracked_bbox_t *prev_boxes, int prev_count)
{
    if (cur_count == 0 || prev_count == 0) return 0;

    float total_displacement = 0.0f;

    for (int i = 0; i < cur_count; i++) {
        float min_dist = 1000.0f;  /* 大初始值 */

        for (int j = 0; j < prev_count; j++) {
            float dx = cur_boxes[i].cx - prev_boxes[j].cx;
            float dy = cur_boxes[i].cy - prev_boxes[j].cy;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < min_dist) min_dist = dist;
        }

        total_displacement += min_dist;
    }

    /* 归一化：假设每条鱼最大位移约 15% 画面宽度 */
    float avg_displacement = total_displacement / cur_count;
    int score = (int)(avg_displacement * 100.0f / 0.15f);
    return (score > 100) ? 100 : score;
}

/**
 * 活跃度标签映射。
 */
static const char *get_activity_label_yolo(int motion_score)
{
    if (motion_score < 5)  return "calm";
    if (motion_score < 15) return "moderate";
    return "active";
}

#endif /* CONFIG_SMART_FISHER_DETECTION_YOLO */

/* ═══════════════════════════════════════════════════════════════════
 * 帧差法活跃度标签
 * ═══════════════════════════════════════════════════════════════════ */

#ifndef CONFIG_SMART_FISHER_DETECTION_YOLO

static const char *get_activity_label(int motion_score)
{
    if (motion_score < 5)  return "calm";
    if (motion_score < 15) return "moderate";
    return "active";
}

#endif

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API — fish_detector_init()
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t fish_detector_init(void)
{
#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO

    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  Fish Detector: YOLO (ESP-DL)");
    ESP_LOGI(TAG, "  Input: %d×%d RGB888", YOLO_INPUT_W, YOLO_INPUT_H);
    ESP_LOGI(TAG, "  Conf threshold: %.2f", (double)YOLO_CONF_THRESHOLD);
    ESP_LOGI(TAG, "  IoU threshold: %.2f", (double)YOLO_IOU_THRESHOLD);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    /* ── 步骤 1: 获取模型大小 ── */
    size_t model_size = (size_t)(
        _binary_models_espdet_pico_fish_espdl_end -
        _binary_models_espdet_pico_fish_espdl_start);

    if (model_size == 0 || model_size > 10 * 1024 * 1024) {
        ESP_LOGE(TAG, "Model size invalid (%u bytes). "
                 "Make sure main/models/espdet_pico_fish.espdl exists.",
                 (unsigned int)model_size);
        ESP_LOGE(TAG, "Run the training pipeline first: "
                 "see docs/yolo-practical-guide.md");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Model size: %u bytes (%.1f KB)",
             (unsigned int)model_size, (double)model_size / 1024.0);

    /* ── 步骤 2: 加载模型 ──
     *
     * espdl_model_load() 解析 .espdl 文件，初始化推理引擎。
     * 模型权重保留在 Flash 中（const data），
     * 运行时按需将部分权重加载到 PSRAM 加速。
     */
    esp_dl_model_config_t model_cfg = {
        .model_data = (void *)_binary_models_espdet_pico_fish_espdl_start,
        .model_size = model_size,
        .prefer_mem = ESP_DL_MEM_PSRAM,    /* 优先使用 PSRAM */
    };

    yolo_model = esp_dl_model_load(&model_cfg);
    if (yolo_model == NULL) {
        ESP_LOGE(TAG, "Failed to load YOLO model. "
                 "Check that the .espdl file is valid.");
        ESP_LOGE(TAG, "Model may be corrupted or incompatible "
                 "with this ESP-DL version.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "YOLO model loaded successfully");

    /* ── 步骤 3: 分配模型输入 buffer ── */
    yolo_input = (uint8_t *)heap_caps_malloc(
        YOLO_INPUT_W * YOLO_INPUT_H * YOLO_INPUT_C,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (yolo_input == NULL) {
        ESP_LOGE(TAG, "Failed to allocate model input buffer (%d bytes)",
                 YOLO_INPUT_W * YOLO_INPUT_H * YOLO_INPUT_C);
        esp_dl_model_free(yolo_model);
        yolo_model = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* ── 步骤 4: 清空追踪状态 ── */
    memset(prev_bboxes, 0, sizeof(prev_bboxes));
    prev_bbox_count = 0;

    ESP_LOGI(TAG, "YOLO detector ready. Input buffer: %d bytes (%d×%d×%d)",
             YOLO_INPUT_W * YOLO_INPUT_H * YOLO_INPUT_C,
             YOLO_INPUT_W, YOLO_INPUT_H, YOLO_INPUT_C);

    return ESP_OK;

#else /* CONFIG_SMART_FISHER_DETECTION_FRAME_DIFF */

    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  Fish Detector: Frame Differencing");
    ESP_LOGI(TAG, "  Threshold: %d, Min area: %d, Max fish: %d",
             MOTION_THRESHOLD, MIN_FISH_AREA, MAX_FISH_COUNT);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    has_reference = false;
    img_width = 0;
    img_height = 0;
    img_pixels = 0;

    return ESP_OK;

#endif
}

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API — fish_detector_analyze()  [YOLO 版本]
 * ═══════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO

esp_err_t fish_detector_analyze(const uint8_t *jpeg_data, size_t jpeg_len,
                                fish_result_t *result)
{
    if (jpeg_data == NULL || jpeg_len == 0 || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t t_total_start = esp_timer_get_time();

    memset(result, 0, sizeof(fish_result_t));
    result->activity = "unknown";

    if (yolo_model == NULL) {
        ESP_LOGW(TAG, "YOLO model not loaded, skipping detection");
        return ESP_ERR_INVALID_STATE;
    }

    /* ───────────────────────────────────────────────────────────
     * 第 1 步: JPEG 解码 → RGB565 (1/4 缩放, 200×150)
     * ─────────────────────────────────────────────────────────── */

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg_data,
        .indata_size = (uint32_t)jpeg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = YOLO_JPEG_SCALE,
        .flags = { .swap_color_bytes = 0 },
        .advanced = {
            .working_buffer = jpeg_work_buf,
            .working_buffer_size = sizeof(jpeg_work_buf),
        },
    };

    esp_jpeg_image_output_t jpeg_out;
    esp_err_t ret = esp_jpeg_get_image_info(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG info failed: 0x%X", (unsigned int)ret);
        return ESP_FAIL;
    }

    int jpeg_w = jpeg_out.width;
    int jpeg_h = jpeg_out.height;
    size_t rgb565_size = jpeg_w * jpeg_h * 2;

    uint16_t *rgb565_buf = (uint16_t *)heap_caps_malloc(
        rgb565_size, MALLOC_CAP_SPIRAM);
    if (rgb565_buf == NULL) {
        ESP_LOGE(TAG, "RGB565 buffer alloc failed (%u bytes)",
                 (unsigned int)rgb565_size);
        return ESP_ERR_NO_MEM;
    }

    jpeg_cfg.outbuf = (uint8_t *)rgb565_buf;
    jpeg_cfg.outbuf_size = (uint32_t)rgb565_size;

    ret = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed: 0x%X", (unsigned int)ret);
        free(rgb565_buf);
        return ESP_FAIL;
    }

    int64_t t_after_jpeg = esp_timer_get_time();

    /* ───────────────────────────────────────────────────────────
     * 第 2 步: 缩放到 224×224 RGB888（双线性插值）
     * ─────────────────────────────────────────────────────────── */

    resize_rgb565_to_rgb888_bilinear(
        rgb565_buf, jpeg_w, jpeg_h,
        yolo_input, YOLO_INPUT_W, YOLO_INPUT_H);

    free(rgb565_buf);
    rgb565_buf = NULL;

    int64_t t_after_resize = esp_timer_get_time();

    /* ───────────────────────────────────────────────────────────
     * 第 3 步: ESP-DL 推理
     *
     * 输入: yolo_input (224×224×3 uint8 RGB888, NHWC 格式)
     * 输出: 检测框 float 数组 [N][6] = {x1,y1,x2,y2,conf,class}
     *       或原始 YOLO 输出（需要额外解码）
     *
     * 注意: 具体的输出格式取决于模型导出时的配置。
     *       espdet_pico 默认输出已解码的检测框。
     * ─────────────────────────────────────────────────────────── */

    esp_dl_tensor_t input_tensor = {
        .data   = yolo_input,
        .shape  = {1, YOLO_INPUT_H, YOLO_INPUT_W, YOLO_INPUT_C},
        .ndim   = 4,
        .dtype  = ESP_DL_DTYPE_UINT8,
    };

    esp_dl_tensor_t *outputs = NULL;
    int num_outputs = 0;

    ret = esp_dl_model_run(yolo_model, &input_tensor, 1,
                           &outputs, &num_outputs);
    if (ret != ESP_OK || outputs == NULL || num_outputs < 1) {
        ESP_LOGE(TAG, "Model inference failed: 0x%X (num_outputs=%d)",
                 (unsigned int)ret, num_outputs);
        return ESP_FAIL;
    }

    int64_t t_after_infer = esp_timer_get_time();

    /* ───────────────────────────────────────────────────────────
     * 第 4 步: 后处理 — 提取检测框 + NMS
     *
     * 输出格式（取决于模型导出配置）:
     *
     *   格式 A — 已解码的检测框:
     *     output[0]: float数组 shape [N_det][6]
     *     每行: [x1, y1, x2, y2, confidence, class_id]
     *     坐标已归一化到 [0,1]，需要 × 图像尺寸
     *
     *   格式 B — YOLO 原始输出（需要自己解码）:
     *     output[0]: [1, 84, 8400] 或类似
     *     需要 anchor 解码 + sigmoid + NMS
     *
     * 这里按格式 A（espdet_pico 默认）处理。
     * ─────────────────────────────────────────────────────────── */

    esp_dl_tensor_t *det_output = &outputs[0];
    float *raw_data = (float *)det_output->data;

    /*
     * 推算检测框数量。
     * det_output->size = N × 6 × sizeof(float)
     * → N = size / (6 * 4)
     */
    int raw_count = det_output->size / (6 * sizeof(float));
    if (raw_count > YOLO_MAX_DETECTIONS * 2) {
        raw_count = YOLO_MAX_DETECTIONS * 2;  /* 截断 */
    }
    ESP_LOGD(TAG, "Raw detections before NMS: %d", raw_count);

    /* ── 提取超过置信度阈值的框 ── */
    float detections[YOLO_MAX_DETECTIONS * 5];
    int det_count = 0;

    for (int i = 0; i < raw_count && det_count < YOLO_MAX_DETECTIONS; i++) {
        float x1    = raw_data[i * 6 + 0];
        float y1    = raw_data[i * 6 + 1];
        float x2    = raw_data[i * 6 + 2];
        float y2    = raw_data[i * 6 + 3];
        float conf  = raw_data[i * 6 + 4];
        /* float cls = raw_data[i * 6 + 5]; — 单类检测，不需要 */

        if (conf >= YOLO_CONF_THRESHOLD) {
            int idx = det_count * 5;
            detections[idx + 0] = x1;
            detections[idx + 1] = y1;
            detections[idx + 2] = x2;
            detections[idx + 3] = y2;
            detections[idx + 4] = conf;
            det_count++;
        }
    }

    ESP_LOGD(TAG, "Detections above threshold (%.2f): %d",
             (double)YOLO_CONF_THRESHOLD, det_count);

    /* ── NMS ── */
    int nms_count = det_count;
    nms_suppress(detections, &nms_count, YOLO_MAX_DETECTIONS, YOLO_IOU_THRESHOLD);

    ESP_LOGD(TAG, "Detections after NMS: %d", nms_count);

    /* ───────────────────────────────────────────────────────────
     * 第 5 步: 活跃度追踪
     * ─────────────────────────────────────────────────────────── */

    tracked_bbox_t cur_bboxes[YOLO_MAX_DETECTIONS];
    int cur_count = (nms_count < YOLO_MAX_DETECTIONS) ? nms_count : YOLO_MAX_DETECTIONS;

    for (int i = 0; i < cur_count; i++) {
        float x1 = detections[i * 5 + 0];
        float y1 = detections[i * 5 + 1];
        float x2 = detections[i * 5 + 2];
        float y2 = detections[i * 5 + 3];
        cur_bboxes[i].cx = (x1 + x2) / 2.0f;
        cur_bboxes[i].cy = (y1 + y2) / 2.0f;
        cur_bboxes[i].w  = x2 - x1;
        cur_bboxes[i].h  = y2 - y1;
    }

    int motion_score = compute_motion_score_bbox(
        cur_bboxes, cur_count,
        prev_bboxes, prev_bbox_count);

    /* ── 保存当前帧（供下次对比）── */
    memcpy(prev_bboxes, cur_bboxes, cur_count * sizeof(tracked_bbox_t));
    prev_bbox_count = cur_count;

    /* ───────────────────────────────────────────────────────────
     * 第 6 步: 填充结果
     * ─────────────────────────────────────────────────────────── */

    result->fish_count   = cur_count;
    result->motion_score = motion_score;
    result->activity     = get_activity_label_yolo(motion_score);

    int64_t t_total_end = esp_timer_get_time();

    ESP_LOGI(TAG, "🐟 Detection: %d fish, %s (score=%d%%) "
             "[jpeg=%lldms, resize=%lldms, infer=%lldms, total=%lldms]",
             result->fish_count, result->activity, result->motion_score,
             (long long)((t_after_jpeg - t_total_start) / 1000),
             (long long)((t_after_resize - t_after_jpeg) / 1000),
             (long long)((t_after_infer - t_after_resize) / 1000),
             (long long)((t_total_end - t_total_start) / 1000));

    return ESP_OK;
}

#else /* CONFIG_SMART_FISHER_DETECTION_FRAME_DIFF */

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API — fish_detector_analyze()  [帧差法版本]
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t fish_detector_analyze(const uint8_t *jpeg_data, size_t jpeg_len,
                                fish_result_t *result)
{
    if (jpeg_data == NULL || jpeg_len == 0 || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t t_start = esp_timer_get_time();

    memset(result, 0, sizeof(fish_result_t));
    result->activity = "unknown";

    /* ── 步骤 1: JPEG 解码 → RGB565 ── */
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
        ESP_LOGE(TAG, "JPEG info failed: 0x%X", (unsigned int)ret);
        return ESP_FAIL;
    }

    int new_width = jpeg_out.width;
    int new_height = jpeg_out.height;
    int new_pixels = new_width * new_height;
    size_t outbuf_size = new_pixels * 2;

    /* 尺寸变化时重新分配 buffer */
    if (new_pixels != img_pixels) {
        if (current_gray != NULL)   { free(current_gray);   current_gray = NULL; }
        if (reference_gray != NULL) { free(reference_gray); reference_gray = NULL; }
        if (motion_map != NULL)     { free(motion_map);     motion_map = NULL; }
        if (label_map != NULL)      { free(label_map);      label_map = NULL; }

        img_width = new_width;
        img_height = new_height;
        img_pixels = new_pixels;

        current_gray   = (uint8_t *)malloc(img_pixels);
        reference_gray = (uint8_t *)malloc(img_pixels);
        motion_map     = (uint8_t *)malloc(img_pixels);
        label_map      = (uint16_t *)malloc(img_pixels * sizeof(uint16_t));

        if (current_gray == NULL || reference_gray == NULL
            || motion_map == NULL || label_map == NULL) {
            ESP_LOGE(TAG, "Buffer alloc failed (%d pixels)", img_pixels);
            fish_detector_reset();
            return ESP_ERR_NO_MEM;
        }

        memset(current_gray, 0, img_pixels);
        memset(reference_gray, 0, img_pixels);

        ESP_LOGI(TAG, "Buffers allocated: %dx%d (%d pixels)",
                 img_width, img_height, img_pixels);
    }

    /* 解码 JPEG */
    uint16_t *rgb565_buf = (uint16_t *)malloc(outbuf_size);
    if (rgb565_buf == NULL) {
        ESP_LOGE(TAG, "RGB565 buffer alloc failed (%u bytes)",
                 (unsigned int)outbuf_size);
        return ESP_ERR_NO_MEM;
    }

    jpeg_cfg.outbuf = (uint8_t *)rgb565_buf;
    jpeg_cfg.outbuf_size = (uint32_t)outbuf_size;

    ret = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed: 0x%X", (unsigned int)ret);
        free(rgb565_buf);
        return ESP_FAIL;
    }

    /* ── 步骤 2: RGB565 → 灰度 ── */
    convert_to_grayscale(rgb565_buf, current_gray, img_pixels);
    free(rgb565_buf);

    /* ── 步骤 3: 帧差 ── */
    if (!has_reference) {
        memcpy(reference_gray, current_gray, img_pixels);
        has_reference = true;
        int64_t t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "Reference frame stored (%dx%d). Took %lld ms.",
                 img_width, img_height, (long long)((t_end - t_start) / 1000));
        return ESP_OK;
    }

    int motion_pixels = compute_frame_diff(current_gray, reference_gray,
                                           motion_map, img_pixels);

    /* ── 步骤 4: 连通域分析 ── */
    int fish_count = 0;
    if (motion_pixels >= MIN_FISH_AREA) {
        fish_count = find_connected_components(motion_map, label_map,
                                               img_width, img_height,
                                               MAX_FISH_COUNT);
    }

    /* ── 步骤 5: 计算分数 ── */
    int motion_score = (motion_pixels * 100) / img_pixels;
    const char *activity = get_activity_label(motion_score);

    /* ── 更新参考帧 ── */
    memcpy(reference_gray, current_gray, img_pixels);

    /* ── 输出 ── */
    result->fish_count   = fish_count;
    result->motion_score = motion_score;
    result->activity     = activity;

    int64_t t_end = esp_timer_get_time();
    ESP_LOGI(TAG, "🐟 Detection: %d fish, %s (score=%d%%, "
             "motion_pixels=%d) [%lld ms]",
             fish_count, activity, motion_score, motion_pixels,
             (long long)((t_end - t_start) / 1000));

    return ESP_OK;
}

#endif /* CONFIG_SMART_FISHER_DETECTION_YOLO / FRAME_DIFF */

/* ═══════════════════════════════════════════════════════════════════
 * 公开 API — fish_detector_reset()
 * ═══════════════════════════════════════════════════════════════════ */

void fish_detector_reset(void)
{
    ESP_LOGI(TAG, "Resetting detector state");

#ifdef CONFIG_SMART_FISHER_DETECTION_YOLO
    memset(prev_bboxes, 0, sizeof(prev_bboxes));
    prev_bbox_count = 0;
#else
    has_reference = false;
    if (reference_gray != NULL) memset(reference_gray, 0, img_pixels);
    if (current_gray != NULL)   memset(current_gray, 0, img_pixels);
    if (motion_map != NULL)     memset(motion_map, 0, img_pixels);
    if (label_map != NULL)      memset(label_map, 0, img_pixels * sizeof(uint16_t));
#endif
}
