// ============================================================
// postprocess.h —— YOLOv8 官方优化版模型后处理
// 对齐 rknn_model_zoo/examples/yolov8/cpp/postprocess.cc 的 process_fp32
// 官方模型 9 个输出（3 尺度 × 3 组，NCHW）：
//   box:  (1, 64, g, g)  ← 4 坐标 × 16 DFL bins（未解码）
//   cls:  (1, 80, g, g)  ← 80 类置信度（图内已融合 sigmoid，直接和阈值比！）
//   sum:  (1, 1, g, g)   ← 总分（快速过滤用，也是原始值直接比）
// 关键：cls/sum 不要 sigmoid（模型导出时已 sigmoid 过），
//       否则背景格子的 ~0 值 sigmoid 后变 0.5，全部通过阈值 → 满屏小框
// ============================================================
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "rknn_infer.h"

// 三个尺度的 stride 和 grid
struct YoloScale { int stride; int grid; };
static const YoloScale YOLO_SCALES[] = {
    {8, 80}, {16, 40}, {32, 20}
};

// 输出索引约定：输出顺序 = [scale0_box, scale0_cls, scale0_sum,
//                           scale1_box, scale1_cls, scale1_sum,
//                           scale2_box, scale2_cls, scale2_sum]
#define NUM_CLASSES 80

// DFL 解码：16 bins softmax → 加权和（等价官方 compute_dfl）
static inline float dfl_decode(const float *dfl, int bin) {
    float sum = 0.0f, acc = 0.0f;
    float maxv = dfl[0];
    for (int i = 1; i < bin; i++) if (dfl[i] > maxv) maxv = dfl[i];
    for (int i = 0; i < bin; i++) {
        float e = expf(dfl[i] - maxv);
        sum += e;
        acc += e * i;
    }
    return acc / (sum + 1e-6f);
}

// 后处理：把 9 个输出转成检测框（640 输入空间像素坐标）
static void yolov8_postprocess(
    const std::vector<std::vector<float>> &outputs,
    std::vector<DetectBox> &boxes,
    float conf_thresh = 0.25f, float nms_thresh = 0.45f) {

    struct RawBox { float x1, y1, x2, y2, score; int cls; };
    std::vector<RawBox> raw;

    for (int s = 0; s < 3; s++) {
        int stride = YOLO_SCALES[s].stride;
        int grid = YOLO_SCALES[s].grid;
        int grid_len = grid * grid;

        // NCHW 布局：通道 c、格子 cell 的元素索引 = c*grid_len + cell
        const float *box_out  = outputs[s*3 + 0].data();  // (1,64,g,g)
        const float *cls_out  = outputs[s*3 + 1].data();  // (1,80,g,g) 已 sigmoid，原始值
        const float *sum_out  = outputs[s*3 + 2].data();  // (1,1,g,g)  原始值

        for (int gy = 0; gy < grid; gy++) {
            for (int gx = 0; gx < grid; gx++) {
                int cell = gy * grid + gx;

                // 快速过滤：score sum 原始值直接比（不 sigmoid）
                if (sum_out[cell] < conf_thresh) continue;

                // 找最大类（原始值直接比，不 sigmoid）
                int best_cls = -1; float best_score = 0;
                for (int c = 0; c < NUM_CLASSES; c++) {
                    float sc = cls_out[c * grid_len + cell];
                    if (sc > conf_thresh && sc > best_score) {
                        best_score = sc;
                        best_cls = c;
                    }
                }
                if (best_cls < 0) continue;

                // DFL 解码 4 个坐标（每坐标 16 bins，通道步长 = grid_len）
                float box[4];
                for (int b = 0; b < 4; b++) {
                    const float *dfl = box_out + (b * 16) * grid_len + cell;
                    box[b] = dfl_decode(dfl, 16);
                }

                // 官方坐标公式：x1 = (-box[0] + gx + 0.5) * stride
                float x1 = (-box[0] + gx + 0.5f) * stride;
                float y1 = (-box[1] + gy + 0.5f) * stride;
                float x2 = ( box[2] + gx + 0.5f) * stride;
                float y2 = ( box[3] + gy + 0.5f) * stride;

                if (x1 < 0) x1 = 0; if (y1 < 0) y1 = 0;
                if (x2 > 640) x2 = 640; if (y2 > 640) y2 = 640;

                raw.push_back({x1, y1, x2, y2, best_score, best_cls});
            }
        }
    }

    // NMS（对齐官方 postprocess.cc：按类分别抑制，overlap 公式带 +1）
    std::sort(raw.begin(), raw.end(),
              [](const RawBox &a, const RawBox &b) { return a.score > b.score; });
    std::vector<bool> removed(raw.size(), false);
    for (size_t i = 0; i < raw.size(); i++) {
        if (removed[i]) continue;
        boxes.push_back({raw[i].x1, raw[i].y1, raw[i].x2, raw[i].y2,
                         raw[i].score, raw[i].cls});
        for (size_t j = i + 1; j < raw.size(); j++) {
            if (removed[j] || raw[j].cls != raw[i].cls) continue;
            // 官方 CalculateOverlap
            float w = std::max(0.f, std::min(raw[i].x2, raw[j].x2) -
                                    std::max(raw[i].x1, raw[j].x1) + 1.f);
            float h = std::max(0.f, std::min(raw[i].y2, raw[j].y2) -
                                    std::max(raw[i].y1, raw[j].y1) + 1.f);
            float inter = w * h;
            float u = (raw[i].x2 - raw[i].x1 + 1.f) * (raw[i].y2 - raw[i].y1 + 1.f)
                    + (raw[j].x2 - raw[j].x1 + 1.f) * (raw[j].y2 - raw[j].y1 + 1.f)
                    - inter;
            float iou = (u <= 0.f) ? 0.f : (inter / u);
            if (iou > nms_thresh) removed[j] = true;
        }
    }
}
