// ============================================================
// rknn_infer.h —— RKNN 推理封装（独立实现，只依赖 rknn_api.h）
// 功能：加载 .rknn -> 输入 RGB -> 推理 -> 输出检测框
// 说明：官方 yolov8.rknn 是多输出结构（优化版模型），
//       后处理需按官方输出格式；此处提供通用推理接口，
//       后处理在 postprocess 中实现。
// ============================================================
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include "rknn_api.h"

// 检测框结果
typedef struct {
    float box[4];       // left, top, right, bottom (像素坐标)
    float score;        // 置信度 0~1
    int class_id;       // 类别编号
} DetectBox;

class RknnInfer {
public:
    RknnInfer() : ctx_(0), inited_(false) {}
    ~RknnInfer() { release(); }

    // 加载 .rknn 模型
    int init(const char *model_path) {
        int ret = rknn_init(&ctx_, (void*)model_path, 0, 0, NULL);
        if (ret < 0) return ret;
        inited_ = true;

        // 查询输入输出数量
        rknn_input_output_num io_num;
        rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
        n_input_ = io_num.n_input;
        n_output_ = io_num.n_output;
        printf("RKNN: %d 输入, %d 输出\n", n_input_, n_output_);

        // 查询输入输出形状（写后处理的关键依据）
        for (int i = 0; i < n_input_; i++) {
            rknn_tensor_attr attr;
            memset(&attr, 0, sizeof(attr));
            attr.index = i;
            if (rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr)) == 0) {
                printf("输入[%d]: dims=[%d,%d,%d,%d] type=%d fmt=%d\n",
                       i, attr.dims[0], attr.dims[1], attr.dims[2], attr.dims[3],
                       attr.type, attr.fmt);
            }
        }
        for (int i = 0; i < n_output_; i++) {
            rknn_tensor_attr attr;
            memset(&attr, 0, sizeof(attr));
            attr.index = i;
            if (rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) == 0) {
                printf("输出[%d]: dims=[%d,%d,%d,%d] type=%d\n",
                       i, attr.dims[0], attr.dims[1], attr.dims[2], attr.dims[3],
                       attr.type);
            }
        }
        return 0;
    }

    // 推理：输入 RGB（640x640x3，raw uint8，NHWC），输出原始输出张量
    // 注意：ultralytics 导出的 ONNX 图内自带 /255 归一化，转换后应直接喂
    // raw 0~255（与官方 rknn_model_zoo demo 一致），软件里不要再 /255
    int infer_rgb(const uint8_t *rgb, int w, int h,
                  std::vector<std::vector<float>> &outputs) {
        if (!inited_) return -1;

        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = (uint32_t)((size_t)w * h * 3);
        inputs[0].fmt = RKNN_TENSOR_NHWC;   // 模型内部布局由驱动处理
        inputs[0].buf = (void*)rgb;
        if (rknn_inputs_set(ctx_, 1, inputs) < 0) return -2;

        if (rknn_run(ctx_, NULL) < 0) return -3;

        // 输出
        std::vector<rknn_output> outs(n_output_);
        for (int i = 0; i < n_output_; i++) {
            memset(&outs[i], 0, sizeof(outs[i]));
            outs[i].want_float = 1;          // 反量化
        }
        if (rknn_outputs_get(ctx_, n_output_, outs.data(), NULL) < 0) return -4;

        outputs.resize(n_output_);
        for (int i = 0; i < n_output_; i++) {
            size_t sz = outs[i].size / sizeof(float);
            float *p = (float*)outs[i].buf;
            outputs[i].assign(p, p + sz);
        }
        rknn_outputs_release(ctx_, n_output_, outs.data());
        return 0;
    }

    void release() {
        if (inited_) { rknn_destroy(ctx_); inited_ = false; }
    }

private:
    rknn_context ctx_;
    bool inited_;
    int n_input_ = 0, n_output_ = 0;
};
