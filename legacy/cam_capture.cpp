// ============================================================
// cam_capture.cpp —— 摄像头采集测试（独立验证 V4L2 取帧）
// 目的：验证 OV5695 → rkisp mainpath 取帧链路
// 功能：
//   1. 打开 /dev/video-camera0（= video42, rkisp mainpath）
//   2. 设置 NV12 2592x1944（Multiplanar）
//   3. 连续取 N 帧，打印帧信息
//   4. 保存第一帧：NV12 raw + 转成 PPM（可直接查看）
// 编译：g++ cam_capture.cpp v4l2_cam.cpp -o cam_capture
// ============================================================

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include "v4l2_cam.h"

#define CAM_PATH  "/dev/video-camera0"   // = /dev/video42
#define CAM_W     2592
#define CAM_H     1944
#define FRAMES    30                     // 取 30 帧

// NV12 → PPM（P6 格式，软件转换，最近邻缩放到 dw×dh 便于查看）
static void nv12_to_ppm(const uint8_t *y, const uint8_t *uv,
                        int sw, int sh,
                        const char *path, int dw, int dh) {
    FILE *fp = fopen(path, "wb");
    if (!fp) { printf("无法创建 %s\n", path); return; }
    fprintf(fp, "P6\n%d %d\n255\n", dw, dh);

    for (int dy = 0; dy < dh; dy++) {
        int sy = dy * sh / dh;
        for (int dx = 0; dx < dw; dx++) {
            int sx = dx * sw / dw;
            int uv_i = (sy/2) * sw + (sx & ~1u);
            int yy = y[sy*sw + sx];
            int u = uv[uv_i], v = uv[uv_i+1];
            int c = yy-16, d = u-128, e = v-128;
            int r = (298*c + 409*e + 128) >> 8;
            int g = (298*c - 100*d - 208*e + 128) >> 8;
            int b = (298*c + 516*d + 128) >> 8;
            if (r<0)r=0; if(r>255)r=255;
            if (g<0)g=0; if(g>255)g=255;
            if (b<0)b=0; if(b>255)b=255;
            fputc(r, fp); fputc(g, fp); fputc(b, fp);
        }
    }
    fclose(fp);
    printf("已保存 %s (%dx%d)\n", path, dw, dh);
}

int main() {
    // ---------- 1. 打开并配置摄像头 ----------
    V4l2Camera cam(CAM_PATH);
    try {
        cam.v4l2_open();
        cam.v4l2_query_cap();
        cam.v4l2_set_fmt(CAM_W, CAM_H);       // NV12 2592x1944
        cam.v4l2_req_buf(8);                  // 8 个缓冲
        cam.v4l2_full_queue(V4L2_MEMORY_MMAP);
        cam.v4l2_mmap();
        cam.v4l2_on_stream();
    } catch (const std::exception &e) {
        printf("❌ 摄像头初始化失败: %s\n", e.what());
        return -1;
    }
    printf("✅ 摄像头就绪: %s  %dx%d NV12\n", CAM_PATH, CAM_W, CAM_H);

    // ---------- 2. 取帧循环 ----------
    // 3A（AE/AWB）需要几十帧收敛，保存最后几帧
    for (int i = 0; i < FRAMES; i++) {
        V4l2Frame f = cam.v4l2_dequeue();

        // 打印 Y 均值，观察 AE 收敛（Y 应逐渐上升）
        const uint8_t *yplane = (const uint8_t*)f.data[0];
        uint64_t ysum = 0;
        for (int k = 0; k < CAM_W * CAM_H; k += 64)   // 抽样统计
            ysum += yplane[k];
        int ymean = (int)(ysum / ((CAM_W * CAM_H) / 64));
        printf("帧[%d]: index=%d nplanes=%u Y均值=%d\n",
               i, f.index, f.nplanes, ymean);

        // 保存最后 3 帧（3A 已收敛）
        if (i >= FRAMES - 3) {
            char nv12_path[64], ppm_path[64];
            snprintf(nv12_path, sizeof(nv12_path), "/userdata/frame%d.nv12", i);
            snprintf(ppm_path, sizeof(ppm_path), "/userdata/frame%d.ppm", i);

            // 保存 NV12 raw（完整 2592x1944）
            FILE *raw = fopen(nv12_path, "wb");
            if (raw) {
                if (f.nplanes >= 2) {
                    const uint8_t *y = (const uint8_t*)f.data[0];
                    const uint8_t *uv = (const uint8_t*)f.data[1];
                    fwrite(y, 1, (size_t)CAM_W*CAM_H, raw);
                    fwrite(uv, 1, (size_t)CAM_W*CAM_H/2, raw);
                } else {
                    fwrite(f.data[0], 1, (size_t)CAM_W*CAM_H*3/2, raw);
                }
                fclose(raw);
                printf("✅ 已保存 %s\n", nv12_path);
            }

            // 转 PPM 便于查看
            if (f.nplanes >= 2) {
                nv12_to_ppm((const uint8_t*)f.data[0],
                            (const uint8_t*)f.data[1],
                            CAM_W, CAM_H, ppm_path, 640, 480);
            } else {
                nv12_to_ppm((const uint8_t*)f.data[0],
                            (const uint8_t*)f.data[0] + (size_t)CAM_W*CAM_H,
                            CAM_W, CAM_H, ppm_path, 640, 480);
            }
        }

        cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
    }

    // ---------- 3. 关闭 ----------
    cam.v4l2_off_stream();
    printf("✅ 采集测试完成，共 %d 帧\n", FRAMES);
    return 0;
}
