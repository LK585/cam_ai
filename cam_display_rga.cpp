// ============================================================
// cam_display_rga.cpp —— RGA 硬件加速预览（dma-buf fd 模式）
//   单线程：DQBUF -> RGA(相机dma-buf -> 显示dma-buf) -> QBUF
//   RGA 无 IOMMU，必须用 fd 模式（物理连续内存），不能用 vaddr 模式！
//   自动曝光: 每 30 帧采样 Y 均值，暗->加曝光/数字增益，亮->减
// 编译:
//   g++ -O2 cam_display_rga.cpp v4l2_cam.cpp display.cpp -o cam_display_rga \
//       -I./rga_include/im2d_api -I/usr/include/libdrm -ldrm -lpthread -lrga
// ============================================================
#include <cstdio>
#include <cstring>
#include <csignal>
#include <thread>
#include <chrono>
#include <algorithm>
#include <unistd.h>
#include "v4l2_cam.h"
#include "display.h"
#include "im2d.h"
#include "rga.h"

#define CAM_PATH     "/dev/video-camera0"
#define CARD_PATH    "/dev/dri/card0"
#define SENSOR_SUBDEV "/dev/v4l-subdev5"
#define CAM_W        1280
#define CAM_H        720

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// ---------------- 传感器 subdev（自动曝光） ----------------
struct SensorCtrl {
    int fd = -1;
    bool ok = false;
    int exposure = 1104;
    int dgain = 1024;
    bool sensor_open() {
        fd = open(SENSOR_SUBDEV, O_RDWR);
        if (fd < 0) { printf("[警告] 打不开传感器 %s\n", SENSOR_SUBDEV); return false; }
        ok = true;
        return true;
    }
    int set_ctrl(unsigned id, int val) {
        struct v4l2_control c;
        memset(&c, 0, sizeof(c));
        c.id = id; c.value = val;
        return ioctl(fd, VIDIOC_S_CTRL, &c);
    }
};

// ---------------- RGA: NV12(dma-buf) -> XRGB8888(dma-buf) + 缩放 ----------------
static bool rga_convert(int src_fd, int sw, int sh,
                        int dst_fd, int dw, int dh, int dpitch) {
    rga_buffer_t src  = wrapbuffer_fd(src_fd, sw, sh, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dstb = wrapbuffer_fd(dst_fd, dw, dh, RK_FORMAT_XRGB_8888, dpitch / 4, dh);
    im_rect srect = {0, 0, sw, sh};
    im_rect drect = {0, 0, dw, dh};
    rga_buffer_t pat = {};
    im_rect prect = {0, 0, 0, 0};
    // 10 参数版 improcess（自己创建作业）；不要传 ctx_id
    IM_STATUS st = improcess(src, dstb, pat, srect, drect, prect, -1, NULL, NULL, 0);
    if (st != IM_STATUS_SUCCESS) {
        printf("RGA improcess 失败: %s\n", imStrError(st));
        return false;
    }
    return true;
}

// Y 平面均值（抽样，每 4 像素取 1）
static int y_mean(const uint8_t *y, int w, int h) {
    long sum = 0, n = 0;
    for (int yy = 0; yy < h; yy += 4)
        for (int xx = 0; xx < w; xx += 4) { sum += y[yy * w + xx]; n++; }
    return n ? (int)(sum / n) : 0;
}

// ---------------- 主流程（单线程） ----------------
int main() {
    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);
    signal(SIGHUP,  on_signal);
    signal(SIGPIPE, SIG_IGN);

    V4l2Camera cam(CAM_PATH);
    try {
        cam.v4l2_open();
        cam.v4l2_query_cap();
        cam.v4l2_set_fmt(CAM_W, CAM_H);
        cam.v4l2_req_buf(8);
        cam.v4l2_full_queue(V4L2_MEMORY_MMAP);
        cam.v4l2_mmap();
        cam.v4l2_on_stream();
    } catch (const std::exception &e) {
        printf("[错误] 摄像头失败: %s\n", e.what());
        return -1;
    }
    printf("[OK] 摄像头就绪\n");

    // 导出 8 个相机缓冲的 dma-buf fd（RGA 源）
    int cam_fd[8];
    for (int i = 0; i < 8; i++) {
        cam_fd[i] = cam.v4l2_export_fd(i);
        if (cam_fd[i] < 0) { printf("[错误] 相机缓冲 %d 导出失败\n", i); return -1; }
    }
    printf("[OK] 相机 dma-buf 导出成功\n");

    struct drm_disp drm = {};
    Drm_display disp(CARD_PATH, drm);
    if (disp.Drm_open_display(drm) < 0) {
        printf("[错误] 显示打开失败: %s\n", CARD_PATH);
        cam.v4l2_off_stream();
        return -1;
    }
    printf("[OK] 显示就绪: %ux%u pitch=%u size=%u\n",
           drm.w, drm.h, drm.fb[0].pitch, drm.fb[0].size);

    // 导出显示 fb 的 dma-buf fd（RGA 目标）
    int dst_fd = disp.Drm_export_fb_fd(drm.fb[0]);
    if (dst_fd < 0) { printf("[错误] 显示 fb 导出失败\n"); return -1; }
    printf("[OK] 显示 dma-buf 导出成功\n");

    SensorCtrl sc;
    sc.sensor_open();

    // 单线程循环：DQBUF -> RGA -> QBUF
    long frame_count = 0;
    auto t_last = std::chrono::steady_clock::now();
    while (!g_stop) {
        try {
            int pr = cam.v4l2_poll(500);
            if (g_stop) break;
            if (pr <= 0) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last).count() > 3000) {
                    printf("[警告] 流卡死，重启流\n");
                    cam.v4l2_off_stream();
                    cam.v4l2_full_queue(V4L2_MEMORY_MMAP);
                    cam.v4l2_on_stream();
                    t_last = now;
                }
                continue;
            }

            V4l2Frame f = cam.v4l2_dequeue();
            if (rga_convert(cam_fd[f.index], CAM_W, CAM_H,
                            dst_fd, drm.w, drm.h, drm.fb[0].pitch)) {
                frame_count++;
                if (frame_count % 30 == 0) {
                    int ym = y_mean((const uint8_t*)cam.v4l2_buf_data(f.index), CAM_W, CAM_H);
                    printf("已显示 %ld 帧 Y均值=%d (曝光=%d dgain=%d)\n",
                           frame_count, ym, sc.exposure, sc.dgain);
                    // AE: 每 30 帧调一次
                    if (sc.ok) {
                        if (ym < 108 && sc.exposure < 2020) {
                            sc.exposure = std::min(2020, sc.exposure + sc.exposure/8 + 8);
                            sc.set_ctrl(V4L2_CID_EXPOSURE, sc.exposure);
                        } else if (ym < 108 && sc.dgain < 8192) {
                            sc.dgain = std::min(8192, sc.dgain * 2);
                            sc.set_ctrl(V4L2_CID_DIGITAL_GAIN, sc.dgain);
                        } else if (ym > 132 && sc.dgain > 1024) {
                            sc.dgain = std::max(1024, sc.dgain / 2);
                            sc.set_ctrl(V4L2_CID_DIGITAL_GAIN, sc.dgain);
                        } else if (ym > 132 && sc.exposure > 100) {
                            sc.exposure = std::max(100, sc.exposure - sc.exposure/8 - 8);
                            sc.set_ctrl(V4L2_CID_EXPOSURE, sc.exposure);
                        }
                    }
                }
            }
            cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
            t_last = std::chrono::steady_clock::now();
        } catch (const std::exception &e) {
            printf("[警告] 异常: %s\n", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    try { cam.v4l2_off_stream(); } catch (...) {}
    disp.Drm_close_display(drm);
    printf("正常退出，共显示 %ld 帧\n", frame_count);
    return 0;
}
