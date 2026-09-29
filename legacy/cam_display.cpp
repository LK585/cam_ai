// ============================================================
// cam_display.cpp —— 摄像头视频流 -> MIPI 屏幕持续显示（最小验证）
// 两线程：
//   线程1 采集: V4L2 NV12 2592x1944 -> 共享最新帧
//   线程2 显示: NV12 -> XRGB（软件白平衡补偿）-> DRM 双缓冲显示
// 编译：g++ cam_display.cpp v4l2_cam.cpp display.cpp -o cam_display -ldrm -lpthread
// ============================================================

#include <cstdio>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include "v4l2_cam.h"
#include "display.h"

#define CAM_PATH     "/dev/video-camera0"   // = /dev/video42
#define CARD_PATH    "/dev/dri/card0"       // MIPI 屏（by-path 确认）
#define CAM_W        2592
#define CAM_H        1944
#define DISP_FPS     30

// 软件白平衡增益（补偿 RK3576 无 3A 的偏绿）
// 实测：G/R=1.26 -> R×1.26；B 略低 -> B×1.10
#define WB_R_GAIN    126    // R 增益 ×1.26
#define WB_B_GAIN    110    // B 增益 ×1.10

// 共享帧
struct SharedFrame {
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<uint8_t> nv12;
    bool fresh = false;
};

// NV12 -> XRGB（显示尺寸，最近邻缩放 + 软件白平衡）
static void nv12_to_xrgb_wb(const uint8_t *y, const uint8_t *uv,
                            int sw, int sh, uint8_t *dst,
                            int dw, int dh, int dpitch) {
    for (int dy = 0; dy < dh; dy++) {
        int sy = dy * sh / dh;
        uint8_t *row = dst + dy * dpitch;
        for (int dx = 0; dx < dw; dx++) {
            int sx = dx * sw / dw;
            int uv_i = (sy/2)*sw + (sx & ~1u);
            int yy = y[sy*sw+sx], u = uv[uv_i], v = uv[uv_i+1];
            int c = yy-16, d = u-128, e = v-128;
            // 标准 YUV->RGB，然后 R/B 增益补偿偏绿
            int r = ((298*c + 409*e + 128) * WB_R_GAIN) >> 16;
            int g =  (298*c - 100*d - 208*e + 128) >> 8;
            int b = ((298*c + 516*d + 128) * WB_B_GAIN) >> 16;
            if (r<0)r=0; if(r>255)r=255;
            if (g<0)g=0; if(g>255)g=255;
            if (b<0)b=0; if(b>255)b=255;
            row[dx*4+0]=b; row[dx*4+1]=g; row[dx*4+2]=r; row[dx*4+3]=0xff;
        }
    }
}

// 线程1: 采集
void capture_thread(V4l2Camera &cam, SharedFrame &sf, std::atomic<bool> &running) {
    while (running) {
        V4l2Frame f = cam.v4l2_dequeue();
        {
            std::lock_guard<std::mutex> lk(sf.mtx);
            const uint8_t *y = (const uint8_t*)f.data[0];
            const uint8_t *uv;
            size_t y_size = (size_t)CAM_W * CAM_H;
            size_t uv_size = y_size / 2;
            if (f.nplanes >= 2) uv = (const uint8_t*)f.data[1];
            else uv = y + y_size;
            sf.nv12.resize(y_size + uv_size);
            memcpy(sf.nv12.data(), y, y_size);
            memcpy(sf.nv12.data() + y_size, uv, uv_size);
            sf.fresh = true;
        }
        sf.cv.notify_all();
        cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
    }
}

// 线程2: 显示（30fps）
void display_thread(Drm_display &disp, struct drm_disp &drm,
                    SharedFrame &sf, std::atomic<bool> &running) {
    int buf_idx = 0;
    std::vector<uint8_t> xrgb((size_t)drm.w * drm.h * 4);
    while (running) {
        auto t0 = std::chrono::steady_clock::now();
        bool got = false;
        std::vector<uint8_t> nv12;
        {
            std::lock_guard<std::mutex> lk(sf.mtx);
            if (sf.fresh && !sf.nv12.empty()) {
                nv12 = sf.nv12;
                sf.fresh = false;
                got = true;
            }
        }
        if (got) {
            printf("D: 转换中...\n"); fflush(stdout);
            nv12_to_xrgb_wb(nv12.data(), nv12.data()+(size_t)CAM_W*CAM_H,
                            CAM_W, CAM_H, xrgb.data(), drm.w, drm.h, drm.fb[0].pitch);
            printf("D: memcpy fb[%d] size=%u\n", buf_idx, drm.fb[buf_idx].size);
            fflush(stdout);
            memcpy(drm.fb[buf_idx].vaddr, xrgb.data(), drm.fb[buf_idx].size);
            printf("D: Drm_show...\n"); fflush(stdout);
            disp.Drm_show(drm, buf_idx);
            printf("D: 显示完成\n"); fflush(stdout);
            buf_idx = 1 - buf_idx;
        }
        auto t1 = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
        auto target = std::chrono::milliseconds(1000/DISP_FPS);
        if (el < target) std::this_thread::sleep_for(target - el);
    }
}

int main() {
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

    struct drm_disp drm = {};
    Drm_display disp(CARD_PATH, drm);
    if (disp.Drm_open_display(drm) < 0) {
        printf("[错误] 显示打开失败: %s\n", CARD_PATH);
        return -1;
    }
    printf("[OK] 显示就绪: %ux%u (card0)\n", drm.w, drm.h);

    SharedFrame sf;
    std::atomic<bool> running{true};
    std::thread t_cap(capture_thread, std::ref(cam), std::ref(sf), std::ref(running));
    std::thread t_disp(display_thread, std::ref(disp), std::ref(drm), std::ref(sf), std::ref(running));

    printf("🎥 摄像头视频流显示中（30fps），Ctrl+C 退出\n");
    getchar();

    running = false;
    t_cap.join(); t_disp.join();
    disp.Drm_close_display(drm);
    cam.v4l2_off_stream();
    return 0;
}
