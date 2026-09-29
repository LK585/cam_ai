// ============================================================
// cam_display3.cpp —— 双线程摄像头预览（30fps）
//   线程1 采集: V4L2 取帧(NV12) -> 共享最新帧（流卡死自动重启）
//   线程2 显示: 取最新帧 -> NV12->XRGB -> 写 fb[0]（30fps 节奏）
// 用法: ./cam_display3 [R增益] [B增益]   （默认 1.0 1.0）
//       软件白平衡手动调：R/B > 1 补偿偏绿（本板暗房实测约 R=1.7 B=1.6）
// 每 30 帧打印画面 R/G/B 均值，方便按需微调
// 退出：Ctrl+C / kill -TERM -> 干净关流
// ============================================================

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include "v4l2_cam.h"
#include "display.h"

#define CAM_PATH     "/dev/video-camera0"
#define CARD_PATH    "/dev/dri/card0"
#define CAM_W        1280
#define CAM_H        720
#define DISP_FPS     30

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// ---------------- 共享最新帧 ----------------
struct SharedFrame {
    std::mutex mtx;
    std::vector<uint8_t> nv12;
    bool fresh = false;
};

// ---------------- NV12 -> XRGB（显示尺寸，预计算缩放表） ----------------
// rg/bg: R/B 通道增益（1.0 = 标准不缩放），补偿无 AWB 的偏色
// sum_r/sum_g/sum_b: 输出本帧 R/G/B 累加值（均值用于调白平衡），可为 NULL
static void nv12_to_xrgb(const uint8_t *y, const uint8_t *uv,
                         int sw, int sh, uint8_t *dst,
                         int dw, int dh, int dpitch,
                         float rg, float bg,
                         long *sum_r, long *sum_g, long *sum_b) {
    int rGain = (int)(rg * 256.0f);
    int bGain = (int)(bg * 256.0f);
    long sr = 0, sg = 0, sb = 0;
    std::vector<int> ysrc(dh), xsrc(dw);
    for (int dy = 0; dy < dh; dy++) ysrc[dy] = dy * sh / dh;
    for (int dx = 0; dx < dw; dx++) xsrc[dx] = dx * sw / dw;
    for (int dy = 0; dy < dh; dy++) {
        int sy = ysrc[dy];
        const uint8_t *yrow  = y  + sy * sw;
        const uint8_t *uvrow = uv + (sy >> 1) * sw;
        uint8_t *row = dst + dy * dpitch;
        for (int dx = 0; dx < dw; dx++) {
            int sx  = xsrc[dx];
            int uv_i = (sx & ~1u);
            int yy = yrow[sx], u = uvrow[uv_i], v = uvrow[uv_i+1];
            int c = yy-16, d = u-128, e = v-128;
            int r = ((298*c + 409*e + 128) * rGain) >> 16;
            int g =  (298*c - 100*d - 208*e + 128) >> 8;
            int b = ((298*c + 516*d + 128) * bGain) >> 16;
            if (r<0)r=0; if(r>255)r=255;
            if (g<0)g=0; if(g>255)g=255;
            if (b<0)b=0; if(b>255)b=255;
            row[dx*4+0]=b; row[dx*4+1]=g; row[dx*4+2]=r; row[dx*4+3]=0xff;
            sr += r; sg += g; sb += b;
        }
    }
    if (sum_r) *sum_r = sr;
    if (sum_g) *sum_g = sg;
    if (sum_b) *sum_b = sb;
}

// ---------------- 线程1: 采集 ----------------
void capture_thread(V4l2Camera &cam, SharedFrame &sf, std::atomic<bool> &running) {
    long captured = 0;
    auto t_last = std::chrono::steady_clock::now();
    while (running) {
        try {
            int pr = cam.v4l2_poll(500);
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
            {
                std::lock_guard<std::mutex> lk(sf.mtx);
                const uint8_t *y = (const uint8_t*)f.data[0];
                const uint8_t *uv = (f.nplanes >= 2) ?
                    (const uint8_t*)f.data[1] : y + (size_t)CAM_W*CAM_H;
                size_t y_size = (size_t)CAM_W * CAM_H;
                sf.nv12.resize(y_size + y_size/2);
                memcpy(sf.nv12.data(), y, y_size);
                memcpy(sf.nv12.data() + y_size, uv, y_size/2);
                sf.fresh = true;
            }
            cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
            t_last = std::chrono::steady_clock::now();
            if (++captured % 30 == 0)
                printf("已采集 %ld 帧\n", captured);
        } catch (const std::exception &e) {
            printf("[警告] 采集异常: %s\n", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
}

// ---------------- 线程2: 显示（30fps） ----------------
void display_thread(struct drm_disp &drm, SharedFrame &sf, std::atomic<bool> &running,
                    float rgain, float bgain) {
    uint8_t *fb = (uint8_t*)drm.fb[0].vaddr;
    int pitch = drm.fb[0].pitch;
    size_t y_size = (size_t)CAM_W * CAM_H;
    long shown = 0;
    long npix = (long)drm.w * drm.h;
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
            long sr = 0, sg = 0, sb = 0;
            nv12_to_xrgb(nv12.data(), nv12.data() + y_size,
                         CAM_W, CAM_H, fb, drm.w, drm.h, pitch,
                         rgain, bgain, &sr, &sg, &sb);
            if (++shown % 30 == 0)
                printf("已显示 %ld 帧 均值R=%ld G=%ld B=%ld (增益 %.2f/%.2f)\n",
                       shown, sr/npix, sg/npix, sb/npix, rgain, bgain);
        }
        auto t1 = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
        auto target = std::chrono::milliseconds(1000 / DISP_FPS);
        if (el < target) std::this_thread::sleep_for(target - el);
    }
}

// ---------------- 主函数 ----------------
int main(int argc, char **argv) {
    // 用法: cam_display3 [R增益] [B增益]   （默认 1.0 1.0）
    float rgain = 1.0f, bgain = 1.0f;
    if (argc > 1) rgain = atof(argv[1]);
    if (argc > 2) bgain = atof(argv[2]);
    printf("参数: R增益=%.2f B增益=%.2f\n", rgain, bgain);

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

    struct drm_disp drm = {};
    Drm_display disp(CARD_PATH, drm);
    if (disp.Drm_open_display(drm) < 0) {
        printf("[错误] 显示打开失败: %s\n", CARD_PATH);
        cam.v4l2_off_stream();
        return -1;
    }
    printf("[OK] 显示就绪: %ux%u pitch=%u size=%u\n",
           drm.w, drm.h, drm.fb[0].pitch, drm.fb[0].size);

    SharedFrame sf;
    std::atomic<bool> running{true};
    std::thread t_cap(capture_thread, std::ref(cam), std::ref(sf), std::ref(running));
    std::thread t_disp(display_thread, std::ref(drm), std::ref(sf), std::ref(running),
                       rgain, bgain);

    printf("双线程预览中... Ctrl+C 退出（会干净关流）\n");
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    running = false;
    t_cap.join();
    t_disp.join();
    try { cam.v4l2_off_stream(); } catch (...) {}
    disp.Drm_close_display(drm);
    printf("正常退出\n");
    return 0;
}
