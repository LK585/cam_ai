// ============================================================
// main.cpp —— RK3576 板端"摄像头→RKNN推理→MIPI屏显示"三线程程序
// 架构：
//   线程1 采集: V4L2 取帧(NV12) → 共享最新帧
//   线程2 推理: NV12→640x640 RGB → RKNN → 检测结果
//   线程3 显示: NV12→XRGB + 画框 → DRM 显示（30fps 刷新）
// 零 OpenCV 依赖
// 已修正（相对初版）：
//   - 显示线程不再每帧 SetCrtc（RK3576 每次 ~1s 会卡死）→ 只写 fb[0]
//   - 分辨率 1280x720（已验证稳定）
//   - 采集线程带 poll 超时 + 异常保护，可干净退出
//   - SIGTERM/SIGINT 干净关流（timeout 强杀不再污染下次开流）
// ============================================================

#include <cstdio>
#include <cstring>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <string>
#include "v4l2_cam.h"
#include "display.h"
#include "rknn_infer.h"
#include "postprocess.h"

#define CAM_PATH     "/dev/video-camera0"
#define CARD_PATH    "/dev/dri/card0"
#define CAM_W        1280
#define CAM_H        720
#define AI_SIZE      640
#define DISP_FPS     30
#define MODEL_PATH   "/userdata/cam_ai/yolov8.rknn"
#define LABEL_PATH   "/userdata/cam_ai/coco_80_labels_list.txt"

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// ---------------- 共享结构 ----------------
struct SharedFrame {
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<uint8_t> nv12;
    bool fresh = false;
};
struct SharedResult {
    std::mutex mtx;
    std::vector<DetectBox> boxes;
    bool fresh = false;
};

// ---------------- 工具函数 ----------------

// 读取类别名文件（每行一个名字）
static std::vector<std::string> load_labels(const char *path) {
    std::vector<std::string> labels;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line))
        labels.push_back(line);
    return labels;
}

// NV12 → RGB(AI_SIZE x AI_SIZE)，最近邻缩放（喂模型）
static void nv12_to_rgb640(const uint8_t *y, const uint8_t *uv,
                           int sw, int sh, uint8_t *rgb) {
    for (int dy = 0; dy < AI_SIZE; dy++) {
        int sy = dy * sh / AI_SIZE;
        for (int dx = 0; dx < AI_SIZE; dx++) {
            int sx = dx * sw / AI_SIZE;
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
            rgb[(dy*AI_SIZE+dx)*3+0] = r;
            rgb[(dy*AI_SIZE+dx)*3+1] = g;
            rgb[(dy*AI_SIZE+dx)*3+2] = b;
        }
    }
}

// 画框（XRGB8888）
static void draw_box(uint8_t *xrgb, int pitch, int w, int h,
                     int x1, int y1, int x2, int y2) {
    if (x1<0)x1=0; if(y1<0)y1=0; if(x2>=w)x2=w-1; if(y2>=h)y2=h-1;
    for (int t = 0; t < 3; t++) {
        for (int x = x1; x <= x2; x++) {
            uint8_t *a = xrgb + ((y1+t)*pitch + x*4);
            uint8_t *b = xrgb + ((y2-t)*pitch + x*4);
            a[0]=0; a[1]=255; a[2]=0; a[3]=0xff;
            b[0]=0; b[1]=255; b[2]=0; b[3]=0xff;
        }
        for (int yy = y1; yy <= y2; yy++) {
            uint8_t *a = xrgb + (yy*pitch + (x1+t)*4);
            uint8_t *b = xrgb + (yy*pitch + (x2-t)*4);
            a[0]=0; a[1]=255; a[2]=0; a[3]=0xff;
            b[0]=0; b[1]=255; b[2]=0; b[3]=0xff;
        }
    }
}

// NV12 → XRGB（显示尺寸，预计算缩放表，无每像素除法）
static void nv12_to_xrgb(const uint8_t *y, const uint8_t *uv,
                         int sw, int sh, uint8_t *dst,
                         int dw, int dh, int dpitch) {
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
            int r = (298*c + 409*e + 128) >> 8;
            int g = (298*c - 100*d - 208*e + 128) >> 8;
            int b = (298*c + 516*d + 128) >> 8;
            if (r<0)r=0; if(r>255)r=255;
            if (g<0)g=0; if(g>255)g=255;
            if (b<0)b=0; if(b>255)b=255;
            row[dx*4+0]=b; row[dx*4+1]=g; row[dx*4+2]=r; row[dx*4+3]=0xff;
        }
    }
}

// ---------------- 线程1: 采集 ----------------
void capture_thread(V4l2Camera &cam, SharedFrame &sf, std::atomic<bool> &running) {
    while (running) {
        try {
            int pr = cam.v4l2_poll(500);      // 500ms 超时，保证能退出
            if (pr <= 0) continue;            // 超时/出错 → 等下一轮
            V4l2Frame f = cam.v4l2_dequeue();
            {
                std::lock_guard<std::mutex> lk(sf.mtx);
                const uint8_t *y = (const uint8_t*)f.data[0];
                const uint8_t *uv;
                size_t y_size = (size_t)CAM_W * CAM_H;
                size_t uv_size = y_size / 2;
                if (f.nplanes >= 2) {
                    uv = (const uint8_t*)f.data[1];   // 多平面：Y 和 UV 分开
                } else {
                    uv = y + y_size;                  // 单平面连续 NV12
                }
                sf.nv12.resize(y_size + uv_size);
                memcpy(sf.nv12.data(), y, y_size);
                memcpy(sf.nv12.data() + y_size, uv, uv_size);
                sf.fresh = true;
            }
            sf.cv.notify_all();
            cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
        } catch (const std::exception &e) {
            printf("⚠️ 采集异常: %s\n", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
}

// ---------------- 线程2: 推理 ----------------
void infer_thread(RknnInfer &model, SharedFrame &sf, SharedResult &sr,
                  std::atomic<bool> &running,
                  const std::vector<std::string> &labels) {
    std::vector<uint8_t> rgb(AI_SIZE*AI_SIZE*3);
    long infer_cnt = 0;
    while (running) {
        std::unique_lock<std::mutex> lk(sf.mtx);
        sf.cv.wait(lk, [&]{ return sf.fresh || !running; });
        if (!running) break;
        std::vector<uint8_t> nv12 = sf.nv12;   // 拷贝
        sf.fresh = false;
        lk.unlock();

        nv12_to_rgb640(nv12.data(), nv12.data()+(size_t)CAM_W*CAM_H,
                       CAM_W, CAM_H, rgb.data());

        std::vector<std::vector<float>> outputs;
        int ret = model.infer_rgb(rgb.data(), AI_SIZE, AI_SIZE, outputs);
        if (ret == 0) {
            std::vector<DetectBox> boxes;
            yolov8_postprocess(outputs, boxes);
            {
                std::lock_guard<std::mutex> lk2(sr.mtx);
                sr.boxes = boxes;
                sr.fresh = true;
            }
            if (++infer_cnt % 30 == 0) {
                printf("推理 %ld 次，检测到 %zu 个目标:", infer_cnt, boxes.size());
                for (auto &b : boxes) {
                    const char *name = (b.class_id >= 0 && b.class_id < (int)labels.size())
                                           ? labels[b.class_id].c_str() : "?";
                    printf(" %s(%.2f)", name, b.score);
                }
                printf("\n");
            }
        }
    }
}

// ---------------- 线程3: 显示（30fps） ----------------
void display_thread(struct drm_disp &drm,
                    SharedFrame &sf, SharedResult &sr,
                    std::atomic<bool> &running) {
    uint8_t *fb = (uint8_t*)drm.fb[0].vaddr;
    int pitch = drm.fb[0].pitch;
    while (running) {
        auto t0 = std::chrono::steady_clock::now();
        bool got = false;
        std::vector<uint8_t> nv12;
        {
            std::lock_guard<std::mutex> lk(sf.mtx);
            if (sf.fresh && !sf.nv12.empty()) {
                nv12 = sf.nv12;
                sf.fresh = false;   // 显示线程消费后重置
                got = true;
            }
        }
        if (got) {
            nv12_to_xrgb(nv12.data(), nv12.data()+(size_t)CAM_W*CAM_H,
                         CAM_W, CAM_H, fb, drm.w, drm.h, pitch);
            std::vector<DetectBox> boxes;
            {
                std::lock_guard<std::mutex> lk(sr.mtx);
                boxes = sr.boxes;
            }
            for (auto &b : boxes) {
                float kx = (float)drm.w / AI_SIZE, ky = (float)drm.h / AI_SIZE;
                draw_box(fb, pitch, drm.w, drm.h,
                         (int)(b.box[0]*kx), (int)(b.box[1]*ky),
                         (int)(b.box[2]*kx), (int)(b.box[3]*ky));
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
        auto target = std::chrono::milliseconds(1000/DISP_FPS);
        if (el < target) std::this_thread::sleep_for(target - el);
    }
}

// ---------------- 主函数 ----------------
int main() {
    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);

    V4l2Camera cam(CAM_PATH);
    try {
        cam.v4l2_open();
        cam.v4l2_query_cap();
        cam.v4l2_set_fmt(CAM_W, CAM_H);
        cam.v4l2_req_buf();
        cam.v4l2_full_queue(V4L2_MEMORY_MMAP);
        cam.v4l2_mmap();
        cam.v4l2_on_stream();
    } catch (const std::exception &e) {
        printf("❌ 摄像头失败: %s\n", e.what());
        return -1;
    }
    printf("✅ 摄像头就绪\n");

    struct drm_disp drm = {};
    Drm_display disp(CARD_PATH, drm);
    if (disp.Drm_open_display(drm) < 0) {
        printf("❌ 显示打开失败: %s\n", CARD_PATH);
        cam.v4l2_off_stream();
        return -1;
    }
    printf("✅ 显示就绪: %ux%u\n", drm.w, drm.h);

    RknnInfer model;
    if (model.init(MODEL_PATH) < 0) {
        printf("❌ 模型加载失败: %s\n", MODEL_PATH);
        return -1;
    }
    printf("✅ RKNN 就绪\n");

    std::vector<std::string> labels = load_labels(LABEL_PATH);
    if (labels.empty()) {
        printf("❌ 类别文件加载失败: %s（请先拷贝 coco_80_labels_list.txt 到工程目录）\n", LABEL_PATH);
        return -1;
    }
    printf("✅ 类别加载 %zu 个，第一个: %s\n", labels.size(), labels[0].c_str());

    SharedFrame sf; SharedResult sr;
    std::atomic<bool> running{true};
    std::thread t_cap(capture_thread, std::ref(cam), std::ref(sf), std::ref(running));
    std::thread t_inf(infer_thread, std::ref(model), std::ref(sf), std::ref(sr), std::ref(running), std::ref(labels));
    std::thread t_disp(display_thread, std::ref(drm), std::ref(sf), std::ref(sr), std::ref(running));

    printf("三线程运行中...（Ctrl+C / timeout 退出）\n");
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    running = false;
    sf.cv.notify_all();
    t_cap.join(); t_inf.join(); t_disp.join();
    try { cam.v4l2_off_stream(); } catch (...) {}
    disp.Drm_close_display(drm);
    printf("正常退出\n");
    return 0;
}
