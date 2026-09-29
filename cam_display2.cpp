// ============================================================
// cam_display2.cpp —— 摄像头视频流 -> MIPI 屏（单线程，持续预览版）
// 改进（保证"一直显示"）：
//  1) 主循环 try/catch —— 取帧/入队失败不再让程序退出
//  2) poll 带 3 秒超时 —— 流卡死（无帧）时自动重启流
//  3) SIGTERM/SIGINT 干净关流再退出 —— 避免上次强杀污染下次开流
//     （"8帧后自己退出"的间歇现象，很大程度是上次 timeout 强杀留下的脏状态）
// ============================================================

#include <cstdio>
#include <cstring>
#include <csignal>
#include <vector>
#include <ctime>
#include <unistd.h>
#include "v4l2_cam.h"
#include "display.h"

#define CAM_PATH     "/dev/video-camera0"
#define CARD_PATH    "/dev/dri/card0"
#define CAM_W        1280
#define CAM_H        720

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// NV12 -> XRGB（显示尺寸），预计算缩放表，去掉每像素除法（30fps 必需）
// 标准 YUV->RGB（BT.601），和朋友代码一致（不加白平衡/增益）
static void nv12_to_xrgb(const uint8_t *y, const uint8_t *uv,
                         int sw, int sh, uint8_t *dst,
                         int dw, int dh, int dpitch) {
    std::vector<int> ysrc(dh), xsrc(dw);
    for (int dy = 0; dy < dh; dy++) ysrc[dy] = dy * sh / dh;
    for (int dx = 0; dx < dw; dx++) xsrc[dx] = dx * sw / dw;
    for (int dy = 0; dy < dh; dy++) {
        int sy = ysrc[dy];
        const uint8_t *yrow  = y  + sy * sw;
        const uint8_t *uvrow = uv + (sy >> 1) * sw;   // NV12: 每行 UV 跨距 = sw 字节
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

int main() {
    signal(SIGTERM, on_signal);   // timeout / kill -TERM
    signal(SIGINT,  on_signal);   // Ctrl+C
    signal(SIGHUP,  on_signal);   // 终端/adb 连接断开
    signal(SIGPIPE, SIG_IGN);     // stdout 关闭时不要被 SIGPIPE 杀死

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
        printf("[错误] 显示打开失败\n");
        return -1;
    }
    printf("[OK] 显示就绪: %ux%u pitch=%u size=%u\n",
           drm.w, drm.h, drm.fb[0].pitch, drm.fb[0].size);
    printf("持续预览中... Ctrl+C 退出（会干净关流）\n");

    // 重启流（STREAMOFF -> 全部重新入队 -> STREAMON）
    auto restart_stream = [&]() -> bool {
        try {
            cam.v4l2_off_stream();
            cam.v4l2_full_queue(V4L2_MEMORY_MMAP);
            cam.v4l2_on_stream();
            printf("[OK] 流已重启，继续预览\n");
            return true;
        } catch (const std::exception &e) {
            printf("[错误] 重启流失败: %s\n", e.what());
            return false;
        }
    };

    // 持续预览循环：SetCrtc 只在 Drm_open_display 里做过一次，
    // VOP 持续扫描该 buffer，每帧写新数据即刷新显示
    long frame_count = 0;
    long restart_cnt = 0;
    while (!g_stop) {
        try {
            // 等新帧（3 秒超时；流卡死时 poll 超时 -> 重启流）
            int pr = cam.v4l2_poll(3000);
            if (g_stop) break;
            if (pr == 0) {
                printf("[警告] 3秒无新帧（流卡死），重启流 #%ld\n", ++restart_cnt);
                if (!restart_stream()) break;
                continue;
            }
            if (pr < 0) {
                perror("[警告] poll 出错");
                if (!restart_stream()) break;
                continue;
            }

            V4l2Frame f = cam.v4l2_dequeue();
            const uint8_t *y = (const uint8_t*)f.data[0];
            const uint8_t *uv = (f.nplanes >= 2) ?
                (const uint8_t*)f.data[1] : y + (size_t)CAM_W*CAM_H;

            nv12_to_xrgb(y, uv, CAM_W, CAM_H,
                         (uint8_t*)drm.fb[0].vaddr,
                         drm.w, drm.h, drm.fb[0].pitch);

            cam.v4l2_queue(V4L2_MEMORY_MMAP, f.index);
            frame_count++;
            if (frame_count % 30 == 0)
                printf("已显示 %ld 帧\n", frame_count);
        } catch (const std::exception &e) {
            printf("[警告] 取帧异常: %s -> 重启流 #%ld\n", e.what(), ++restart_cnt);
            if (!restart_stream()) break;
        }
    }

    // 干净关流（重要：避免脏状态影响下一次开流）
    try { cam.v4l2_off_stream(); } catch (...) {}
    printf("正常退出，共显示 %ld 帧（重启流 %ld 次）\n", frame_count, restart_cnt);
    return 0;
}
