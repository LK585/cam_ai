#include <iostream>
#include <errno.h>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <libdrm/drm_fourcc.h>

#define FPS_MAX 2
//缓冲帧结构体
struct drm_fb{
    uint32_t pitch;     //行字节
	uint32_t handle;    //句柄
	uint32_t size;      //总大小
	void *vaddr;     //地址
	uint32_t fb_id;     //缓冲帧id
	int flip_done;      //是否已用 SetCrtc 建立过（后续用 PageFlip）
};
//显示结构体
struct drm_disp{
    uint32_t w;
    uint32_t h;
    drmModeModeInfo modeinfo;
    uint32_t conn_id;
    uint32_t crtc_id;
    struct drm_fb fb[FPS_MAX];
};

class Drm_display{
public:
    Drm_display(std::string path, struct drm_disp &drm_disp);
    ~Drm_display();
    int Drm_open_display(struct drm_disp &drm_disp);
    void Drm_close_display(struct drm_disp &drm_disp);
    int Drm_show(struct drm_disp &drm_disp,int index);
    int Drm_export_fb_fd(struct drm_fb &fb);   // 导出 fb 的 dma-buf fd（供 RGA 写），失败返回 -1
private:
    int Drm_create_fb(int fd, uint32_t w, uint32_t h, struct drm_fb &drm_fb);
    void Drm_destory_fb(int fd,struct drm_fb &drm_fb);
    int fd;
    std::string drm_path;
    drmModeResPtr drm_res = NULL;
    drmModeConnectorPtr drm_conn = NULL;
    drmModeEncoderPtr drm_enc = NULL;
};

