#include "display.h"
#include <libdrm/drm.h>

Drm_display::Drm_display(std::string path, struct drm_disp &drm_disp):
    fd(-1),drm_path(path),drm_res(NULL),drm_conn(NULL),drm_enc(NULL)
{
}

Drm_display::~Drm_display(){
    /* 由调用方 Drm_close_display(disp) 释放；此处无成员 drm_disp */
}

int Drm_display::Drm_create_fb(int fd, uint32_t w, uint32_t h, struct drm_fb &drm_fb){
    int ret;
    struct drm_mode_create_dumb cdumb = {};
    struct drm_mode_map_dumb mdumb = {};
    struct drm_mode_destroy_dumb ddumb = {};
    uint32_t handles[4] = { 0 }, pitches[4] = { 0 }, offsets[4] = { 0 };
    cdumb.width = w;
    cdumb.height = h;
    cdumb.bpp = 32;
    ret = drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cdumb);
    if(ret < 0){
        goto err_CREATE_DUMB;
    }
    drm_fb.handle = cdumb.handle;
    drm_fb.pitch = cdumb.pitch;
    drm_fb.size = cdumb.size;

    handles[0] = drm_fb.handle;
    pitches[0] = drm_fb.pitch;
    ret = drmModeAddFB2(fd, w, h, DRM_FORMAT_XRGB8888, handles, pitches, offsets, &drm_fb.fb_id, 0);
    if(ret < 0){
        goto err_AddFB2;
    }
    mdumb.handle = drm_fb.handle;

    ret = drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mdumb);
    if(ret < 0){
        goto err_MAP_DUMB;
    }

    drm_fb.vaddr = mmap(0, drm_fb.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mdumb.offset);
    if(drm_fb.vaddr == NULL){
        goto err_mmap;
    }
    memset(drm_fb.vaddr, 0, drm_fb.size);
    return 0;
err_mmap:
err_MAP_DUMB:
    drmModeRmFB(fd, drm_fb.fb_id);
err_AddFB2:
    memset(&ddumb, 0, sizeof(ddumb));
    ddumb.handle = drm_fb.handle;
    drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &ddumb);
err_CREATE_DUMB:
    return ret;
}

void Drm_display::Drm_destory_fb(int fd,struct drm_fb &drm_fb){
    if(drm_fb.vaddr){
        munmap(drm_fb.vaddr, drm_fb.size);
    }
    if(drm_fb.fb_id){
        drmModeRmFB(fd, drm_fb.fb_id);
    }
    if(drm_fb.handle){
        struct drm_mode_destroy_dumb ddumb = {};
        memset(&ddumb, 0, sizeof(ddumb));
        ddumb.handle = drm_fb.handle;
        drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &ddumb);
    }

}

int Drm_display::Drm_open_display(struct drm_disp &drm_disp){
    int ret = -1;
    fd = open(drm_path.c_str(), O_RDWR);
    drm_res = drmModeGetResources(fd);

    if(!drm_res){
        goto err_drm_res;
    }
    //获取物理显示器
    for(int i = 0;i < drm_res->count_connectors;i++){
        drm_conn = drmModeGetConnector(fd, drm_res->connectors[i]);
        if(drm_conn == NULL){
            continue;
        }
        if(drm_conn->connection == DRM_MODE_CONNECTED && drm_conn->count_modes > 0){
            break;
        }
        drmModeFreeConnector(drm_conn);
        drm_conn = NULL;
    }
    if(drm_conn == NULL){
        goto err_drm_conn;
    }
    drm_disp.conn_id = drm_conn->connector_id;
    drm_disp.w = drm_conn->modes[0].hdisplay;
    drm_disp.h = drm_conn->modes[0].vdisplay;
    drm_disp.modeinfo = drm_conn->modes[0];
    //获取编码器和crtc
    if(!drm_conn->encoder_id){
        for(int i = 0;i < drm_conn->count_encoders;i++){
            drm_enc = drmModeGetEncoder(fd, drm_conn->encoders[i]);
            if(!drm_enc){
                
                continue;
            }
            if(drm_enc->crtc_id != 0){
                drm_disp.crtc_id = drm_enc->crtc_id;
                break;
            }
            for(int j = 0;j < drm_res->count_crtcs;j++){
                if(drm_enc->possible_crtcs & (1u << j)){
                    drm_disp.crtc_id = drm_res->crtcs[j];
                    break;
                }
            }
            if (drm_disp.crtc_id){
                break; 
            }
            drmModeFreeEncoder(drm_enc);
            drm_enc = NULL;
        }

    }else{
        drm_enc = drmModeGetEncoder(fd, drm_conn->encoder_id);
        if(!drm_enc){
            goto err_drm_enc;
        }
        if(drm_enc->crtc_id != 0){
            drm_disp.crtc_id = drm_enc->crtc_id;
        }else{
            for(int i = 0;i < drm_res->count_crtcs;i++){
                if(drm_enc->possible_crtcs & (1u << i)){
                    drm_disp.crtc_id = drm_res->crtcs[i];
                    break;
                }
            }
        }
    }
    //创建缓冲帧
    for(int i = 0;i < FPS_MAX;i++){
        ret = Drm_create_fb(fd,drm_disp.w,drm_disp.h,drm_disp.fb[i]);
        if(ret < 0){
            goto err;
        }
    }
    if(drmModeSetCrtc(fd, drm_disp.crtc_id, drm_disp.fb[0].fb_id, 0, 0, 
        &drm_disp.conn_id, 1, &drm_disp.modeinfo) < 0){
            goto err;
    }

    drmModeFreeEncoder(drm_enc);
    drmModeFreeConnector(drm_conn);
    drmModeFreeResources(drm_res);
    return fd;
err:
    drmModeFreeEncoder(drm_enc);
err_drm_enc:
    drmModeFreeConnector(drm_conn);
err_drm_conn:
    drmModeFreeResources(drm_res);
err_drm_res:
    return ret;
}

void Drm_display::Drm_close_display(struct drm_disp &drm_disp){
    for(int i = 0;i < FPS_MAX;i++){
        Drm_destory_fb(fd,drm_disp.fb[i]);
    }
    if(!fd){
        close(fd);
    }
}

int Drm_display::Drm_show(struct drm_disp &drm_disp,int index){
    int ret;
    // 始终 SetCrtc（简单可靠；RK VOP 在 fb 内容更新后 SetCrtc 会刷新）
    if((ret = drmModeSetCrtc(fd, drm_disp.crtc_id, drm_disp.fb[index].fb_id, 0, 0,
        &drm_disp.conn_id, 1, &drm_disp.modeinfo)) < 0){
            return ret;
    }
    return 0;
}

//导出 fb 的 dma-buf fd（供 RGA 直接写入）
int Drm_display::Drm_export_fb_fd(struct drm_fb &fb){
    struct drm_prime_handle args;
    memset(&args, 0, sizeof(args));
    args.handle = fb.handle;
    args.flags = DRM_CLOEXEC;
    if (drmIoctl(fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &args) < 0)
        return -1;
    return args.fd;
}
