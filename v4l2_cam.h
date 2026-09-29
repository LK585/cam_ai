#include <linux/videodev2.h>
#include <iostream>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

struct V4l2Frame {
    int index;
    unsigned nplanes;
    void *data[VIDEO_MAX_PLANES];
    size_t bytesused[VIDEO_MAX_PLANES];
};

class V4l2Camera{
public:

    V4l2Camera(std::string path);
    ~V4l2Camera();
    void v4l2_open();
    void v4l2_close();
    void v4l2_query_cap();
    void v4l2_set_fmt(int width = 2592, int height = 1944,enum v4l2_field = V4L2_FIELD_NONE, __u32 pixelformat = V4L2_PIX_FMT_NV12);
    void v4l2_get_fmt();
    void v4l2_enum_fmt();
    void v4l2_req_buf(int count = 8, enum v4l2_memory memory = V4L2_MEMORY_MMAP);
    void v4l2_mmap();
    void v4l2_unmap();
    void v4l2_full_queue(enum v4l2_memory memory = V4L2_MEMORY_MMAP);
    void v4l2_queue(enum v4l2_memory memory = V4L2_MEMORY_MMAP, int index = 0);
    V4l2Frame v4l2_dequeue(enum v4l2_memory memory = V4L2_MEMORY_MMAP);
    void v4l2_on_stream();
    void v4l2_off_stream();
    int v4l2_poll(int timeout_ms);           // 等待新帧：>0 有数据，0 超时，<0 出错
    int v4l2_export_fd(int index);           // 导出 dma-buf fd（物理连续，给 RGA 用），失败返回 -1
    void *v4l2_buf_data(int index);          // 返回第 index 个缓冲的虚拟地址（Y 平面）
private:

    std::string path_;
    int fd_;
    int type_;
    int num_planes_;
    int buf_cnt_;
    bool streaming_;
    size_t lengths_[VIDEO_MAX_FRAME][VIDEO_MAX_PLANES];
    void *buffers_[VIDEO_MAX_FRAME][VIDEO_MAX_PLANES];
    struct v4l2_capability cap_;
    struct v4l2_format fmt_;
    struct v4l2_fmtdesc fmtdes_;
    struct v4l2_frmsizeenum fsm_;
    struct v4l2_requestbuffers req_buf_;

};
