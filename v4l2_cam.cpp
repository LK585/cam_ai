#include "v4l2_cam.h"
#include <poll.h>


V4l2Camera::V4l2Camera(std::string path):
    path_(path),fd_(-1),type_(0),num_planes_(0),buf_cnt_(0),streaming_(false)
{
    memset(buffers_, 0, sizeof(buffers_));
    memset(lengths_, 0, sizeof(lengths_));
    memset(&cap_, 0, sizeof(cap_));
    memset(&fmt_, 0, sizeof(fmt_));
}
V4l2Camera::~V4l2Camera(){
    v4l2_off_stream();
    v4l2_unmap();
    v4l2_close();
}
//打开文件描述符
void V4l2Camera::v4l2_open(){
    fd_ = open(path_.c_str(), O_RDWR);
    if(fd_ < 0 ){
        throw std::runtime_error("open failed error");
    }
}
void V4l2Camera::v4l2_close(){
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}
//查能力
void V4l2Camera::v4l2_query_cap(){
    //通过VIDIOC_QUERYCAP查看能力
    if(ioctl(fd_, VIDIOC_QUERYCAP, &cap_) < 0){
        throw std::runtime_error("ioctl VIDIOC_QUERYCAP error");
    }
    // 正确：优先用 device_caps（该 video 节点实际使用的能力）
    // capabilities 可能同时含单平面+多平面位，不能直接用来判断
    __u32 caps = cap_.capabilities;
    if (caps & V4L2_CAP_DEVICE_CAPS)
        caps = cap_.device_caps;

    if(caps & V4L2_CAP_VIDEO_CAPTURE){
        type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        std::cout << "type : V4L2_BUF_TYPE_VIDEO_CAPTURE" << std::endl;
    }else if(caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE){
        type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        std::cout << "type : V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE" << std::endl;
    }else{
        throw std::runtime_error("no capture capability");
    }
}

//设置像素格式
void V4l2Camera::v4l2_set_fmt(int width, int height, enum v4l2_field field, __u32 pixelformat){
    //优先设置type，type没设置或设置不对会报错
    fmt_.type = type_;
    if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
        fmt_.fmt.pix_mp.width = width;
        fmt_.fmt.pix_mp.height = height;
        fmt_.fmt.pix_mp.field = field;
        fmt_.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        //通过VIDIOC_S_FMT设置像素格式
        if(ioctl(fd_, VIDIOC_S_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_S_FMT error");
        //获取像素格式格式
        if(ioctl(fd_, VIDIOC_G_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_G_FMT error");
        //保存多少平面
        num_planes_ = fmt_.fmt.pix_mp.num_planes;
    }else if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE){
        fmt_.fmt.pix.width = width;
        fmt_.fmt.pix.height = height;
        fmt_.fmt.pix.field = field;
        fmt_.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        //通过VIDIOC_S_FMT设置像素格式
        if(ioctl(fd_, VIDIOC_S_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_S_FMT error");
        //获取像素格式格式
        if(ioctl(fd_, VIDIOC_G_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_G_FMT error");
        
        num_planes_ = 1;
    }
    
}
//获取像素格式
void V4l2Camera::v4l2_get_fmt(){
    fmt_.type = type_;
    
    if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){

        if(ioctl(fd_, VIDIOC_G_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_G_FMT error");
        std::cout << "type:\t" << fmt_.type << std::endl;
        std::cout << "pix:\t" << fmt_.fmt.pix_mp.width << " x " << fmt_.fmt.pix_mp.height << std::endl;
        std::cout << "field:\t" << fmt_.fmt.pix_mp.field << std::endl;
        std::cout << "num_planes_:\t" << fmt_.fmt.pix_mp.num_planes << std::endl;

    }else if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE){
        if(ioctl(fd_, VIDIOC_G_FMT,&fmt_) < 0)
            throw std::runtime_error("ioctl VIDIOC_G_FMT error");
        std::cout << "type:\t" << fmt_.type << std::endl;
        std::cout << "pix:\t" << fmt_.fmt.pix.width << " x " << fmt_.fmt.pix.height << std::endl;
        std::cout << "field:\t" << fmt_.fmt.pix.field << std::endl;
    }
}

void V4l2Camera::v4l2_enum_fmt(){
    for(int i = 0;;i++){
        fmtdes_.index = i;
        fmtdes_.type = type_;
        //通过VIDIOC_ENUM_FMT获取像素枚举
        if(ioctl(fd_, VIDIOC_ENUM_FMT,&fmtdes_) < 0){
            break;
        }
        std::cout << "[" << fmtdes_.index << "]" << "pixelformat: " << fmtdes_.pixelformat << "description: " << fmtdes_.description << std::endl;
        for(int j = 0;;j++){
            memset(&fsm_,0,sizeof(fsm_));
            fsm_.index = j;
            fsm_.pixel_format = fmtdes_.pixelformat;
            //通过VIDIOC_ENUM_FRAMESIZES 获取 每个像素格式下支持的大小，也就是分辨率
            //不断枚举，直到获取失败
            if(ioctl(fd_, VIDIOC_ENUM_FRAMESIZES,&fsm_) < 0){
                break;
            }
            std::cout << "\t" << "[" << fsm_.index << "]" << fsm_.discrete.width << " x " << fsm_.discrete.height << std::endl;
        }
    }
}
//请求buf
void V4l2Camera::v4l2_req_buf(int count, enum v4l2_memory memory){
    //req_buf_清零
    memset(&req_buf_, 0, sizeof(req_buf_));
    //设置申请缓冲区的数量
    req_buf_.count = count;
    //设置内存格式
    req_buf_.memory = memory;
    //设置摄像头类型
    req_buf_.type = type_;
    //通过VIDIOC_REQBUFS申请缓冲区
    if(ioctl(fd_, VIDIOC_REQBUFS, &req_buf_) < 0){
        throw std::runtime_error("ioctl VIDIOC_REQBUFS error");
    }
    //保存申请的缓冲区数量
    buf_cnt_ = req_buf_.count;
}
//检查和映射buf
void V4l2Camera::v4l2_mmap(){
    //映射缓冲区
    for(int i = 0;i < buf_cnt_ ;i++){
        //定义缓冲区结构体变量
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        //平面结构体，多平面类型需要定义
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&planes, 0, sizeof(planes));
        if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
            buf.m.planes = planes;
            buf.length = num_planes_;
        }

        buf.index = i;
        buf.type = type_;
        buf.memory = V4L2_MEMORY_MMAP;
        //获取缓冲区
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
        {
            std::cout << "index: " << i << std::endl;
            throw std::runtime_error("ioctl VIDIOC_QUERYBUF error");
        }
        //映射缓冲区，多平面的数据分开存储，所以需要二维指针数组进行存储，然后进行映射
        for (int j = 0; j < num_planes_; j++)
        {
            if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                buffers_[i][j] = mmap(NULL, buf.m.planes[j].length, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd_, buf.m.planes[j].m.mem_offset);
                lengths_[i][j] = buf.m.planes[j].length;
            }else if(type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE){
                buffers_[i][j] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd_, buf.m.offset);
                lengths_[i][j] = buf.length;
            }
            if (buffers_[i][j] == MAP_FAILED) 
            {
                throw std::runtime_error("mmap error");
            }
        }

    }
}
//取消映射
void V4l2Camera::v4l2_unmap(){
    for (unsigned i = 0; i < buf_cnt_; i++) {
        unsigned np = (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) 
                        ? num_planes_ 
                        : 1;
        for (unsigned j = 0; j < np; j++) {
            if (buffers_[i][j] && buffers_[i][j] != MAP_FAILED)
                munmap(buffers_[i][j], lengths_[i][j]);
            buffers_[i][j] = NULL;
            lengths_[i][j] = 0;
        }
    }
}
void V4l2Camera::v4l2_full_queue(enum v4l2_memory memory){
    for(int i = 0;i < buf_cnt_;i++){
        v4l2_queue(memory, i);
    }
}
//入队
void V4l2Camera::v4l2_queue(enum v4l2_memory memory,int index){
    
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    memset(&planes, 0, sizeof(planes));

    buf.index = index;
    buf.type = type_;
    buf.memory = memory;
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = num_planes_;
    }

    if (0 != ioctl(fd_, VIDIOC_QBUF, &buf))
    {
        throw std::runtime_error("ioctl VIDIOC_QBUF error");
    }
    
}
//出队
V4l2Frame V4l2Camera::v4l2_dequeue(enum v4l2_memory memory){
    //定义缓冲区
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&planes, 0, sizeof(planes));
    // buf.index = index;
    buf.type = type_;
    buf.memory = memory;
    if (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = num_planes_;
    }

    if (0 != ioctl(fd_, VIDIOC_DQBUF, &buf))
    {
        throw std::runtime_error("ioctl VIDIOC_DQBUF error");
    }
    V4l2Frame f{};
    f.index = (int)buf.index;
    f.nplanes = (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) 
                ? num_planes_ 
                : 1;
    for (unsigned j = 0; j < f.nplanes; j++) {
        f.data[j] = buffers_[buf.index][j];
        f.bytesused[j] = (type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                             ? planes[j].bytesused
                             : buf.bytesused;
    }
    return f;
}

//等待新帧（带超时），返回 poll() 结果：>0 有数据，0 超时，<0 出错
int V4l2Camera::v4l2_poll(int timeout_ms){
    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    return poll(&pfd, 1, timeout_ms);
}

//导出 dma-buf fd（物理连续内存，供 RGA 等硬件直接使用）
int V4l2Camera::v4l2_export_fd(int index){
    struct v4l2_exportbuffer exp;
    memset(&exp, 0, sizeof(exp));
    exp.type = type_;
    exp.index = index;
    exp.flags = O_CLOEXEC;
    if (ioctl(fd_, VIDIOC_EXPBUF, &exp) < 0)
        return -1;
    return exp.fd;
}

//返回第 index 个缓冲的虚拟地址（Y 平面起始）
void *V4l2Camera::v4l2_buf_data(int index){
    return buffers_[index][0];
}

//开流
void V4l2Camera::v4l2_on_stream(){
    if (0 != ioctl(fd_, VIDIOC_STREAMON, &type_))
    {
        throw std::runtime_error("ioctl VIDIOC_STREAMON error");
    }
    streaming_ = true;
}
//关流
void V4l2Camera::v4l2_off_stream(){
    if(streaming_){
        if (0 != ioctl(fd_, VIDIOC_STREAMOFF, &type_))
        {
            throw std::runtime_error("ioctl VIDIOC_STREAMOFF error");
        }
        streaming_ = false;
    }
}

