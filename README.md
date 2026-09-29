# cam_ai —— RK3576 摄像头实时预览 + RKNN 目标检测

在 RK3576 开发板上用 **纯 C++（零 OpenCV）** 实现的实时目标检测管线：

> **MIPI 摄像头（OV5695）→ V4L2 采集 → 颜色转换 → YOLOv8（RKNN/NPU）推理 → MIPI 屏实时显示（含检测框）**，稳定 **30 FPS**。

## 项目亮点

| 亮点 | 说明 |
|---|---|
| 🎯 **完整链路自研** | 从 V4L2 采集、DRM/KMS 显示到 RKNN 推理全部自己实现，不依赖 OpenCV / GStreamer |
| ⚡ **30 FPS 实时** | 采集 / 推理 / 显示三线程流水线，显示侧规避了 RK3576 的平台性能陷阱 |
| 🧠 **NPU 部署** | YOLOv8 模型部署、九输出后处理（DFL 解码 + 网格还原 + NMS）全流程 |
| 🔧 **硬件加速探索** | 软件颜色转换（查表法优化）+ RGA 2D 硬件加速（dma-buf fd 模式）双实现 |
| 🐛 **底层问题定位** | 通过内核日志 + 源码分析 + 对照实验，定位并解决多个平台级疑难问题（见下） |

<!-- 建议在此处放一张屏幕实拍照片，对面试官非常有说服力：
![运行效果](docs/preview.jpg)
-->

## 硬件平台

| 项目 | 参数 |
|---|---|
| SoC | RK3576（4×Cortex-A72 + 4×Cortex-A53，内置 NPU） |
| 摄像头 | OV5695（5MP，MIPI CSI-2，2 lane × 840Mbps，I2C 地址 0x36） |
| 显示屏 | 5.5 寸 MIPI DSI 屏，1080×1920 @60Hz |
| 采集格式 | NV12（1280×720 @30fps），设备节点 `/dev/video-camera0` |
| 推理运行时 | RKNN 2.3.0（`librknnrt.so`，NPU 节点 `/dev/dri/renderD130`） |

## 架构

```
┌──────────────┐  共享最新帧   ┌───────────────┐  检测结果  ┌──────────────┐
│ 线程1 采集    │ ───────────→ │ 线程2 推理     │ ────────→ │ 线程3 显示    │
│ V4L2 DQBUF   │ mutex + cv   │ NV12→640 RGB  │  mutex    │ NV12→XRGB    │
│ → 拷贝共享    │              │ → rknn_run    │           │ + 画框 → DRM  │
└──────────────┘              └───────────────┘           └──────────────┘
     30 FPS                        NPU 速度                    30 FPS
```

**硬件数据通路**

```
相机：OV5695 → csi2-dphy → mipi-csi2 → rkcif → rkisp（ISP）→ /dev/video-camera0 (NV12)
显示：显存(dumb buffer) → VOP2(VP2) → DSI 主机 → DC-PHY → MIPI 屏
```

## 文件结构

```
cam_ai/
├── main.cpp              ⭐ 主程序：三线程 采集→RKNN推理→显示画框
├── cam_display3.cpp      ⭐ 摄像头预览（双线程，可独立运行，支持白平衡调节）
├── cam_display2.cpp         摄像头预览（单线程最小实现，便于理解主流程）
├── cam_display_rga.cpp      RGA 硬件加速实验（dma-buf fd 模式）
├── v4l2_cam.cpp/.h          V4L2 采集封装（NV12 / poll 超时 / dma-buf 导出）
├── display.cpp/.h           DRM/KMS 显示封装（dumb buffer + framebuffer）
├── rknn_infer.h             RKNN 推理封装（模型加载 / 输入输出 / 反量化）
├── postprocess.h            YOLOv8 九输出后处理（DFL 解码 + NMS）
├── rknn_api.h               Rockchip RKNN 官方头文件（第三方）
├── Makefile                 编译脚本
└── legacy/                  早期调试版本（V4L2 采集测试 / 三线程调试版）
```

## 编译

**板端本地编译（推荐，板上有完整依赖）：**

```bash
# RKNN 检测主程序
g++ -O2 main.cpp v4l2_cam.cpp display.cpp -o cam_ai \
    -I/usr/include/libdrm -ldrm -lpthread -lrknnrt -lm

# 摄像头预览程序
g++ -O2 cam_display3.cpp v4l2_cam.cpp display.cpp -o cam_display3 \
    -I/usr/include/libdrm -ldrm -lpthread
```

**交叉编译：** `make`（需在 Makefile 中配置 SDK 工具链路径）

## 运行

```bash
# 实时预览（R/B 增益用于手动白平衡，可选参数）
./cam_display3 1.7 1.6

# RKNN 目标检测（摄像头对准目标物体，终端打印检测到的类别与置信度）
./cam_ai
```

**模型准备**（模型文件较大，未纳入仓库）：

```bash
# 使用 rknn_model_zoo 的官方 YOLOv8 模型（9 输出优化结构）
cp <rknn_model_zoo>/examples/yolov8/model/yolov8.rknn            /userdata/cam_ai/
cp <rknn_model_zoo>/examples/yolov8/model/coco_80_labels_list.txt /userdata/cam_ai/
```

## 技术难点与解决

| 问题 | 现象 | 原因与解决 |
|---|---|---|
| **显示卡死** | 每帧刷新屏幕，约 8 帧后程序卡住 | RK3576 的 VOP2+DSI 每次 `SetCrtc` 都要重新协商 DSI 链路（约 0.8s）。**改为只在初始化时 SetCrtc 一次**，之后仅写显存，由 VOP 持续扫描刷新 → 恢复 30fps |
| **转换性能** | 逐像素颜色转换只有约 1fps | 每像素 2 次整数除法（缩放）成为瓶颈。**预计算行列缩放表**，内层循环零除法 |
| **检测满屏小框** | 画面被几十个正方形框覆盖 | 优化版 YOLOv8 的 `cls` 输出**已融合 sigmoid**，代码又套一次 → 背景 0 值变 0.5 超过阈值。**对齐官方 postprocess 逻辑**（不重复激活） |
| **检测不到目标** | 推理无输出 | 模型 ONNX 图内已内置 `/255` 归一化，软件又除一次导致**双重归一化**。改为喂 **raw uint8 NHWC** 输入 |
| **RGA 输出全蓝** | 硬件加速画面纯蓝 | 板上 RGA **无 IOMMU**，虚拟地址模式读到不连续物理内存。改用 **dma-buf fd 模式**（相机缓冲 `VIDIOC_EXPBUF`、显示缓冲 `DRM_PRIME_HANDLE_TO_FD`） |
| **流偶发中断** | 相机跑几帧后停住 | 进程被强杀未做 `STREAMOFF`，污染 ISP 状态。**处理 SIGTERM/SIGINT 干净关流**，并加 poll 超时看门狗自动重启 |
| **多平面判断错误** | 采集参数设置失败 | `VIDIOC_QUERYCAP` 必须用 `device_caps` 判断单/多平面，RK3576 的 `capabilities` 同时含两种 bit |

## 关键实现要点

- **显示刷新**：`SetCrtc` 仅在初始化调用一次 —— VOP 持续扫描显存，改内容即刷新（规避平台级重协商开销）
- **采集健壮性**：`poll()` 带超时取帧 + 流卡死自动重启 + 信号触发的干净关流
- **颜色转换**：BT.601 有限范围 YUV→RGB；手动白平衡用定点数增益（`gain×256 >> 16`）
- **推理输入**：640×640 RGB，`RKNN_TENSOR_UINT8` + `NHWC`（模型内已归一化）
- **后处理**：三个尺度（stride 8/16/32）→ DFL 16-bin softmax 解码 → 网格坐标还原 → 置信度过滤 → 按类 NMS

## 已知限制

- **3A（自动曝光/白平衡）未启用**：板厂未提供该传感器对应 ISP 架构的 IQ 校准文件，官方 rkaiq 服务无法初始化；当前使用手动白平衡增益 + 固定曝光
- **RGA 硬件加速为实验特性**：dma-buf fd 模式通路已验证可行，色度通道偏移问题待进一步定位
- 模型为 COCO 80 类通用检测模型，非专用数据集训练

## 许可

代码仅供学习与交流。`rknn_api.h` 版权归 Rockchip 所有。
