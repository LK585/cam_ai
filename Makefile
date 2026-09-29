# ============================================================
# Makefile —— cam_ai 交叉编译（RK3576）
# 用法：
#   板端本地编译:  make CROSS= SYSROOT=
#   交叉编译:      make（用 SDK prebuilts 工具链）
# ============================================================

# 交叉编译器（RK3576 SDK prebuilts）
CROSS    ?= /home/lk/workspace/taishanpi_sdk/TaishanPi-3-Linux/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

CXX := $(CROSS)g++

CXXFLAGS ?= -O2 -Wall -Wextra -std=c++11
CXXFLAGS += -I.          # 包含 rknn_api.h / postprocess.h / 用户头文件
LDFLAGS  ?= -lrknnrt -ldrm -lpthread -lm

SRCS := main.cpp v4l2_cam.cpp display.cpp
OBJS := $(SRCS:.cpp=.o)
TARGET := cam_ai

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)
