# Makefile for v4l2-stream-test with CUDA (vignette + CRT)
# Usage examples:
#   make                       # build release (default sm_86)
#   make GPU_ARCH=sm_75        # build release for a different GPU
#   make debug                 # debug build with -O0 -g and nvcc -G
#   make run ARGS="--upstream=192.168.168.175:1337 --listen=1337 --mjpeg --jpeg-quality=68 --no-lazy --encode-threads=1"
#   make clean

# -----------------------------------------------------------------------------
# Toolchains and config
# -----------------------------------------------------------------------------
CXX	?= g++
NVCC	?= nvcc
CUDA_PATH	?= /usr/local/cuda
GPU_ARCH	?= sm_86
STD	?= c++20
OPT	?= 3
ARGS	?= --upstream=192.168.168.175:1337 --listen=1337 --mjpeg --jpeg-quality=68 --no-lazy --encode-threads=1

# Include dirs (add your include paths if needed)
INCLUDES	:= -I.

# Lib dirs and libs
CUDA_LIB_DIR	:= $(CUDA_PATH)/lib64
LDLIBS	:= -lpthread -lpopt -ljpeg -lpng -lcudart
LDFLAGS	:= -L$(CUDA_LIB_DIR) -Wl,-rpath,$(CUDA_LIB_DIR)

# C++ and NVCC flags
CXXFLAGS_RELEASE	:= -O$(OPT) -std=$(STD) -DNDEBUG -fopenmp
NVCCFLAGS_RELEASE	:= -O$(OPT) -std=$(STD) -arch=$(GPU_ARCH)

CXXFLAGS_DEBUG	:= -O0 -g -std=$(STD) -fopenmp
NVCCFLAGS_DEBUG	:= -O0 -g -G -std=$(STD) -arch=$(GPU_ARCH)

# -----------------------------------------------------------------------------
# Sources / Targets
# -----------------------------------------------------------------------------
TARGET	:= v4l2-stream-test

# C++ sources
SRCS_CPP	:= v4l2-stream-test.cpp png_loader.cpp

# CUDA sources (match your filenames from build.sh)
SRCS_CU	:= vignette_cuda.cu crt_filter_cuda.cu

OBJS_CPP	:= $(SRCS_CPP:.cpp=.o)
OBJS_CU	:= $(SRCS_CU:.cu=.o)
OBJS	:= $(OBJS_CPP) $(OBJS_CU)

# Default build is release
CXXFLAGS	:= $(CXXFLAGS_RELEASE)
NVCCFLAGS	:= $(NVCCFLAGS_RELEASE)

# -----------------------------------------------------------------------------
# Rules
# -----------------------------------------------------------------------------
.PHONY: all release debug clean rebuild run print

all: release

release: $(TARGET)

debug: CXXFLAGS := $(CXXFLAGS_DEBUG)
debug: NVCCFLAGS := $(NVCCFLAGS_DEBUG)
debug: $(TARGET)

$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)"
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	@echo "CXX  $<"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.cu
	@echo "NVCC $<"
	$(NVCC) $(NVCCFLAGS) $(INCLUDES) -c $< -o $@

run: $(TARGET)
	./$(TARGET) $(ARGS)

print:
	@echo "CXX       = $(CXX)"
	@echo "NVCC      = $(NVCC)"
	@echo "CUDA_PATH = $(CUDA_PATH)"
	@echo "GPU_ARCH  = $(GPU_ARCH)"
	@echo "CXXFLAGS  = $(CXXFLAGS)"
	@echo "NVCCFLAGS = $(NVCCFLAGS)"
	@echo "LDFLAGS   = $(LDFLAGS)"
	@echo "LDLIBS    = $(LDLIBS)"

clean:
	@echo "Cleaning..."
	rm -f $(OBJS) $(TARGET)

rebuild: clean all
