# requirements:
#  64-bit:
#   apt-get install g++-12 gcc-12 make build-essentials libv4l-dev libv4l-0 v4l-utils v4l-conf
#  32-bit:
#   apt-get install libtbb2:armhf libtbb-dev:armhf libz3-dev:armhf libz3-4:armhf libfreetype-dev:armhf
#   apt-get install libfreetype6-dev:armhf libfreetype6:armhf g++-12-arm-linux-gnueabihf libv4l-dev:armhf libv4l-0:armhf

#CXX = g++-12
CXX = arm-linux-gnueabihf-g++-12
#CXX = clang++

EXE = v4l2-capture-test
SOURCES = v4l2-capture-test.cpp crt_filter.cpp png_loader.cpp
OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
UNAME_S := $(shell uname -s)

CXXFLAGS = -std=gnu++20
#CXXFLAGS += -g -Wall -Wformat -O3 -march=armv8-a -mfpu=neon -ftree-vectorize -flax-vector-conversions -fopenmp
#CXXFLAGS += -g -Wall -Wformat -ftree-vectorize -flax-vector-conversions -fopenmp -O1 -Wextra -fno-omit-frame-pointer -fsanitize=address
CXXFLAGS += -g -Wall -Wformat -ftree-vectorize -flax-vector-conversions -fopenmp -O1 -Wextra -fno-omit-frame-pointer
#CXXFLAGS += -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2
LIBS = `pkg-config --libs libv4l2 tbb32`
LIBS += -lm -ldl -lpopt -ljpeg -lpthread -latomic -lpng

##---------------------------------------------------------------------
## BUILD FLAGS PER PLATFORM
##---------------------------------------------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += -ldl

	CFLAGS = $(CXXFLAGS)
endif

##---------------------------------------------------------------------
## BUILD RULES
##---------------------------------------------------------------------

%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXE)
	@echo Build complete for $(ECHO_MESSAGE)

$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

clean:
	rm -f $(EXE) $(OBJS)

run:
	LD_PRELOAD=/usr/lib/arm-linux-gnueabihf/libasan.so.6 ASAN_OPTIONS=detect_leaks=1 ./$(EXE) --devices /dev/video1 --port 1337 --fps 60 --mjpeg --bgr --jpeg-quality 10 --encode-threads 3 --lazy --lazy-mod=1 --lazy-lookback=1
