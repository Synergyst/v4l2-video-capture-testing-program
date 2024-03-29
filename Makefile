# requirements:
#  64-bit:
#   apt-get install libsdl2-dev g++-10 gcc-10 make build-essentials libv4l-dev libv4l-0 v4l-utils v4l-conf
#  32-bit:
#   apt-get install libtbb2:armhf libtbb-dev:armhf libgl-dev:armhf libgl1-mesa-dev:armhf libsdl2-dev:armhf libsdl2-2.0-0:armhf libz3-dev:armhf libz3-4:armhf libfreetype-dev:armhf
#   apt-get install libfreetype6-dev:armhf libfreetype6:armhf g++-10-arm-linux-gnueabihf libv4l-dev:armhf libv4l-0:armhf libgomp1:armhf libdrm-dev:armhf libgbm-dev:armhf

CXX = g++
#CXX = arm-linux-gnueabihf-g++-10
#CXX = clang++

EXE = v4l2-capture-test
IMGUI_DIR = /root/projects/v4l2-capture-test/imgui.docking
SOURCES = v4l2-capture-test.cpp
SOURCES += $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_demo.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp
SOURCES += $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl2.cpp
OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
UNAME_S := $(shell uname -s)

CXXFLAGS = -std=gnu++20 -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends
#CXXFLAGS += -I/usr/include/opencv4/opencv2 -I/usr/include/opencv4
#CXXFLAGS += -I/usr/local/include/drm -I/opt/vc/include -I/opt/vc/include/interface/vchiq_arm -I/opt/vc/src/hello_pi/libs/ilclient -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux
#CXXFLAGS += -I/opt/vc/include/IL -I/opt/vc/include/vcinclude
#CXXFLAGS += -g -Wall -Wformat -O3 -march=armv8-a -mfpu=neon -ftree-vectorize -flax-vector-conversions -fopenmp
CXXFLAGS += -g -Wall -Wformat -O3 -mcpu=native -ftree-vectorize -flax-vector-conversions -fopenmp `sdl2-config --cflags`
#CXXFLAGS += -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2
#CXXFLAGS += -DOMX -DOMX_SKIP64BIT -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi
LIBS = `pkg-config --libs libv4l2 tbb opencv4`
LIBS += -lm -ldl -lopencv_core -lopencv_videoio -lopencv_highgui -lGL -ldl `sdl2-config --libs` -lpopt -lopencv_calib3d
#LIBS = -L/opt/vc/lib -lm -ldl `pkg-config --libs libv4l2 tbb opencv4` -lbrcmEGL -lbrcmGLESv2 -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lilclient -lpthread -lrt -lm
#LIBS += -L/opt/vc/src/hello_pi/libs/ilclient -L/opt/vc/src/hello_pi/libs/vgfont -L/opt/vc/src/hello_pi/libs/revision -L/usr/lib/arm-linux-gnueabihf

##---------------------------------------------------------------------
## BUILD FLAGS PER PLATFORM
##---------------------------------------------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += -lGL -ldl `sdl2-config --libs`

	CXXFLAGS += `sdl2-config --cflags`
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo `sdl2-config --libs`
	LIBS += -L/usr/local/lib -L/opt/local/lib

	CXXFLAGS += `sdl2-config --cflags`
	CXXFLAGS += -I/usr/local/include -I/opt/local/include
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(OS), Windows_NT)
	ECHO_MESSAGE = "MinGW"
	LIBS += -lgdi32 -lopengl32 -limm32 `pkg-config --static --libs sdl2`

	CXXFLAGS += `pkg-config --cflags sdl2`
	CFLAGS = $(CXXFLAGS)
endif

##---------------------------------------------------------------------
## BUILD RULES
##---------------------------------------------------------------------

%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:$(IMGUI_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:$(IMGUI_DIR)/backends/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXE)
	@echo Build complete for $(ECHO_MESSAGE)

$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)
#	$(CXX) -o $@ $^ $(CXXFLAGS) /opt/vc/src/hello_pi/libs/ilclient/libilclient.a $(LIBS)

clean:
	rm -f $(EXE) $(OBJS)
