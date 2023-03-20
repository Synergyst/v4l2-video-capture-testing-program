/*
 *  V4L2 video capture test program
 *
 *  This is not meant to be representative of a production-ready program, it may not work without heavy modification.
 *  Lots of variables, functions, etc may be named incorrectly, have issues, etc.
 *  This file gets updated frequently with test code; please understand that a lot of parts of it may not make any sense time to time/at all. :)
 *  Understanding that this is a public testing branch; feel free to modify/reuse/distribute this code in any way without restrictions!
 *
 *  Example command-line usage:
 *    Usage: ./build.sh
 *    This will attempt to build the source code
 * 
 *    Usage: ./v4l2-capture-test
 *    This will return the most recent usage information for the utility
 */
#include <iostream>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> // getopt_long()
#include <fcntl.h>  // low-level i/o
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <omp.h>
#include <span>
#include <errno.h>
#include <future>
#include <linux/fb.h>
#include <execution>
#include <numeric>
#include <arm_neon.h> // For SIMD instructions
#include "imgui.docking/imgui.h"
#include "imgui.docking/backends/imgui_impl_sdl2.h"
#include "imgui.docking/backends/imgui_impl_opengl2.h"
#include <stdio.h>
#include <SDL.h>
#include <SDL_opengl.h>
#include <bcm_host.h>

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define IS_RGB_DEVICE false // change in case capture device is really RGB24 and not BGR24
int fbfd = -1, ret = 1, retSize = 1, frame_number = 0, byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, alpha_channel_amount = 244, allDevicesTargetFramerate = 30, numPixels = defaultWidth * defaultHeight;
const int def_circle_center_x = defaultWidth / 2, def_circle_center_y = defaultHeight / 2, def_circle_diameter = 5, def_circle_thickness = 1, def_circle_red = 0, def_circle_green = 0, def_circle_blue = 0;
int circle_center_x = def_circle_center_x, circle_center_y = def_circle_center_y, circle_diameter = def_circle_diameter, circle_thickness = def_circle_thickness, circle_red = def_circle_red, circle_green = def_circle_green, circle_blue = def_circle_blue;
unsigned char* outputWithAlpha = new unsigned char[defaultWidth * defaultHeight * 4];
unsigned char* prevOutputFrame = new unsigned char[defaultWidth * defaultHeight * byteScaler];
const int num_threads = std::thread::hardware_concurrency() - 1;
bool isDualInput = false, done = false;
std::atomic<bool> shouldLoop;
std::future<int> background_task_cap_main;
std::future<int> background_task_cap_alt;
std::vector<std::future<void>> futures(num_threads);
std::vector<std::string> devNames;
struct buffer {
  void* start;
  size_t length;
};
struct devInfo {
  int frame_number,
    framerate,
    framerateDivisor,
    startingWidth,
    startingHeight,
    startingSize,
    force_format,
    targetFramerate,
    fd;
  unsigned int n_buffers;
  double frameDelayMicros,
    frameDelayMillis;
  bool isTC358743 = true;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  int index;
  unsigned char *outputFrame;
  char* device;
};
struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;
struct fb_var_screeninfo vinfo;
struct fb_fix_screeninfo finfo;
long int screensize;
size_t stride;
char* fbmem;

/*class ImageDisplayer {
public:
  ImageDisplayer(int width, int height) : width_(width), height_(height) {
    std::cout << "BCM host init.." << std::endl;
    bcm_host_init();
    std::cout << "BCM host init complete." << std::endl;

    std::cout << "Dispmanx opening.." << std::endl;
    display_handle_ = vc_dispmanx_display_open(0);
    if (display_handle_ == 0) {
      std::cerr << "Failed to open Dispmanx display." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    std::cout << "Dispmanx opened." << std::endl << display_handle_;

    std::cout << "Dispmanx resource creating.." << std::endl;
    //resource_handle_ = vc_dispmanx_resource_create(VC_IMAGE_RGB888, width_, height_, nullptr);
    resource_handle_ = vc_dispmanx_resource_create(VC_IMAGE_RGB565, width_, height_, nullptr);
    if (resource_handle_ == 0) {
      std::cerr << "Failed to create Dispmanx resource." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    std::cout << "Dispmanx resource created." << std::endl;

    element_handle_ = 0;
  }

  ~ImageDisplayer() {
    if (element_handle_ != 0) {
      stop_displaying();
    }

    vc_dispmanx_resource_delete(resource_handle_);

    vc_dispmanx_display_close(display_handle_);

    bcm_host_deinit();
  }

  void display_rgb24_image(const unsigned char* image) {
    DISPMANX_UPDATE_HANDLE_T update_handle = vc_dispmanx_update_start(0);

    VC_RECT_T dst_rect;
    vc_dispmanx_rect_set(&dst_rect, 0, 0, width_, height_);

    VC_RECT_T src_rect;
    vc_dispmanx_rect_set(&src_rect, 0, 0, width_ << 16, height_ << 16);

    vc_dispmanx_resource_write_data(resource_handle_, VC_IMAGE_RGB888, width_ * 3, const_cast<unsigned char*>(image), &src_rect);

    if (element_handle_ == 0) {
      element_handle_ = vc_dispmanx_element_add(
        update_handle, display_handle_, 0, &dst_rect, resource_handle_, &src_rect, DISPMANX_PROTECTION_NONE, nullptr, nullptr, DISPMANX_NO_ROTATE);
    }

    vc_dispmanx_update_submit_sync(update_handle);
  }

  void stop_displaying() {
    DISPMANX_UPDATE_HANDLE_T update_handle = vc_dispmanx_update_start(0);
    vc_dispmanx_element_remove(update_handle, element_handle_);
    vc_dispmanx_update_submit_sync(update_handle);
    element_handle_ = 0;
  }

private:
  int width_;
  int height_;
  DISPMANX_DISPLAY_HANDLE_T display_handle_;
  DISPMANX_RESOURCE_HANDLE_T resource_handle_;
  DISPMANX_ELEMENT_HANDLE_T element_handle_;
};*/
void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}
int xioctl(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}
void invert_greyscale(unsigned char*& input, unsigned char*& output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd num_threads(num_threads)
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}
int init_dev_stage1(struct buffer*& buffers, struct devInfo*& devInfos) {
  fprintf(stderr, "\n[cap%d] Starting V4L2 capture testing program with the following V4L2 device: %s\n", devInfos->index, devInfos->device);
  struct stat st;
  if (-1 == stat(devInfos->device, &st)) {
    fprintf(stderr, "[cap%d] Cannot identify '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "[cap%d] %s is no device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  devInfos->fd = open(devInfos->device, O_RDWR | O_NONBLOCK, 0);
  if (-1 == devInfos->fd) {
    fprintf(stderr, "[cap%d] Cannot open '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "[cap%d] Opened V4L2 device: %s\n", devInfos->index, devInfos->device);
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s is no V4L2 device\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "[cap%d] %s is no video capture device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  // Select video input, video standard and tune here.
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(devInfos->fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        // Cropping not supported.
        break;
      default:
        // Errors ignored.
        break;
      }
    }
  } else {
    // Errors ignored.
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "[cap%d] Forcing format for %s to: %d\n", devInfos->index, devInfos->device, devInfos->force_format);
  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 2) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 1) {
      //byteScaler = 2;
      //fmt.fmt.pix.width = devInfos->startingWidth;
      //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  } else {
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctl(devInfos->fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }
  // Buggy driver paranoia.
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  //struct v4l2_requestbuffers req;
  CLEAR(devInfos->req);
  devInfos->req.count = 4;
  devInfos->req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  devInfos->req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_REQBUFS, &devInfos->req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s does not support memory mapping\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (devInfos->req.count < 2) {
    fprintf(stderr, "[cap%d] Insufficient buffer memory on %s\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  return 0;
}
int init_dev_stage2(struct buffer*& buffers, struct devInfo*& devInfos) {
  if (!buffers) {
    fprintf(stderr, "[cap%d] Out of memory\n", devInfos->index);
    exit(EXIT_FAILURE);
  }
  for (devInfos->n_buffers = 0; devInfos->n_buffers < devInfos->req.count; ++devInfos->n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = devInfos->n_buffers;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    buffers[devInfos->n_buffers].length = buf.length;
    buffers[devInfos->n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, devInfos->fd, buf.m.offset);
    if (MAP_FAILED == buffers[devInfos->n_buffers].start)
      errno_exit("mmap");
  }
  // TODO: Implement a more proper way to handle settings in this area regarding if we really need DV timings to be set
  if (devInfos->isTC358743) {
    struct v4l2_dv_timings timings;
    v4l2_std_id std;
    int ret;
    memset(&timings, 0, sizeof timings);
    ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
    if (ret >= 0) {
      fprintf(stderr, "[cap%d] QUERY_DV_TIMINGS for %s: %ux%u pixclk %llu\n", devInfos->index, devInfos->device, timings.bt.width, timings.bt.height, timings.bt.pixelclock);
      devInfos->startingWidth = timings.bt.width;
      devInfos->startingHeight = timings.bt.height;
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;
      // Can read DV timings, so set them.
      ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
      if (ret < 0) {
        fprintf(stderr, "[cap%d] Failed to set DV timings\n", devInfos->index);
        return 1;
      } else {
        double tot_height, tot_width;
        const struct v4l2_bt_timings* bt = &timings.bt;
        tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
        tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
        devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));
        devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
        devInfos->frameDelayMicros = (1000000 / devInfos->framerate);
        devInfos->frameDelayMillis = (1000 / devInfos->framerate);
        int rawInputThroughput = (float)((float)(devInfos->framerate * devInfos->startingSize) / 125000.0F); // Measured in megabits/sec based on input framerate
        int rawOutputThroughput = (float)((((float)devInfos->framerate / devInfos->framerateDivisor) * devInfos->startingSize) / 125000.0F); // Measured in megabits/sec based on output framerate
        fprintf(stderr, "[cap%d] device_name: %s, startingWidth: %d, startingHeight: %d, startingSize: %d, framerate(actual): %u, framerateDivisor: %d, targetFramerate: %d, frameDelayMicros: %f, frameDelayMillis: %f\n",
          devInfos->index, devInfos->device, devInfos->startingWidth, devInfos->startingHeight, devInfos->startingSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, devInfos->frameDelayMicros, devInfos->frameDelayMillis);
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, rawInputThroughput: ~%dMb/sec, rawOutputThroughput: ~%dMb/sec\n", devInfos->index, devInfos->device, devInfos->isTC358743, rawInputThroughput, rawOutputThroughput);
      }
    } else {
      memset(&std, 0, sizeof std);
      ret = ioctl(devInfos->fd, VIDIOC_QUERYSTD, &std);
      if (ret >= 0) {
        // Can read standard, so set it.
        ret = xioctl(devInfos->fd, VIDIOC_S_STD, &std);
        if (ret < 0) {
          fprintf(stderr, "[cap%d] Failed to set standard\n", devInfos->index);
          return 1;
        } else {
          // SD video - assume 50Hz / 25fps
          devInfos->framerate = 25;
        }
      }
    }
  } else {
    fprintf(stderr, "[cap%d] Fatal: Only the TC358743 is supported for now. Support for general camera inputs (such as: %s) will need to be added in the future..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
  unsigned int i;
  for (i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
  }
  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type))
    errno_exit("VIDIOC_STREAMON");
  fprintf(stderr, "[cap%d] Initialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  return 0;
}
int get_frame(struct buffer* buffers, struct devInfo* devInfos) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period to wait for device to respond
  tv.tv_sec = 0;
  tv.tv_usec = devInfos->frameDelayMicros;
  r = select(devInfos->fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) {
    if (EINTR == errno)
      // Do nothing
      //continue;
    errno_exit("select");
  }
  if (0 == r) {
    fprintf(stderr, "[cap%d] select timeout\n", devInfos->index);
    exit(EXIT_FAILURE);
  }
  struct v4l2_buffer buf;
  unsigned int i;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
    case EAGAIN:
      fprintf(stderr, "[cap%d] EAGAIN\n", devInfos->index);
      return 0;
    case EIO:
      // Could ignore EIO, see spec.
      // fall through
    default:
      errno_exit("VIDIOC_DQBUF");
    }
  }
  assert(buf.index < devInfos->n_buffers);
  if (devInfos->index == 1) {
    unsigned char* preP = (unsigned char*)buffers[buf.index].start;
    std::vector<int> indices(numPixels);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i) {
      int idx = i * 8; // Processing 8 pixels at a time
      if (idx + 7 < numPixels) {
        uint8x8x3_t rgb = vld3_u8(&preP[idx * 3]);
        uint8x8x4_t rgba;
        rgba.val[0] = rgb.val[0];
        rgba.val[1] = rgb.val[1];
        rgba.val[2] = rgb.val[2];
        rgba.val[3] = vdup_n_u8(alpha_channel_amount);
        vst4_u8(&outputWithAlpha[idx * 4], rgba);
      } else {
        // Process the remaining pixels
        for (; idx < numPixels; ++idx) {
          outputWithAlpha[idx * 4] = preP[idx * 3];
          outputWithAlpha[idx * 4 + 1] = preP[idx * 3 + 1];
          outputWithAlpha[idx * 4 + 2] = preP[idx * 3 + 2];
          outputWithAlpha[idx * 4 + 3] = alpha_channel_amount;
        }
      }
    });
  } else {
    std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, buffers[buf.index].length); // copy frame data to frame buffer
  }
  //if (devInfos->frame_number % devInfos->framerateDivisor == 0) devInfos->frame_number++;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
    errno_exit("VIDIOC_QBUF");
  // EAGAIN - continue select loop
  CLEAR(i);
  CLEAR(buf);
  CLEAR(r);
  CLEAR(tv);
  CLEAR(fds);
  return 0;
}
int deinit_bufs(struct buffer*& buffers, struct devInfo*& devInfos) {
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
      errno_exit("munmap");
  free(buffers);
  fprintf(stderr, "[cap%d] Uninitialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  if (-1 == close(devInfos->fd))
    errno_exit("close");
  devInfos->fd = -1;
  fprintf(stderr, "[cap%d] Closed V4L2 device: %s\n", devInfos->index, devInfos->device);
  fprintf(stderr, "\n");
  return 0;
}
void did_memory_allocate_correctly(struct devInfo*& devInfos) {
  if (devInfos->outputFrame == NULL) {
    fprintf(stderr, "[cap%d] Fatal: Memory allocation failed of output frame for device: %s..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
}
int init_vars(struct devInfo*& devInfos, struct buffer*& bufs, const int force_format, const int targetFramerate, const bool isTC358743, const bool isThermalCamera, char*& dev_name, int index) {
  devInfos = (devInfo*)calloc(1, sizeof(*devInfos));
  devInfos->device = (char*)calloc(sizeof(dev_name)+1, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->frame_number = 0,
    devInfos->framerate = 30,
    devInfos->framerateDivisor = 1,
    devInfos->startingWidth = defaultWidth,
    devInfos->startingHeight = defaultHeight,
    devInfos->startingSize = (devInfos->startingWidth * devInfos->startingHeight * byteScaler),
    devInfos->force_format = force_format,
    devInfos->targetFramerate = targetFramerate,
    devInfos->fd = -1,
    devInfos->isTC358743 = isTC358743,
    devInfos->index = index;
  init_dev_stage1(buffersMain, devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * byteScaler), sizeof(unsigned char)); // allocate memory for frame buffer
  did_memory_allocate_correctly(devInfos);
  retSize = (devInfos->startingWidth * devInfos->startingHeight * byteScaler * 2);
  return 0;
}
void commandline_usage(const int argcnt, char** args) {
  switch (argcnt) {
  case 3:
    devNames.push_back(args[1]);
    devNames.push_back(args[2]);
    isDualInput = false;
    break;
  case 4:
    devNames.push_back(args[1]);
    devNames.push_back(args[2]);
    devNames.push_back(args[3]);
    isDualInput = true;
    break;
  default:
    fprintf(stderr, "[main] Usage:\n\t%s <V4L2 main device> <V4L2 alt device> </dev/fb out device>\n\t%s <V4L2 main device> </dev/fb out device>\nExample:\n\t%s /dev/video0 /dev/video1 /dev/fb0\n\t%s /dev/video0 /dev/fb0\n", args[0], args[0], args[0], args[0]);
    exit(1);
    break;
  }
}
void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
  deinit_bufs(buffersAlt, devInfoAlt);
  delete[] outputWithAlpha;
  munmap(fbmem, screensize);
  close(fbfd);
}
void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt, int argCnt, char **args) {
  commandline_usage(argCnt, args);
  fprintf(stderr, "[main] Initializing..\n");
  // allocate memory for structs
  init_vars(deviMain, bufMain, 3, allDevicesTargetFramerate, true, true, args[1], 0);
  if (isDualInput)
    init_vars(deviAlt, bufAlt, 3, allDevicesTargetFramerate, true, true, args[2], 1);
  shouldLoop.store(true);
  numPixels = devInfoMain->startingWidth * devInfoMain->startingHeight;
  fbfd = open(devNames.at(isDualInput ? 2 : 1).c_str(), O_RDWR);
  if (fbfd == -1) {
    perror("Error: cannot open framebuffer device");
    exit(1);
  }
  ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
  if (vinfo.bits_per_pixel != 16 || vinfo.xres != (unsigned int)devInfoMain->startingWidth || vinfo.yres != (unsigned int)devInfoMain->startingHeight) {
    fprintf(stderr, "Error: framebuffer does not accept RGB24 frames with %dx%d resolution\n", devInfoMain->startingWidth, devInfoMain->startingHeight);
    exit(1);
  }
  //screensize = vinfo.xres * vinfo.yres * 4; // size for 32 BPP
  fprintf(stderr, "Actual frame buffer configuration: BPP: %u, xres: %u, yres: %u\n", vinfo.bits_per_pixel, vinfo.xres, vinfo.yres);
  screensize = vinfo.xres * vinfo.yres * 2; // original size for RGB565LE
  // Get the fixed frame buffer information
  if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
    perror("Error getting fixed frame buffer information");
    exit(EXIT_FAILURE);
  }
  // Calculate the stride
  stride = finfo.line_length / (vinfo.bits_per_pixel / 8);
  fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fbmem == MAP_FAILED) {
    perror("Error: failed to mmap framebuffer device to memory");
    exit(1);
  }
}
void draw_hollow_circle(unsigned char* image, int width, int height, int x, int y, int diameter, int thickness, unsigned char red, unsigned char green, unsigned char blue) {
  // Calculate the radius of the circle
  int radius = diameter / 2;
  // Calculate the coordinates of the top-left corner of the bounding box
  int x_min = x - radius;
  int y_min = y - radius;
  // Iterate over every pixel in the bounding box
#pragma omp parallel for simd num_threads(num_threads)
  for (int i = x_min; i < x_min + diameter; i++) {
    for (int j = y_min; j < y_min + diameter; j++) {
      // Calculate the distance between this pixel and the center of the circle
      int dx = i - x;
      int dy = j - y;
      double distance = std::sqrt(dx * dx + dy * dy);
      // Check if the pixel is within the thickness of the circle
      if (distance >= radius - thickness && distance <= radius) {
        // Calculate the index of the first byte of this pixel in the image data array
        int index = 3 * (j * width + i);
        // Set the color of the pixel to the desired RGB value
        image[index] = red;
        image[index + 1] = green;
        image[index + 2] = blue;
      }
    }
  }
}
int startImGuiThread() {
  // Setup SDL
  SDL_Window* window;
  ImGuiIO io;
  SDL_WindowFlags window_flags;
  SDL_GLContext gl_context;
  ImGuiStyle style;
  ImVec4 clear_color;
  {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
      printf("Error: %s\n", SDL_GetError());
      return -1;
    }
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif
    // Setup window
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    window = SDL_CreateWindow("Configuration", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 460, 460, window_flags);
    gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows
    //io.ConfigViewportsNoAutoMerge = true;
    //io.ConfigViewportsNoTaskBarIcon = true;
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      style.WindowRounding = 5.0f;
      style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL2_Init();
    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);
    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  }
  // Main loop
  while (!done) {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        done = true;
      if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
        done = true;
    }
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    //
    ImGui::Begin("configuration / test object(s)");
    if (devNames.size() > 2)
      ImGui::Checkbox("dual-in", &isDualInput);
    ImGui::SliderInt("alpha_channel_amount", &alpha_channel_amount, 0, 255);
    ImGui::SliderInt("diameter", &circle_diameter, 1, defaultHeight);
    ImGui::SliderInt("circle_thickness", &circle_thickness, 1, circle_diameter / 2);
    ImGui::SliderInt("circle_red", &circle_red, 0, 255);
    ImGui::SliderInt("circle_green", &circle_green, 0, 255);
    ImGui::SliderInt("circle_blue", &circle_blue, 0, 255);
    ImGui::SliderInt("circle_center_x", &circle_center_x, circle_diameter / 2, defaultWidth - (circle_diameter / 2));
    ImGui::SliderInt("circle_center_y", &circle_center_y, circle_diameter / 2, defaultHeight - (circle_diameter / 2));
    if (ImGui::Button("reset"))
      circle_center_x = def_circle_center_x, circle_center_y = def_circle_center_y, circle_diameter = def_circle_diameter, circle_thickness = def_circle_thickness, circle_red = def_circle_red, circle_green = def_circle_green, circle_blue = def_circle_blue;
    ImGui::ColorEdit3("clear color", (float*)&clear_color);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
    //
    // Rendering
    ImGui::Render();
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    // Update and Render additional Platform Windows
    // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
    //  For this specific demo app we could also call SDL_GL_MakeCurrent(window, gl_context) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      SDL_Window* backup_current_window = SDL_GL_GetCurrentWindow();
      SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
    }
    SDL_GL_SwapWindow(window);
  }
  // Cleanup
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
int main(const int argc, char **argv) {
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  //ImageDisplayer displayer(devInfoMain->startingWidth, devInfoMain->startingHeight);
  fprintf(stderr, "\n[main] Starting main loop now\n");
  std::thread bgThread(startImGuiThread);
  bgThread.detach();
  while (shouldLoop) {
    if (frame_number % 2 == 0) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      if (isDualInput) {
        background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
        background_task_cap_alt.wait();
      }
      background_task_cap_main.wait();
      if (isDualInput) {
#pragma omp parallel for simd num_threads(num_threads)
        for (int t = 0; t < num_threads; ++t) {
          const int start = ((t * numPixels) / num_threads), end = (((t + 1) * numPixels) / num_threads);
          auto task = std::packaged_task<void()>([=] {
            for (int i = start; i < end; i += 8) {
              uint8x8x3_t rgb = vld3_u8(&devInfoMain->outputFrame[i * 3]);
              uint8x8_t r1 = rgb.val[0];
              uint8x8_t g1 = rgb.val[1];
              uint8x8_t b1 = rgb.val[2];
              uint8x8x4_t rgba2 = vld4_u8(&outputWithAlpha[i * 4]);
              uint8x8_t r2 = rgba2.val[0];
              uint8x8_t g2 = rgba2.val[1];
              uint8x8_t b2 = rgba2.val[2];
              uint8x8_t alpha = rgba2.val[3];
              uint16x8_t alpha_ratio = vmovl_u8(alpha);
              uint16x8_t inv_alpha_ratio = vsubq_u16(vdupq_n_u16(255), alpha_ratio);
              uint16x8_t r16 = vaddq_u16(vmull_u8(r1, vqmovn_u16(inv_alpha_ratio)), vmull_u8(r2, vqmovn_u16(alpha_ratio)));
              uint16x8_t g16 = vaddq_u16(vmull_u8(g1, vqmovn_u16(inv_alpha_ratio)), vmull_u8(g2, vqmovn_u16(alpha_ratio)));
              uint16x8_t b16 = vaddq_u16(vmull_u8(b1, vqmovn_u16(inv_alpha_ratio)), vmull_u8(b2, vqmovn_u16(alpha_ratio)));
              r16 = vrshrq_n_u16(r16, 8);
              g16 = vrshrq_n_u16(g16, 8);
              b16 = vrshrq_n_u16(b16, 8);
              rgb.val[0] = vqmovn_u16(r16);
              rgb.val[1] = vqmovn_u16(g16);
              rgb.val[2] = vqmovn_u16(b16);
              vst3_u8(&devInfoMain->outputFrame[i * 3], rgb);
            }
          });
          futures[t] = task.get_future();
          std::thread(std::move(task)).detach();
        }
        for (auto& f : futures) f.wait();
      }
      draw_hollow_circle(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight, circle_center_x, circle_center_y, circle_diameter, circle_thickness, circle_red, circle_green, circle_blue);
      /*std::vector<int> indices(numPixels);
      std::iota(indices.begin(), indices.end(), 0);
      std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i) {
        int idx = i * 8; // Processing 8 pixels at a time
        if (idx + 7 < numPixels) {
          uint8x8x3_t rgb = vld3_u8(&devInfoMain->outputFrame[idx * 3]);
          uint8x8x4_t rgba;
          rgba.val[0] = rgb.val[0];
          rgba.val[1] = rgb.val[1];
          rgba.val[2] = rgb.val[2];
          rgba.val[3] = vdup_n_u8(alpha_channel_amount);
          vst4_u8(&outputWithAlpha[idx * 4], rgba);
        } else {
          // Process the remaining pixels
          for (; idx < numPixels; ++idx) {
            outputWithAlpha[idx * 4] = devInfoMain->outputFrame[idx * 3];
            outputWithAlpha[idx * 4 + 1] = devInfoMain->outputFrame[idx * 3 + 1];
            outputWithAlpha[idx * 4 + 2] = devInfoMain->outputFrame[idx * 3 + 2];
            outputWithAlpha[idx * 4 + 3] = alpha_channel_amount;
          }
        }
      });
      char* currentPixel = fbmem;
      for (unsigned int y = 0; y < vinfo.yres; y++) {
        for (unsigned int x = 0; x < vinfo.xres; x++) {
          int pixelOffset = y * vinfo.xres * 3 + x * 3;
          unsigned char r = devInfoMain->outputFrame[pixelOffset];
          unsigned char g = devInfoMain->outputFrame[pixelOffset + 1];
          unsigned char b = devInfoMain->outputFrame[pixelOffset + 2];
          unsigned short pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
          *(unsigned short*)currentPixel = pixel;
          currentPixel += 2;
        }
      }*/
      /*char* currentPixel = fbmem;
      for (int p = 0; p < numPixels; p++) {
        *(uint32_t*)currentPixel = outputWithAlpha[p];
      }*/
      //displayer.display_rgb24_image(devInfoMain->outputFrame);
      //display_rgb24_image(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight);
#pragma omp parallel for simd num_threads(num_threads)
      for (int y = 0; y < (int)vinfo.yres; y++) {
        for (int x = 0; x < (int)vinfo.xres; x += 8) { // Process 8 pixels at a time
          int pixelOffset = y * vinfo.xres * 3 + x * 3;
          uint8x8x3_t rgb = vld3_u8(&devInfoMain->outputFrame[pixelOffset]);
          if (!IS_RGB_DEVICE) {
            // Swap red and blue channels
            uint8x8_t tmp = rgb.val[0];
            rgb.val[0] = rgb.val[2];
            rgb.val[2] = tmp;
          }
          uint8x8_t r = vshr_n_u8(rgb.val[0], 3);
          uint8x8_t g = vshr_n_u8(rgb.val[1], 2);
          uint8x8_t b = vshr_n_u8(rgb.val[2], 3);
          uint16x8_t pixel = vshlq_n_u16(vmovl_u8(r), 11);
          pixel = vsliq_n_u16(pixel, vmovl_u8(g), 5);
          pixel = vorrq_u16(pixel, vmovl_u8(b));
          vst1q_u16(reinterpret_cast<uint16_t*>(fbmem + y * vinfo.xres * 2 + x * 2), pixel);
        }
      }
    }
    frame_number++;
  }
  //displayer.stop_displaying();
  cleanup_vars();
  return 0;
}
