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
#include <array>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
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
#include <ilclient.h>

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define IS_RGB_DEVICE false // change in case capture device is really RGB24 and not BGR24
int fbfd = -1, ret = 1, retSize = 1, frame_number = 0, byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, alpha_channel_amount = 0, allDevicesTargetFramerate = 30, numPixels = defaultWidth * defaultHeight;
const int def_circle_center_x = defaultWidth / 2, def_circle_center_y = defaultHeight / 2, def_circle_diameter = 5, def_circle_thickness = 1, def_circle_red = 0, def_circle_green = 0, def_circle_blue = 0;
int circle_center_x = def_circle_center_x, circle_center_y = def_circle_center_y, circle_diameter = def_circle_diameter, circle_thickness = def_circle_thickness, circle_red = def_circle_red, circle_green = def_circle_green, circle_blue = def_circle_blue, configRefreshDelay = 33;
unsigned char* outputWithAlpha = new unsigned char[defaultWidth * defaultHeight * 4];
unsigned char* prevOutputFrame = new unsigned char[defaultWidth * defaultHeight * byteScaler];
int num_threads = std::thread::hardware_concurrency() + 1;
//const int num_threads = std::thread::hardware_concurrency() / 2;
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
bool experimentalMode = false;
uint16_t *rgb565le = new uint16_t[defaultWidth * defaultHeight * 2];
std::mutex mtx;

// Some functions to be used later which are maybe a bit more polished though not in use currently:
uint16_t rgb888torgb565_pixel(const std::array<uint8_t, 3>& rgb888Pixel) {
  uint16_t r = ((rgb888Pixel[2] >> 3) & 0x1f);
  uint16_t g = (((rgb888Pixel[1] >> 2) & 0x3f) << 5);
  uint16_t b = (((rgb888Pixel[0] >> 3) & 0x1f) << 11);
  return r | g | b;
}
uint16_t bgr888torgb565_pixel(const std::array<uint8_t, 3>& bgr888Pixel) {
  uint16_t b = ((bgr888Pixel[2] >> 3) & 0x1f);
  uint16_t g = (((bgr888Pixel[1] >> 2) & 0x3f) << 5);
  uint16_t r = (((bgr888Pixel[0] >> 3) & 0x1f) << 11);
  return r | g | b;
}
uint16_t rgb888torgb565_pixel_neon(const uint8x8_t& rgb888Pixel) {
  uint8_t r8 = vget_lane_u8(rgb888Pixel, 2);
  uint8_t g8 = vget_lane_u8(rgb888Pixel, 1);
  uint8_t b8 = vget_lane_u8(rgb888Pixel, 0);
  uint16_t r = ((r8 >> 3) & 0x1f);
  uint16_t g = (((g8 >> 2) & 0x3f) << 5);
  uint16_t b = (((b8 >> 3) & 0x1f) << 11);
  return r | g | b;
}
uint16_t bgr888torgb565_pixel_neon(const uint8x8_t& bgr888Pixel) {
  uint8_t b8 = vget_lane_u8(bgr888Pixel, 2);
  uint8_t g8 = vget_lane_u8(bgr888Pixel, 1);
  uint8_t r8 = vget_lane_u8(bgr888Pixel, 0);
  uint16_t r = ((r8 >> 3) & 0x1f);
  uint16_t g = (((g8 >> 2) & 0x3f) << 5);
  uint16_t b = (((b8 >> 3) & 0x1f) << 11);
  return r | g | b;
}
uint16x8_t rgb888torgb565_8pixels_neon(const uint8x8x3_t& rgb888Pixels) {
  // Load 8 R, G, and B values from the input structure
  uint8x8_t r8 = rgb888Pixels.val[2];
  uint8x8_t g8 = rgb888Pixels.val[1];
  uint8x8_t b8 = rgb888Pixels.val[0];
  // Convert RGB888 values to RGB565
  uint16x8_t r = vshll_n_u8(r8, 8); // Shift left by 8 to prepare for merging with G and B
  r = vshrq_n_u16(r, 11); // Shift right by 3 (masking with 0x1F) and then by 8 (to be in the final position)
  uint16x8_t g = vshll_n_u8(g8, 8); // Shift left by 8 to prepare for merging with R and B
  g = vshrq_n_u16(g, 10); // Shift right by 2 (masking with 0x3F) and then by 8 (to be in the final position)
  uint16x8_t b = vshll_n_u8(b8, 8); // Shift left by 8 to prepare for merging with R and G
  b = vshrq_n_u16(b, 13); // Shift right by 3 (masking with 0x1F)
  // Combine R, G, and B channels
  uint16x8_t rgb565 = vorrq_u16(r, g);
  rgb565 = vorrq_u16(rgb565, b);
  return rgb565;
}
uint16x8_t bgr888torgb565_8pixels_neon(const uint8x8x3_t& bgr888Pixels) {
  // Load 8 B, G, and R values from the input structure
  uint8x8_t b8 = bgr888Pixels.val[2];
  uint8x8_t g8 = bgr888Pixels.val[1];
  uint8x8_t r8 = bgr888Pixels.val[0];
  // Convert BGR888 values to RGB565
  uint16x8_t r = vshll_n_u8(r8, 8); // Shift left by 8 to prepare for merging with G and B
  r = vshrq_n_u16(r, 11); // Shift right by 3 (masking with 0x1F) and then by 8 (to be in the final position)
  uint16x8_t g = vshll_n_u8(g8, 8); // Shift left by 8 to prepare for merging with R and B
  g = vshrq_n_u16(g, 10); // Shift right by 2 (masking with 0x3F) and then by 8 (to be in the final position)
  uint16x8_t b = vshll_n_u8(b8, 8); // Shift left by 8 to prepare for merging with R and G
  b = vshrq_n_u16(b, 13); // Shift right by 3 (masking with 0x1F)
  // Combine B, G, and R channels
  uint16x8_t rgb565 = vorrq_u16(r, g);
  rgb565 = vorrq_u16(rgb565, b);
  return rgb565;
}
// there's likely better ways to do this for the test_load_8pixels function; untested
template <int I>
void print_rgb565_pixel(const uint16x8_t& rgb565Pixels) {
  uint16_t pixel = vgetq_lane_u16(rgb565Pixels, I);
  std::cout << "RGB565 pixel " << I + 1 << ": " << std::hex << pixel << std::endl;
  if constexpr (I > 0) {
    print_rgb565_pixel<I - 1>(rgb565Pixels);
  }
}
int test_load_8pixels() {
  // Create 8 RGB888 pixels as input
  uint8_t r_values[8] = { 255, 0, 0, 255, 255, 127, 63, 31 };
  uint8_t g_values[8] = { 0, 255, 0, 255, 127, 255, 127, 63 };
  uint8_t b_values[8] = { 0, 0, 255, 255, 127, 63, 255, 255 };
  // Load the RGB888 values into a uint8x8x3_t structure
  uint8x8_t r8 = vld1_u8(r_values);
  uint8x8_t g8 = vld1_u8(g_values);
  uint8x8_t b8 = vld1_u8(b_values);
  uint8x8x3_t rgb888Pixels = { r8, g8, b8 };
  // Call the function to convert the RGB888 pixels to RGB565
  uint16x8_t rgb565Pixels = rgb888torgb565_8pixels_neon(rgb888Pixels);
  // Print the converted RGB565 pixel values
  print_rgb565_pixel<7>(rgb565Pixels);
  return 0;
}
// Should work better as an example function
void convert_rgb888_to_rgb565_neon(const std::vector<uint8_t>& rgb888Data, std::vector<uint16_t>& rgb565Data) {
  size_t numPixels = rgb888Data.size() / 3;
  for (size_t i = 0; i < numPixels; i += 8) {
    uint8x8_t r8 = vld1_u8(&rgb888Data[i * 3]);
    uint8x8_t g8 = vld1_u8(&rgb888Data[i * 3 + 1]);
    uint8x8_t b8 = vld1_u8(&rgb888Data[i * 3 + 2]);
    uint8x8x3_t rgb888Pixels = { r8, g8, b8 };
    uint16x8_t rgb565Pixels = rgb888torgb565_8pixels_neon(rgb888Pixels);
    vst1q_u16(&rgb565Data[i], rgb565Pixels);
  }
}
void convert_bgr888_to_bgr565_neon(const std::vector<uint8_t>& bgr888Data, std::vector<uint16_t>& bgr565Data) {
  size_t numPixels = bgr888Data.size() / 3;
  for (size_t i = 0; i < numPixels; i += 8) {
    uint8x8_t b8 = vld1_u8(&bgr888Data[i * 3]);
    uint8x8_t g8 = vld1_u8(&bgr888Data[i * 3 + 1]);
    uint8x8_t r8 = vld1_u8(&bgr888Data[i * 3 + 2]);
    uint8x8x3_t bgr888Pixels = { r8, g8, b8 };
    uint16x8_t bgr565Pixels = bgr888torgb565_8pixels_neon(bgr888Pixels);
    vst1q_u16(&bgr565Data[i], bgr565Pixels);
  }
}
// End of new test functions
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
  tv.tv_usec = devInfos->frameDelayMicros*2;
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
  /*if (devInfos->index == 1) {
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
  }*/
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, buffers[buf.index].length); // copy frame data to frame buffer
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
  /*if (vinfo.bits_per_pixel != 16 || vinfo.xres != (unsigned int)devInfoMain->startingWidth || vinfo.yres != (unsigned int)devInfoMain->startingHeight) {
    fprintf(stderr, "Error: framebuffer does not accept RGB24 frames with %dx%d resolution\n", devInfoMain->startingWidth, devInfoMain->startingHeight);
    exit(1);
  }
  screensize = vinfo.xres * vinfo.yres * 2; // original size for RGB565LE
  stride = vinfo.xres * 2; // stride for RGB565LE format*/
  screensize = vinfo.xres * vinfo.yres * (vinfo.bits_per_pixel / 8);
  stride = vinfo.xres * (vinfo.bits_per_pixel / 8);
  fprintf(stderr, "Actual frame buffer configuration: BPP: %u, xres: %u, yres: %u, screensize: %ld, stride: %u\n", vinfo.bits_per_pixel, vinfo.xres, vinfo.yres, screensize, stride);
  if (vinfo.bits_per_pixel != 16) {
    fprintf(stderr, "WARN: Latency will likely be increased as we are not running in a 16-BPP display mode!\nThis could be due to having the vc4-kms-v3d driver is commented out in your /boot/config.txt\n");
    // TODO: stop talking to the frame buffer, then run this shell command, and then check if we are actually in 16-BPP mode before continuing, else fail and exit: fbset -fb /dev/fb0 -depth 16
  }
  if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
    perror("Error getting fixed frame buffer information");
    exit(EXIT_FAILURE);
  }
  fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fbmem == MAP_FAILED) {
    perror("Error: failed to mmap framebuffer device to memory");
    exit(1);
  }
}
int startImGuiThread() {
  // Setup SDL
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
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, vinfo.bits_per_pixel);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  SDL_Window* window = SDL_CreateWindow("Configuration", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 460, 460, window_flags);
  SDL_GLContext gl_context = SDL_GL_CreateContext(window);
  SDL_GL_MakeCurrent(window, gl_context);
  SDL_GL_SetSwapInterval(1); // Enable vsync
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
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
  ImGuiStyle& style = ImGui::GetStyle();
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
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
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
    ImGui::Text("Frame buffer controls");
    ImGui::Checkbox("experimental mode", &experimentalMode);
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
    ImGui::Text("Config window average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::Text("Config window settings");
    ImGui::SliderInt("refresh delay (ms)", &configRefreshDelay, 1, 1000);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(configRefreshDelay));
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
void rgb888_to_rgb565le_threaded(const unsigned char* src, uint16_t* dst, int width, int height, int start, int end) {
  const int num_pixels = end - start;
  src += start * 3;
  dst += start;
  int i = 0;

  // Process 8 pixels at a time using NEON intrinsics
  for (; i <= num_pixels - 8; i += 8, src += 8 * 3, dst += 8) {
    uint8x8x3_t rgb888 = vld3_u8(src);

    uint16x8_t r = vshrq_n_u16(vmovl_u8(rgb888.val[0]), 3);
    uint16x8_t g = vshrq_n_u16(vmovl_u8(rgb888.val[1]), 2);
    uint16x8_t b = vshrq_n_u16(vmovl_u8(rgb888.val[2]), 3);

    uint16x8_t rgb565 = vorrq_u16(vorrq_u16(vshlq_n_u16(r, 11), vshlq_n_u16(g, 5)), b);
    vst1q_u16(dst, rgb565);
  }

  // Process remaining pixels
  for (; i < num_pixels; ++i, src += 3, ++dst) {
    uint16_t r = (src[0] >> 3) << 11;
    uint16_t g = (src[1] >> 2) << 5;
    uint16_t b = (src[2] >> 3);

    dst[0] = r | g | b;
  }
}
void rgb888_to_rgb565le_multithreaded(const unsigned char* src, uint16_t* dst, int width, int height) {
  int num_pixels = width * height;
  int pixels_per_thread = num_pixels / num_threads;
  std::vector<std::thread> threads;

  for (int i = 0; i < num_threads; ++i) {
    int start = i * pixels_per_thread;
    int end = (i == num_threads - 1) ? num_pixels : start + pixels_per_thread;

    threads.emplace_back(std::thread(rgb888_to_rgb565le_threaded, src, dst, width, height, start, end));
  }

  // Wait for all threads to finish
  for (auto& t : threads) {
    t.join();
  }
}
void bgr888_to_rgb565le_threaded(const unsigned char* src, uint16_t* dst, int width, int height, int start, int end) {
  const int num_pixels = end - start;
  src += start * 3;
  dst += start;
  int i = 0;

  // Process 8 pixels at a time using NEON intrinsics
  for (; i <= num_pixels - 8; i += 8, src += 8 * 3, dst += 8) {
    uint8x8x3_t rgb888 = vld3_u8(src);
    uint16x8_t b = vshrq_n_u16(vmovl_u8(rgb888.val[0]), 3);
    uint16x8_t g = vshrq_n_u16(vmovl_u8(rgb888.val[1]), 2);
    uint16x8_t r = vshrq_n_u16(vmovl_u8(rgb888.val[2]), 3);
    uint16x8_t rgb565 = vorrq_u16(vorrq_u16(vshlq_n_u16(r, 11), vshlq_n_u16(g, 5)), b);
    vst1q_u16(dst, rgb565);
  }
  // Process remaining pixels
  for (; i < num_pixels; ++i, src += 3, ++dst) {
    uint16_t b = (src[0] >> 3) << 11;
    uint16_t g = (src[1] >> 2) << 5;
    uint16_t r = (src[2] >> 3);
    dst[0] = r | g | b;
  }
}
void bgr888_to_rgb565le_multithreaded(const unsigned char* src, uint16_t* dst, int width, int height) {
  int num_pixels = width * height;
  int pixels_per_thread = num_pixels / num_threads;
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    int start = i * pixels_per_thread;
    int end = (i == num_threads - 1) ? num_pixels : start + pixels_per_thread;
    threads.emplace_back(std::thread(bgr888_to_rgb565le_threaded, src, dst, width, height, start, end));
  }
  // Wait for all threads to finish
  for (auto& t : threads) {
    t.join();
  }
}
void rgb888_to_rgb565le(const unsigned char* src, uint16_t* dst, int width, int height) {
  const int num_pixels = width * height;
  int i = 0;

  // Process 8 pixels at a time using NEON intrinsics
  for (; i <= num_pixels - 8; i += 8, src += 8 * 3, dst += 8) {
    uint8x8x3_t rgb888 = vld3_u8(src);

    uint16x8_t b = vshrq_n_u16(vmovl_u8(rgb888.val[0]), 3);
    uint16x8_t g = vshrq_n_u16(vmovl_u8(rgb888.val[1]), 2);
    uint16x8_t r = vshrq_n_u16(vmovl_u8(rgb888.val[2]), 3);

    uint16x8_t rgb565 = vorrq_u16(vorrq_u16(vshlq_n_u16(r, 11), vshlq_n_u16(g, 5)), b);
    vst1q_u16(dst, rgb565);
  }

  // Process remaining pixels
  for (; i < num_pixels; ++i, src += 3, ++dst) {
    uint16_t b = (src[0] >> 3) << 11;
    uint16_t g = (src[1] >> 2) << 5;
    uint16_t r = (src[2] >> 3);

    dst[0] = r | g | b;
  }
}
void old_rgb888_to_rgb565lefbmem(const unsigned char* src) {
#pragma omp parallel for simd num_threads(num_threads)
  for (int y = 0; y < (int)vinfo.yres; y++) {
    for (int x = 0; x < (int)vinfo.xres; x += 8) { // Process 8 pixels at a time
      int pixelOffset = y * vinfo.xres * 3 + x * 3;
      uint8x8x3_t rgb = vld3_u8(&src[pixelOffset]);
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
void neon_overlay_rgba32_on_rgb24() {
#pragma omp parallel for simd num_threads(num_threads)
  for (int t = 0; t < num_threads; ++t) {
    const int start = ((t * numPixels) / num_threads), end = (((t + 1) * numPixels) / num_threads);
    auto task = std::packaged_task<void()>([=] {
      for (int i = start; i < end; i += 8) {
        background_task_cap_main.wait();
        uint8x8x3_t rgb = vld3_u8(&devInfoMain->outputFrame[i * 3]);
        uint8x8_t r1 = rgb.val[0];
        uint8x8_t g1 = rgb.val[1];
        uint8x8_t b1 = rgb.val[2];
        background_task_cap_alt.wait();
        uint8x8x3_t rgb2 = vld3_u8(&devInfoAlt->outputFrame[i * 3]);
        //uint8x8x4_t rgba2 = vld4_u8(&outputWithAlpha[i * 4]);
        uint8x8_t r2 = rgb2.val[0];
        uint8x8_t g2 = rgb2.val[1];
        uint8x8_t b2 = rgb2.val[2];
        uint8x8_t alpha = vdup_n_u8(alpha_channel_amount);
        //uint8x8_t alpha = rgba2.val[3];
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
  for (auto& f : futures) if (f.valid()) f.wait();
}
#ifndef ALIGN_UP
#define ALIGN_UP(x,y) ((x + (y)-1) & ~((y)-1))
#endif
typedef struct {
  DISPMANX_DISPLAY_HANDLE_T display;
  DISPMANX_MODEINFO_T info;
  void* image;
  DISPMANX_UPDATE_HANDLE_T update;
  DISPMANX_RESOURCE_HANDLE_T resource;
  DISPMANX_ELEMENT_HANDLE_T element;
  uint32_t vc_image_ptr;

} RECT_VARS_T;
static RECT_VARS_T  gRectVars;
static void FillRect(VC_IMAGE_TYPE_T type, void* image, int pitch, int aligned_height, int x, int y, int w, int h, int val) {
  int row;
  int col;
  uint16_t* line = (uint16_t*)image + y * (pitch >> 1) + x;
  for (row = 0; row < h; row++) {
    for (col = 0; col < w; col++) {
      line[col] = val;
    }
    line += (pitch >> 1);
  }
}
int test_dispmanx(void) {
  RECT_VARS_T* vars;
  uint32_t screen = 0;
  int ret;
  VC_RECT_T src_rect;
  VC_RECT_T dst_rect;
  VC_IMAGE_TYPE_T type = VC_IMAGE_RGB565;
  int pitch = ALIGN_UP(defaultWidth * 2, 32);
  int aligned_height = ALIGN_UP(defaultHeight, 16);
  //VC_DISPMANX_ALPHA_T alpha = { DISPMANX_FLAGS_ALPHA_FROM_SOURCE | DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS, 120, /*alpha 0->255*/ 0 };
  VC_DISPMANX_ALPHA_T alpha = {
    static_cast<DISPMANX_FLAGS_ALPHA_T>(DISPMANX_FLAGS_ALPHA_FROM_SOURCE | DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS),
    120, /*alpha 0->255*/
    0
  };
  vars = &gRectVars;
  bcm_host_init();
  printf("Open display[%i]...\n", screen);
  vars->display = vc_dispmanx_display_open(screen);
  ret = vc_dispmanx_display_get_info(vars->display, &vars->info);
  assert(ret == 0);
  printf("Display is %d x %d\n", vars->info.width, vars->info.height);
  vars->image = calloc(1, pitch * defaultHeight);
  assert(vars->image);
  FillRect(type, vars->image, pitch, aligned_height, 0, 0, defaultWidth, defaultHeight, 0xFFFF);
  FillRect(type, vars->image, pitch, aligned_height, 0, 0, defaultWidth, defaultHeight, 0xF800);
  FillRect(type, vars->image, pitch, aligned_height, 20, 20, defaultWidth - 40, defaultHeight - 40, 0x07E0);
  FillRect(type, vars->image, pitch, aligned_height, 40, 40, defaultWidth - 80, defaultHeight - 80, 0x001F);
  vars->resource = vc_dispmanx_resource_create(type, defaultWidth, defaultHeight, &vars->vc_image_ptr);
  assert(vars->resource);
  vc_dispmanx_rect_set(&dst_rect, 0, 0, defaultWidth, defaultHeight);
  ret = vc_dispmanx_resource_write_data(vars->resource, type, pitch, vars->image, &dst_rect);
  assert(ret == 0);
  vars->update = vc_dispmanx_update_start(10);
  assert(vars->update);
  vc_dispmanx_rect_set(&src_rect, 0, 0, defaultWidth << 16, defaultHeight << 16);
  vc_dispmanx_rect_set(&dst_rect, (vars->info.width - defaultWidth) / 2, (vars->info.height - defaultHeight) / 2, defaultWidth, defaultHeight);
  //vars->element = vc_dispmanx_element_add(vars->update, vars->display, 2000, /*layer*/ &dst_rect, vars->resource, &src_rect, DISPMANX_PROTECTION_NONE, &alpha, NULL, /*clamp*/ VC_IMAGE_ROT0);
  vars->element = vc_dispmanx_element_add(vars->update, vars->display, 2000, /*layer*/ &dst_rect, vars->resource, &src_rect, DISPMANX_PROTECTION_NONE, &alpha, NULL, /*clamp*/ DISPMANX_NO_ROTATE);
  ret = vc_dispmanx_update_submit_sync(vars->update);
  assert(ret == 0);
  printf("Sleeping for 10 seconds...\n");
  sleep(10);
  vars->update = vc_dispmanx_update_start(10);
  assert(vars->update);
  ret = vc_dispmanx_element_remove(vars->update, vars->element);
  assert(ret == 0);
  ret = vc_dispmanx_update_submit_sync(vars->update);
  assert(ret == 0);
  ret = vc_dispmanx_resource_delete(vars->resource);
  assert(ret == 0);
  ret = vc_dispmanx_display_close(vars->display);
  assert(ret == 0);
  return 0;
}
int main(const int argc, char** argv) {
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  fprintf(stderr, "\n[main] Starting main loop now\n");
  std::thread bgThread(startImGuiThread);
  bgThread.detach();
  if (!isDualInput)
    num_threads = num_threads * (num_threads * 3);
  if (isDualInput) {
    while (shouldLoop) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
      neon_overlay_rgba32_on_rgb24();
      //draw_hollow_circle_neon(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight, circle_center_x, circle_center_y, circle_diameter, circle_thickness, circle_red, circle_green, circle_blue);
      //draw_hollow_circle(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight, circle_center_x, circle_center_y, circle_diameter, circle_thickness, circle_red, circle_green, circle_blue);
      bgr888_to_rgb565le_multithreaded(devInfoMain->outputFrame, rgb565le, devInfoMain->startingWidth, devInfoMain->startingHeight);
      memcpy(fbmem, rgb565le, screensize);
    }
  } else {
    while (shouldLoop) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
      bgr888_to_rgb565le_multithreaded(devInfoMain->outputFrame, rgb565le, devInfoMain->startingWidth, devInfoMain->startingHeight);
      memcpy(fbmem, rgb565le, screensize);
    }
  }
  cleanup_vars();
  return 0;
}
