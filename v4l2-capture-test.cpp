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
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <popt.h>
#include <stdexcept>

using namespace cv;
using namespace std;
#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define IS_RGB_DEVICE false // change in case capture device is really RGB24 and not BGR24
int fbfd = -1, ret = 1, retSize = 1, frame_number = 0, byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, alpha_channel_amount = 127, allDevicesTargetFramerate = 30, numPixels = defaultWidth * defaultHeight;
const int def_circle_center_x = defaultWidth / 2, def_circle_center_y = defaultHeight / 2, def_circle_diameter = 5, def_circle_thickness = 1, def_circle_red = 0, def_circle_green = 0, def_circle_blue = 0;
int circle_center_x = def_circle_center_x, circle_center_y = def_circle_center_y, circle_diameter = def_circle_diameter, circle_thickness = def_circle_thickness, circle_red = def_circle_red, circle_green = def_circle_green, circle_blue = def_circle_blue, configRefreshDelay = 33;
float zoom_factor_val_main = 1.0F, zoom_factor_val_alt = 1.0F;
unsigned char* outputWithAlpha = new unsigned char[defaultWidth * defaultHeight * 4];
unsigned char* prevOutputFrameMain = new unsigned char[defaultWidth * defaultHeight * byteScaler];
unsigned char* prevOutputFrameAlt = new unsigned char[defaultWidth * defaultHeight * byteScaler];
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
char* fbmem;
long int screensize;
size_t stride;
bool experimentalMode = false;
std::mutex mtx;
int num_corners_x = 8;
int num_corners_y = 6;
float square_size = 0.0245; // size of the calibration pattern squares in meters
Size board_size(num_corners_x, num_corners_y);
std::vector<std::vector<Point3f>> object_points;
std::vector<std::vector<Point2f>> image_points;
std::vector<Point3f> pattern_points;
struct Pixel {
  uint8_t y;  // Brightness (luma) of the first pixel
  uint8_t u;  // Chrominance (blue-difference) component
  uint8_t y2; // Brightness (luma) of the second pixel
  uint8_t v;  // Chrominance (red-difference) component
};

std::vector<Pixel> convertToYUYV(const std::vector<uint8_t>& grayscale, int width, int height) {
  std::vector<Pixel> yuyvData;
  yuyvData.reserve(width * height / 2);
  const uint8_t neutralValue = 128;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; x += 16) {
      uint8x16_t y1 = vld1q_u8(&grayscale[y * width + x]);
      uint8x16_t y2 = vld1q_u8(&grayscale[y * width + x + 8]);
      uint8x16_t u = vdupq_n_u8(neutralValue);
      uint8x16_t v = vdupq_n_u8(neutralValue);
      Pixel pixel;
      vst1q_u8(&pixel.y, y1);
      vst1q_u8(&pixel.y2, y2);
      vst1q_u8(&pixel.u, u);
      vst1q_u8(&pixel.v, v);
      yuyvData.push_back(pixel);
    }
  }
  return yuyvData;
}
std::vector<unsigned char> convertRGB888toGrayscaleNEON(const unsigned char* rgb888, int width, int height) {
  const int numPixels = width * height;
  std::vector<uint8_t> grayscaleData(numPixels);

  // Process 8 pixels (24 bytes) at a time
  const int numIterations = numPixels / 8;
  for (int i = 0; i < numIterations; ++i) {
    const uint8x8x3_t rgb = vld3_u8(&rgb888[i * 24]);

    const uint16x8_t weightedSumLow = vmull_u8(rgb.val[0], vdup_n_u8(77));
    const uint16x8_t weightedSumMid = vmull_u8(rgb.val[1], vdup_n_u8(151));
    const uint16x8_t weightedSumHigh = vmull_u8(rgb.val[2], vdup_n_u8(28));

    const uint16x4_t weightedSumLo = vadd_u16(vget_low_u16(weightedSumLow), vget_low_u16(weightedSumMid));
    const uint16x4_t weightedSumHi = vadd_u16(vget_high_u16(weightedSumLow), vget_high_u16(weightedSumMid));

    const uint16x4_t weightedSum = vadd_u16(vadd_u16(weightedSumLo, weightedSumHi), vget_low_u16(weightedSumHigh));

    const uint8x8_t grayscale = vshrn_n_u16(vcombine_u16(weightedSum, weightedSum), 8);

    vst1_u8(&grayscaleData[i * 8], grayscale);
  }

  // Handle the remaining pixels
  const int remainingPixels = numPixels % 8;
  if (remainingPixels > 0) {
    uint8_t remainingRgb[24] = { 0 };
    for (int j = 0; j < remainingPixels; ++j) {
      remainingRgb[j] = rgb888[numIterations * 24 + j];
    }
    const uint8x8x3_t rgb = vld3_u8(remainingRgb);

    const uint16x8_t weightedSumLow = vmull_u8(rgb.val[0], vdup_n_u8(77));
    const uint16x8_t weightedSumMid = vmull_u8(rgb.val[1], vdup_n_u8(151));
    const uint16x8_t weightedSumHigh = vmull_u8(rgb.val[2], vdup_n_u8(28));

    const uint16x4_t weightedSumLo = vadd_u16(vget_low_u16(weightedSumLow), vget_low_u16(weightedSumMid));
    const uint16x4_t weightedSumHi = vadd_u16(vget_high_u16(weightedSumLow), vget_high_u16(weightedSumMid));

    const uint16x4_t weightedSum = vadd_u16(vadd_u16(weightedSumLo, weightedSumHi), vget_low_u16(weightedSumHigh));

    const uint8x8_t grayscale = vshrn_n_u16(vcombine_u16(weightedSum, weightedSum), 8);

    uint8_t* grayscalePtr = &grayscaleData[numIterations * 8];
    vst1_u8(grayscalePtr, grayscale);

    // Fill the remaining pixels with zeros
    for (int i = remainingPixels; i < 8; ++i) {
      grayscalePtr[i] = 0;
    }
  }

  return grayscaleData;
}
void zoom_image(const unsigned char* src, unsigned char* dest, int width, int height, float zoom_factor) {
  if (zoom_factor <= 0) {
    throw std::invalid_argument("Zoom factor must be greater than 0");
  }
  int new_width = static_cast<int>(width * zoom_factor);
  int new_height = static_cast<int>(height * zoom_factor);
  int offset_x = (new_width - width) / 2;
  int offset_y = (new_height - height) / 2;
#pragma omp parallel for
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int src_x = std::clamp(static_cast<int>(x / zoom_factor) + offset_x, 0, width - 1);
      int src_y = std::clamp(static_cast<int>(y / zoom_factor) + offset_y, 0, height - 1);
      int dest_index = (y * width + x) * 3;
      int src_index = (src_y * width + src_x) * 3;
      dest[dest_index] = src[src_index];
      dest[dest_index + 1] = src[src_index + 1];
      dest[dest_index + 2] = src[src_index + 2];
    }
  }
}
void zoom_image_custom_aspect_ratio(const unsigned char* src, unsigned char* dest, int width, int height, float zoom_factor, float aspect_ratio_width, float aspect_ratio_height) {
  if (zoom_factor <= 0) {
    throw std::invalid_argument("Zoom factor must be greater than 0");
  }
  float src_aspect_ratio = static_cast<float>(width) / height;
  int new_width = static_cast<int>(width * zoom_factor);
  int new_height = static_cast<int>(height * zoom_factor);
  int cropped_width = static_cast<int>(new_height * aspect_ratio_width / aspect_ratio_height);
  int cropped_height = static_cast<int>(new_width * aspect_ratio_height / aspect_ratio_width);
  if (src_aspect_ratio > aspect_ratio_width / aspect_ratio_height) {
    new_width = cropped_width;
  } else {
    new_height = cropped_height;
  }
  int offset_x = (new_width - width) / 2;
  int offset_y = (new_height - height) / 2;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int src_x = std::clamp(static_cast<int>(x / zoom_factor) + offset_x, 0, width - 1);
      int src_y = std::clamp(static_cast<int>(y / zoom_factor) + offset_y, 0, height - 1);
      int dest_index = (y * width + x) * 3;
      int src_index = (src_y * width + src_x) * 3;
      dest[dest_index] = src[src_index];
      dest[dest_index + 1] = src[src_index + 1];
      dest[dest_index + 2] = src[src_index + 2];
    }
  }
}
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
    shouldLoop.store(false);
    return 1;
    //exit(EXIT_FAILURE);
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
      fprintf(stderr, "%s error %d, %s\n", "VIDIOC_DQBUF", errno, strerror(errno));
      shouldLoop.store(false);
      return 1;
      //errno_exit("VIDIOC_DQBUF");
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
    /*if (getenv("CALIB")) {
      fprintf(stderr, "Entering single-camera calibration mode..\n");
      isCalibrationRun = true;
    }*/
    break;
  case 4:
    devNames.push_back(args[1]);
    devNames.push_back(args[2]);
    devNames.push_back(args[3]);
    isDualInput = true;
    /*if (getenv("CALIB")) {
      fprintf(stderr, "Entering dual-camera calibration mode..\n");
      isCalibrationRun = true;
    }*/
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
  system("/opt/TurboVNC/bin/vncserver -kill :1");
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
  usleep(1000);
  fprintf(stderr, "\n");
  fbfd = open(devNames.at(isDualInput ? 2 : 1).c_str(), O_RDWR);
  if (fbfd == -1) {
    fprintf(stderr, "[%s]: Error: cannot open framebuffer device", devNames.back().c_str());
    exit(1);
  }
  ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
  screensize = vinfo.xres * vinfo.yres * (vinfo.bits_per_pixel / 8);
  stride = vinfo.xres * (vinfo.bits_per_pixel / 8);
  fprintf(stderr, "[%s]: Actual frame buffer configuration: BPP: %u, xres: %u, yres: %u, screensize: %ld, stride: %lu\n", devNames.back().c_str(), vinfo.bits_per_pixel, vinfo.xres, vinfo.yres, screensize, stride);
  if (vinfo.bits_per_pixel != 24) {
    fprintf(stderr, "[%s]: Setting bit-depth to 24..\n", devNames.back().c_str());
    std::string fbsetCmd = "fbset -fb " + devNames.back() + " -depth 24";
    system(fbsetCmd.c_str());
    close(fbfd);
    fbfd = open(devNames.at(isDualInput ? 2 : 1).c_str(), O_RDWR);
    if (fbfd == -1) {
      fprintf(stderr, "[%s]: Error: cannot open framebuffer device", devNames.back().c_str());
      exit(1);
    }
    ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
    screensize = vinfo.xres * vinfo.yres * (vinfo.bits_per_pixel / 8);
    stride = vinfo.xres * (vinfo.bits_per_pixel / 8);
    fprintf(stderr, "[%s]: New actual frame buffer configuration: BPP: %u, xres: %u, yres: %u, screensize: %ld, stride: %lu\n", devNames.back().c_str(), vinfo.bits_per_pixel, vinfo.xres, vinfo.yres, screensize, stride);
  }
  if (vinfo.bits_per_pixel != 24)
    fprintf(stderr, "[%s]: WARN: Latency will likely be increased as we are not running in a 24-BPP display mode!\nSomething possibly went wrong when setting the frame buffer bit-depth.\n", devNames.back().c_str());
  if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
    fprintf(stderr, "[%s]: Error getting fixed frame buffer information\n", devNames.back().c_str());
    exit(EXIT_FAILURE);
  }
  fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fbmem == MAP_FAILED) {
    fprintf(stderr, "[%s]: Error: failed to mmap framebuffer device to memory", devNames.back().c_str());
    exit(1);
  }
}
void neon_overlay_rgba32_on_rgb24() {
#pragma omp parallel for simd num_threads(num_threads)
  for (int t = 0; t < num_threads; ++t) {
    const int start = ((t * numPixels) / num_threads), end = (((t + 1) * numPixels) / num_threads);
    auto task = std::packaged_task<void()>([=] {
      for (int i = start; i < end; i += 8) {
        /*background_task_cap_main.wait();
        uint8x8x3_t rgb = vld3_u8(&devInfoMain->outputFrame[i * 3]);*/
        uint8x8x3_t rgb = vld3_u8(&prevOutputFrameMain[i * 3]);
        uint8x8_t r1 = rgb.val[0];
        uint8x8_t g1 = rgb.val[1];
        uint8x8_t b1 = rgb.val[2];
        /*background_task_cap_alt.wait();
        uint8x8x3_t rgb2 = vld3_u8(&devInfoAlt->outputFrame[i * 3]);*/
        uint8x8x3_t rgb2 = vld3_u8(&prevOutputFrameAlt[i * 3]);
        uint8x8_t r2 = rgb2.val[0];
        uint8x8_t g2 = rgb2.val[1];
        uint8x8_t b2 = rgb2.val[2];
        uint8x8_t alpha = vdup_n_u8(alpha_channel_amount);
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
    ImGui::Checkbox("experimental mode", &experimentalMode);
    if (devNames.size() > 2)
      ImGui::Checkbox("dual-in", &isDualInput);
    /*ImGui::SliderInt("alpha_channel_amount", &alpha_channel_amount, 0, 255);
    ImGui::SliderInt("diameter", &circle_diameter, 1, defaultHeight);
    ImGui::SliderInt("circle_thickness", &circle_thickness, 1, circle_diameter / 2);
    ImGui::SliderInt("circle_red", &circle_red, 0, 255);
    ImGui::SliderInt("circle_green", &circle_green, 0, 255);
    ImGui::SliderInt("circle_blue", &circle_blue, 0, 255);
    ImGui::SliderInt("circle_center_x", &circle_center_x, circle_diameter / 2, defaultWidth - (circle_diameter / 2));
    ImGui::SliderInt("circle_center_y", &circle_center_y, circle_diameter / 2, defaultHeight - (circle_diameter / 2));*/
    ImGui::SliderFloat("zoom_factor_val_main", &zoom_factor_val_main, 0.1F, 2.0F, "%.3f");
    ImGui::SliderFloat("zoom_factor_val_alt", &zoom_factor_val_alt, 0.1F, 2.0F, "%.3f");
    if (ImGui::Button("reset"))
      zoom_factor_val_main = 1.0F, zoom_factor_val_alt = 1.0F;
      //circle_center_x = def_circle_center_x, circle_center_y = def_circle_center_y, circle_diameter = def_circle_diameter, circle_thickness = def_circle_thickness, circle_red = def_circle_red, circle_green = def_circle_green, circle_blue = def_circle_blue;
    ImGui::ColorEdit3("clear color", (float*)&clear_color);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    if (ImGui::Button("Quit")) {
      shouldLoop = false;
      done = true;
    }
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
    usleep(66000);
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
int main(const int argc, char** argv) {
  system("rw && (v4l2-ctl -d /dev/video0 --set-edid=file=/root/1080P50EDID.txt --fix-edid-checksums && sleep 1 && v4l2-ctl -d /dev/video0 --query-dv-timings && sleep 1 && v4l2-ctl -d /dev/video0 --set-dv-bt-timings query && sleep 1 && v4l2-ctl -d /dev/video0 -V && sleep 1 && v4l2-ctl -d /dev/video0 -v pixelformat=RGB3 && echo -n '/dev/video0:' && v4l2-ctl -d /dev/video0 --log-status) & (v4l2-ctl -d /dev/video1 --set-edid=file=/root/1080P50EDID.txt --fix-edid-checksums && sleep 1 && v4l2-ctl -d /dev/video1 --query-dv-timings && sleep 1 && v4l2-ctl -d /dev/video1 --set-dv-bt-timings query && sleep 1 && v4l2-ctl -d /dev/video1 -V && sleep 1 && v4l2-ctl -d /dev/video1 -v pixelformat=RGB3 && echo -n '/dev/video1:' && v4l2-ctl -d /dev/video1 --log-status) & sleep 3 && wait && fbset -fb /dev/fb0 -g 1920 1080 1920 1080 24");
  system("export DISPLAY=:1 && export LIBGL_ALWAYS_INDIRECT=0 && xrefresh && /opt/TurboVNC/bin/vncserver -noautokill -depth 24 -geometry 466x466 +extension GLX :1");
  // Create a window to display the results
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  usleep(1000);
  if (isDualInput) {
    std::thread bgThread(startImGuiThread);
    bgThread.detach();
  }
  // Load calibration images
  /*vector<Mat> images;
  for (int i = 1; i <= 10; i++) {
    string filename = "calib" + to_string(i) + ".png";
    Mat img = imread(filename, IMREAD_GRAYSCALE);
    if (img.empty()) {
      cerr << "Error: Could not read image " << filename << endl;
      return -1;
    }
    images.push_back(img);
  }
  // Define object points
  vector<vector<Point3f>> object_points;
  vector<Point3f> corners;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 8; j++) {
      corners.push_back(Point3f(i * 2.5f, j * 2.5f, 0.0f));
    }
  }
  for (int i = 0; i < (int)images.size(); i++) {
    object_points.push_back(corners);
  }
  // Find corners in images
  vector<vector<Point2f>> image_points;
  for (int i = 0; i < (int)images.size(); i++) {
    vector<Point2f> corners;
    bool found = findChessboardCorners(images[i], Size(6, 8), corners);
    if (found) {
      image_points.push_back(corners);
      cornerSubPix(images[i], corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
      drawChessboardCorners(images[i], Size(6, 8), corners, found);
    }
  }
  // Calibrate camera
  Mat camera_matrix, distortion_coeffs;
  vector<Mat> rvecs, tvecs;
  calibrateCamera(object_points, image_points, images[0].size(), camera_matrix, distortion_coeffs, rvecs, tvecs);
  // save camera parameters to file
  FileStorage fs("camera_params.xml", FileStorage::WRITE);
  fs << "camera_matrix" << camera_matrix;
  fs << "distortion_coeffs" << distortion_coeffs;
  fs.release();*/
  if (!isDualInput)
    num_threads = num_threads * (num_threads * 3);
  // Capture and ignore some frames for about 1/2 seconds due to strange issue with first frame(s)
  for (int i = 0; i < 60; i++) {
    if (isDualInput) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
      background_task_cap_main.wait();
      background_task_cap_alt.wait();
    } else {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
    }
  }
  fprintf(stderr, "\n[main] Starting main loop now\n");
  std::vector<unsigned char> greyscaleData;
  std::vector<Pixel> yuyvData;
  while (shouldLoop) {
    if (isDualInput) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
      background_task_cap_main.wait();
      zoom_image(devInfoMain->outputFrame, prevOutputFrameMain, defaultWidth, defaultHeight, zoom_factor_val_main);
      greyscaleData.clear();
      yuyvData.clear();
      background_task_cap_alt.wait();
      zoom_image(devInfoAlt->outputFrame, prevOutputFrameAlt, defaultWidth, defaultHeight, zoom_factor_val_alt);
      neon_overlay_rgba32_on_rgb24();
      greyscaleData = convertRGB888toGrayscaleNEON(devInfoMain->outputFrame, defaultWidth, defaultHeight);
      yuyvData = convertToYUYV(greyscaleData, defaultWidth, defaultHeight);
      memcpy(fbmem, devInfoMain->outputFrame, screensize);
    } else {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
      memcpy(fbmem, devInfoMain->outputFrame, screensize);
    }
    usleep(33000 * 0.5F);
  }
  usleep(1000000);
  cleanup_vars();
  return 0;
}