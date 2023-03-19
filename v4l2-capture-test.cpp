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

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define NUM_THREADS std::thread::hardware_concurrency()
#define IS_RGB_DEVICE false // change in case capture device is really RGB24 and not BGR24
int ret = 1, retSize = 1, frame_number = 0, byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, alpha_channel_amount = 127;
unsigned char* outputWithAlpha = new unsigned char[defaultWidth * defaultHeight * 4];
unsigned char* prevOutputFrame = new unsigned char[defaultWidth * defaultHeight * byteScaler];
bool isDualInput = false;
std::atomic<bool> shouldLoop;
std::future<int> background_task_cap_main;
std::future<int> background_task_cap_alt;
std::vector<std::string> devNames;
enum captureType {
  /*
   * TODO: Handle framerate divisor value differences between the two different models of
   * analog (CVBS) to HDMI converter boxes where this enum is passed later in the program
   */
  CHEAP_CONVERTER_BOX,
  EXPENSIVE_CONVERTER_BOX
};
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
#pragma omp parallel for simd num_threads(NUM_THREADS)
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
        int rawInputThroughput = (float)((float)(devInfos->framerate * devInfos->startingSize * 2.0F) / 125000.0F); // Measured in megabits/sec based on input framerate
        int rawOutputThroughput = (float)((((float)devInfos->framerate / devInfos->framerateDivisor) * devInfos->startingSize) / 125000.0F); // Measured in megabits/sec based on output framerate
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, startingWidth: %d, startingHeight: %d, startingSize: %d, framerate(actual): %u, framerateDivisor: %d, targetFramerate: %d, rawInputThroughput: ~%dMb/sec, rawOutputThroughput: ~%dMb/sec\n", devInfos->index, devInfos->device, devInfos->isTC358743, devInfos->startingWidth, devInfos->startingHeight, devInfos->startingSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, rawInputThroughput, rawOutputThroughput);
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
int get_frame(struct buffer* buffers, struct devInfo* devInfos, captureType capType) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period to wait for device to respond
  tv.tv_sec = 1;
  tv.tv_usec = 0;
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
    const size_t numPixels = devInfos->startingWidth * devInfos->startingHeight;
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
}
void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt, int argCnt, char **args) {
  commandline_usage(argCnt, args);
  fprintf(stderr, "[main] Initializing..\n");
  // allocate memory for structs
  init_vars(deviMain, bufMain, 3, 10, true, true, args[1], 0);
  if (isDualInput)
    init_vars(deviAlt, bufAlt, 3, 10, true, true, args[2], 1);
}
void writeFrameToFramebuffer(const unsigned char* frameData, const char* frameBufferDevName = "/dev/fb0") {
  int fbfd = open(frameBufferDevName, O_RDWR);
  if (fbfd == -1) {
    perror("Error: cannot open framebuffer device");
    exit(1);
  }
  struct fb_var_screeninfo vinfo;
  ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
  if (vinfo.bits_per_pixel != 16 || vinfo.xres != devInfoMain->startingWidth || vinfo.yres != devInfoMain->startingHeight) {
    fprintf(stderr, "Error: framebuffer does not accept RGB24 frames with %dx%d resolution\n", devInfoMain->startingWidth, devInfoMain->startingHeight);
    exit(1);
  }
  long int screensize = vinfo.xres * vinfo.yres * 2;
  char* fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fbmem == MAP_FAILED) {
    perror("Error: failed to mmap framebuffer device to memory");
    exit(1);
  }
  for (int y = 0; y < vinfo.yres; y++) {
    for (int x = 0; x < vinfo.xres; x += 8) { // Process 8 pixels at a time
      int pixelOffset = y * vinfo.xres * 3 + x * 3;
      uint8x8x3_t rgb = vld3_u8(&frameData[pixelOffset]);
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
  munmap(fbmem, screensize);
  close(fbfd);
}
void overlayRGBA32OnRGB24(unsigned char* rgb24, int width, int height, int numThreads = std::thread::hardware_concurrency()) {
  const int numPixels = width * height;
  std::vector<std::future<void>> futures(numThreads);
  for (int t = 0; t < numThreads; ++t) {
    const int start = ((t * numPixels) / numThreads), end = (((t + 1) * numPixels) / numThreads);
    auto task = std::packaged_task<void()>([=] {
      for (int i = start; i < end; i += 8) {
        uint8x8x3_t rgb = vld3_u8(&rgb24[i * 3]);
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
        vst3_u8(&rgb24[i * 3], rgb);
      }
    });
    futures[t] = task.get_future();
    std::thread(std::move(task)).detach();
  }
  for (auto& f : futures) f.wait();
}
int main(const int argc, char **argv) {
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  sleep(1);
  fprintf(stderr, "\n[main] Starting main loop now\n");
  shouldLoop.store(true);
  const char* fbDevName = devNames.at(isDualInput ? 2 : 1).c_str();
  while (shouldLoop) {
    if (frame_number % 2 == 0) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain, CHEAP_CONVERTER_BOX);
      if (isDualInput) {
        background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt, CHEAP_CONVERTER_BOX);
        background_task_cap_alt.wait();
      }
      background_task_cap_main.wait();
      overlayRGBA32OnRGB24(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight);
      writeFrameToFramebuffer(devInfoMain->outputFrame, fbDevName);
    }
    frame_number++;
  }
  cleanup_vars();
  return 0;
}