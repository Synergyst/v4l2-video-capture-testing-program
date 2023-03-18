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
//#define USECUDA
#ifdef USECUDA
#include "cuda.h"
#include "cuda_runtime.h"
#include "helper_cuda.h"
#include "cuda_device_runtime_api.h"
#include "cuda_runtime_api.h"
#include "device_launch_parameters.h"
#endif
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
#include <libswscale/swscale.h>
#include <libyuv.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixfmt.h>
#include <libavutil/imgutils.h>
#include <span>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <future>
#include <linux/fb.h>
#include <execution>
#include <numeric>

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define PORT 8888
#define MAX_CLIENTS 1

std::vector<int> client_sockets; // stores connected client sockets
std::atomic<bool> shouldLoop;
std::future<int> background_task_net;
std::future<int> background_task_cap_main;
std::future<int> background_task_cap_alt;
int server_socket, client_socket;
struct sockaddr_in server_address;
struct sockaddr_in client_address;
socklen_t client_len = sizeof(client_address);

int strSfd, strSock, strSopt = 1;;
struct sockaddr_in strSaddr;
int strSaddrlen = sizeof(strSaddr);

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
    scaledOutSize,
    force_format,
    scaledOutWidth,
    scaledOutHeight,
    targetFramerate,
    fd,
    croppedWidth,
    croppedHeight;
  unsigned int n_buffers;
  double frameDelayMicros,
    frameDelayMillis;
  bool isTC358743 = true,
    isThermalCamera = true;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  int index;
  unsigned char *outputFrame,
    *outputFrameGreyscaleScaled,
    *outputFrameGreyscaleUnscaled;
  char* device;
};

struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;

int /*fdOut,*/ ret = 1, retSize = 1, frame_number = 0;
struct v4l2_format fmtOut;
unsigned char* finalOutputFrame;
unsigned char* finalOutputFrameGreyscale;
// Crop size matrix (scale up or down as needed)
int cropMatrix[2][4] = {
  {11, 4, 4, 2},
  {1, 1, 1, 1}
};
const int KERNEL_SIZE = 3; // The kernel size of the Gaussian blur, default: 5
const double SIGMA = 2.0; // The sigma value of the Gaussian blur, default: 2.0
int byteScaler = 0;
const int num_threads = 4;

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
#pragma omp parallel for simd num_threads(4)
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}

void frame_to_stdout(unsigned char*& input, int size) {
  if (input == nullptr) {
    fprintf(stderr, "Fatal: Input for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

int init_dev_stage1(struct buffer*& buffers, struct devInfo*& devInfos) {
  //unsigned int i;
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
      byteScaler = 2;
      //byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GRAY;
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
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * sizeof(unsigned char);
      devInfos->scaledOutSize = devInfos->scaledOutWidth * devInfos->scaledOutHeight;
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
        int rawOutputThroughput = (float)((((float)devInfos->framerate / devInfos->framerateDivisor) * devInfos->scaledOutSize) / 125000.0F); // Measured in megabits/sec based on output framerate
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, isThermalCamera: %d, startingWidth: %d, startingHeight: %d, startingSize: %d, scaledOutWidth: %d, scaledOutHeight: %d, scaledOutSize: %d, framerate: %u, framerateDivisor: %d, targetFramerate: %d, rawInputThroughput: ~%dMb/sec, rawOutputThroughput: ~%dMb/sec\n", devInfos->index, devInfos->device, devInfos->isTC358743, devInfos->isThermalCamera, devInfos->startingWidth, devInfos->startingHeight, devInfos->startingSize, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->scaledOutSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, rawInputThroughput, rawOutputThroughput);
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
    fprintf(stderr, "Fatal: Only the TC358743 is supported for now. Support for general camera inputs will need to be added in the future..\nExiting now.\n");
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

void convert_yuyv_to_yuv(unsigned char*& yuyv_frame, unsigned char*& yuv_frame, int width, int height) {
  int yuyv_idx, yuv_idx;
#pragma omp parallel for
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j += 2) {
      yuyv_idx = (i * width + j) * 2;
      yuv_idx = (i * width + j) * 3;
      yuv_frame[yuv_idx] = yuyv_frame[yuyv_idx];
      yuv_frame[yuv_idx + 1] = yuyv_frame[yuyv_idx + 1];
      yuv_frame[yuv_idx + 2] = yuyv_frame[yuyv_idx + 3];
    }
  }
}

int get_frame(struct buffer* buffers, struct devInfo* devInfos, captureType capType) {
  //memset(devInfos->outputFrameGreyscale, 0, devInfos->startingSize);
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
  //frame_to_stdout((unsigned char*)buffers[buf.index].start, (devInfos->startingWidth * devInfos->startingHeight * 2));
  //rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleScaled, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
  //frame_to_stdout(devInfos->outputFrameGreyscaleScaled, (devInfos->scaledOutWidth * devInfos->scaledOutHeight));

  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, 1280 * 720 * 3);

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
  // We should be using DMA (Direct-Memory-Access), so we shouldn't have much to cleanup
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

bool check_if_scaling(struct devInfo* devInfos) {
  return (devInfos->startingWidth != devInfos->scaledOutWidth || devInfos->startingHeight != devInfos->scaledOutHeight);
}

void did_memory_allocate_correctly(struct devInfo*& devInfos) {
  if (devInfos->outputFrameGreyscaleScaled == NULL || devInfos->outputFrame == NULL || finalOutputFrame == NULL) {
    fprintf(stderr, "Fatal: Memory allocation failed for output frames..\nExiting now.\n");
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
    devInfos->startingWidth = 1280,
    devInfos->startingHeight = 720,
    devInfos->startingSize = (devInfos->startingWidth * devInfos->startingHeight * byteScaler),
    /*devInfos->scaledOutSize = (scaledOutWidth * scaledOutHeight),*/
    devInfos->force_format = force_format,
    /*devInfos->scaledOutWidth = scaledOutWidth,
    devInfos->scaledOutHeight = scaledOutHeight,*/
    devInfos->targetFramerate = targetFramerate,
    devInfos->fd = -1,
    devInfos->isTC358743 = isTC358743,
    devInfos->isThermalCamera = isThermalCamera,
    devInfos->index = index,
    devInfos->croppedWidth = 0,
    devInfos->croppedHeight = 0;
  init_dev_stage1(buffersMain, devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  // allocate memory for commonly used frame buffers
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * byteScaler), sizeof(unsigned char));
  // allocate memory for combining frame buffers
  devInfos->outputFrameGreyscaleUnscaled = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight), sizeof(unsigned char));
  // Stereo-vision only, use outputFrame otherwise! :)
  finalOutputFrameGreyscale = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * 2), sizeof(unsigned char));
  finalOutputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * byteScaler * 2), sizeof(unsigned char));
  did_memory_allocate_correctly(devInfos);
  retSize = (devInfos->startingWidth * devInfos->startingHeight * byteScaler * 2);
  return 0;
}

void combine_multiple_frames(unsigned char*& input, unsigned char*& inputAlt, unsigned char*& output, int width, int height) {
  /*if (input == nullptr || inputAlt == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or inputAlt or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }*/
  int size = width * height;
  std::memcpy(output, input, size);
  std::memcpy(output + size, inputAlt, size);
}

void commandline_usage(const int argcnt, char** args) {
  /*if (argcnt != 6) {
    fprintf(stderr, "[main] Usage: %s <V4L2 main device> <V4L2 alt device> <V4L2 out device> <scaled down width> <scaled down height>\n\nExample: %s /dev/video0 /dev/video1 /dev/video2 640 360\n", args[0], args[0]);
    exit(1);
  }*/
  if (argcnt != 4) {
    fprintf(stderr, "[main] Usage: %s <V4L2 main device> <V4L2 alt device> </dev/fb out device>\n\nExample: %s /dev/video0 /dev/video1 /dev/fb0\n", args[0], args[0]);
    exit(1);
  }
}

void write_outputs(unsigned char*& output, char*& outDev, int width, int height) {
  /*if (write(fdOut, output, (width * height * byteScaler * 2)) < 0)
    fprintf(stderr, "[main] Error writing frame to: %s\n", outDev);*/
}

void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
  deinit_bufs(buffersAlt, devInfoAlt);
  //close(fdOut);
}

void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt, int argCnt, char **args) {
  commandline_usage(argCnt, args);
  fprintf(stderr, "[main] Initializing..\n");
  //init_net();
  // allocate memory for structs
  init_vars(deviMain, bufMain, 3, 10, true, true, args[1], 0);
  init_vars(deviAlt, bufAlt, 3, 10, true, true, args[2], 1);
  //init_output_v4l2_dev(deviMain, args[3]);
}

void writeFrameToFramebuffer(const unsigned char* frameData) {
  int fbfd = open("/dev/fb0", O_RDWR);
  if (fbfd == -1) {
    perror("Error: cannot open framebuffer device");
    exit(1);
  }
  struct fb_var_screeninfo vinfo;
  ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
  if (vinfo.bits_per_pixel != 16 || vinfo.xres != 1280 || vinfo.yres != 720) {
    perror("Error: framebuffer does not accept RGB24 frames with 1280x720 resolution");
    exit(1);
  }
  long int screensize = vinfo.xres * vinfo.yres * 2;
  /*char* fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if ((int)fbmem == -1) {
      perror("Error: failed to mmap framebuffer device to memory");
      exit(1);
  }*/
  char* fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  if (fbmem == MAP_FAILED) {
    perror("Error: failed to mmap framebuffer device to memory");
    exit(1);
  }
  char* currentPixel = fbmem;
  for (int y = 0; y < vinfo.yres; y++) {
    for (int x = 0; x < vinfo.xres; x++) {
      int pixelOffset = y * vinfo.xres * 3 + x * 3;
      unsigned char r = frameData[pixelOffset];
      unsigned char g = frameData[pixelOffset + 1];
      unsigned char b = frameData[pixelOffset + 2];
      unsigned short pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
      *(unsigned short*)currentPixel = pixel;
      currentPixel += 2;
    }
  }
  munmap(fbmem, screensize);
  close(fbfd);
}

/*void overlayRGBA32OnRGB24(unsigned char* rgb24, const unsigned char* rgba32, int width, int height, int numThreads = std::thread::hardware_concurrency()) {
  const int numPixels = width * height;
  const int numBytesRGB24 = numPixels * 3;
  const int numBytesRGBA32 = numPixels * 4;
  std::vector<std::thread> threads(numThreads);
  for (int t = 0; t < numThreads; ++t) {
    const int start = (t * numPixels) / numThreads;
    const int end = ((t + 1) * numPixels) / numThreads;
    threads[t] = std::thread([=] {
      for (int i = start; i < end; ++i) {
        const int r1 = rgb24[i * 3];
        const int g1 = rgb24[i * 3 + 1];
        const int b1 = rgb24[i * 3 + 2];
        const int r2 = rgba32[i * 4];
        const int g2 = rgba32[i * 4 + 1];
        const int b2 = rgba32[i * 4 + 2];
        const int alpha = rgba32[i * 4 + 3];
        if (alpha > 0) {
          const float alpha_ratio = alpha / 255.0f;
          const int r = static_cast<int>(r1 * (1 - alpha_ratio) + r2 * alpha_ratio);
          const int g = static_cast<int>(g1 * (1 - alpha_ratio) + g2 * alpha_ratio);
          const int b = static_cast<int>(b1 * (1 - alpha_ratio) + b2 * alpha_ratio);
          rgb24[i * 3] = static_cast<unsigned char>(r);
          rgb24[i * 3 + 1] = static_cast<unsigned char>(g);
          rgb24[i * 3 + 2] = static_cast<unsigned char>(b);
        }
      }
      });
  }
  for (auto& t : threads) {
    t.join();
  }
}*/

void overlayRGBA32OnRGB24(unsigned char* rgb24, const unsigned char* rgba32, int width, int height, int numThreads = std::thread::hardware_concurrency()) {
  const int numPixels = width * height;
  const int numBytesRGB24 = numPixels * 3;
  const int numBytesRGBA32 = numPixels * 4;
  // Create an array of futures to hold the results of each thread.
  std::vector<std::future<void>> futures(numThreads);
  for (int t = 0; t < numThreads; ++t) {
    const int start = (t * numPixels) / numThreads;
    const int end = ((t + 1) * numPixels) / numThreads;
    // Create a packaged task that takes the arguments for the loop and returns void.
    auto task = std::packaged_task<void()>([=] {
      for (int i = start; i < end; ++i) {
        const int r1 = rgb24[i * 3];
        const int g1 = rgb24[i * 3 + 1];
        const int b1 = rgb24[i * 3 + 2];
        const int r2 = rgba32[i * 4];
        const int g2 = rgba32[i * 4 + 1];
        const int b2 = rgba32[i * 4 + 2];
        const int alpha = rgba32[i * 4 + 3];
        if (alpha > 0) {
          const float alpha_ratio = alpha / 255.0f;
          const int r = static_cast<int>(r1 * (1 - alpha_ratio) + r2 * alpha_ratio);
          const int g = static_cast<int>(g1 * (1 - alpha_ratio) + g2 * alpha_ratio);
          const int b = static_cast<int>(b1 * (1 - alpha_ratio) + b2 * alpha_ratio);
          rgb24[i * 3] = static_cast<unsigned char>(r);
          rgb24[i * 3 + 1] = static_cast<unsigned char>(g);
          rgb24[i * 3 + 2] = static_cast<unsigned char>(b);
        }
      }
    });
    // Get a future from the packaged task and add it to the array.
    futures[t] = task.get_future();
    // Move the packaged task into a new thread and execute it.
    std::thread(std::move(task)).detach();
  }
  // Wait for all the threads to finish.
  for (auto& f : futures) {
    f.wait();
  }
}

// Allocate new array with alpha channel
unsigned char* outputWithAlpha = new unsigned char[1280 * 720 * 4];

unsigned char* add_alpha_channel(struct devInfo* deviAlt) {
  // Copy RGB values from deviAlt->outputFrame and set alpha to 128 using parallel algorithm
  std::vector<int> indices(1280 * 720);
  std::iota(indices.begin(), indices.end(), 0); // Fill vector with [0, 1280*720)
  std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i) {
    outputWithAlpha[i * 4] = deviAlt->outputFrame[i * 3];
    outputWithAlpha[i * 4 + 1] = deviAlt->outputFrame[i * 3 + 1];
    outputWithAlpha[i * 4 + 2] = deviAlt->outputFrame[i * 3 + 2];
    outputWithAlpha[i * 4 + 3] = 128;
    });
  return outputWithAlpha;
}

void process_frames(struct devInfo*& deviMain, struct devInfo*& deviAlt, unsigned char*& outputGreyscale, unsigned char*& output, char*& deviName) {
  // TODO: Append adjustable alpha channel to deviAlt->outputFrame (which is stored in an unsigned char array representing an RGB24 image frame with a resolution of 1280x720) on this very line! :)
  // Copy RGB values from deviAlt->outputFrame and set alpha to 128 (the commented section below is single-threaded)
  /*for (int i = 0; i < 1280 * 720; i++) {
    outputWithAlpha[i * 4] = deviAlt->outputFrame[i * 3];
    outputWithAlpha[i * 4 + 1] = deviAlt->outputFrame[i * 3 + 1];
    outputWithAlpha[i * 4 + 2] = deviAlt->outputFrame[i * 3 + 2];
    outputWithAlpha[i * 4 + 3] = 128;
  }*/
  // multi-threaded operation version
  const int region_width = 1280 / num_threads;
  std::vector<std::jthread> threads(num_threads);
  for (int i = 0; i < num_threads; i++) {
    const int region_start = i * region_width;
    const int region_end = (i == num_threads - 1) ? 1280 : (i + 1) * region_width;
    //threads[i] = std::jthread([&](std::stop_token token) {
    threads[i] = std::jthread([&]() {
      // Process region of the frame
      //overlayRGBA32OnRGB24(deviMain->outputFrame + region_start * 3, outputWithAlpha + region_start * 4, region_end - region_start, 720, token);
      overlayRGBA32OnRGB24(deviMain->outputFrame + region_start * 3, outputWithAlpha + region_start * 4, region_end - region_start, 720, num_threads);
    });
  }
  // Wait for all threads to finish before writing to framebuffer
  for (auto& thread : threads) {
    thread.join();
  }
  overlayRGBA32OnRGB24(deviMain->outputFrame, deviAlt->outputFrame, deviMain->startingWidth, deviMain->startingHeight, num_threads);
  writeFrameToFramebuffer(deviMain->outputFrame);
}

int main(const int argc, char **argv) {
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt, argc, argv);
  // start [main] loop
  while (true) {
    sleep(1);
    fprintf(stderr, "\n[main] Starting main loop now\n");
    // shouldLoop allows the while loop below to loop while we have a client listening (explained further later below)
    shouldLoop.store(true);
    while (shouldLoop) {
      if (frame_number % 2 == 0) {
        background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain, CHEAP_CONVERTER_BOX);
        background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt, CHEAP_CONVERTER_BOX);
        background_task_cap_main.wait();
        background_task_cap_alt.wait();
        process_frames(devInfoMain, devInfoAlt, finalOutputFrameGreyscale, finalOutputFrame, argv[3]);
      }
      frame_number++;
    }
  }
  // Free memory allocated for outputWithAlpha
  delete[] outputWithAlpha;
  cleanup_vars();
  return 0;
}