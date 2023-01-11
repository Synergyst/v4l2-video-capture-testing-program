#include <iostream>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>  /* getopt_long() */
#include <fcntl.h>   /* low-level i/o */
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
#include <dlfcn.h>

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
};

struct buffer {
  void* start;
  size_t length;
};

enum io_method io = IO_METHOD_MMAP;
struct buffer* buffers;
unsigned int n_buffers;

const int startingWidth = 1280, startingHeight = 720, scaledOutWidth = 640, scaledOutHeight = 360, targetFramerate = 15; // The width and height of the input video and any downscaled output video
int frame_number = 0, framerate = -1, framerateDivisor = 1;

// Allocate memory for the input and output frames
unsigned char* outputFrame = new unsigned char[startingWidth * startingHeight * 2];
unsigned char* outputFrameGreyscale = new unsigned char[startingWidth * startingHeight];

void (*rescale_bilinear_from_yuyv)(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height);
void (*gaussianBlur)(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight);
void (*frame_to_stdout)(unsigned char* input, int size);
void (*yuyv_to_greyscale)(const unsigned char* input, unsigned char* grey, int width, int height);
void (*uyvy_to_greyscale)(const unsigned char* input, unsigned char* grey, int width, int height);
void (*crop_greyscale)(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage);
void (*replace_pixels_below_val)(const unsigned char* input, unsigned char* output, int width, int height, const int val);
void (*replace_pixels_above_val)(const unsigned char* input, unsigned char* output, int width, int height, const int val);
void (*greyscale_to_sobel)(const unsigned char* input, unsigned char* output, int width, int height);
void (*uyvy_sobel)(unsigned char* input, unsigned char* output, int width, int height);
void (*uyvy_to_yuyv)(unsigned char* input, unsigned char* output, int width, int height);
void (*yuyv_to_uyvy)(unsigned char* input, unsigned char* output, int width, int height);
void (*rescale_bilinear)(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height);
std::vector<double> computeGaussianKernel(int kernelSize, double sigma);
void (*invert_greyscale)(unsigned char* input, unsigned char* output, int width, int height);

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

void init_read(unsigned int buffer_size) {
  buffers = (buffer*)calloc(1, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);
  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

void init_mmap(int fd, char* device_name) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support memory mapping\n", device_name);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", device_name);
    exit(EXIT_FAILURE);
  }
  buffers = (buffer*)calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);
    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

void init_userp(unsigned int buffer_size, int fd, char* device_name) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support user pointer i/o\n", device_name);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  buffers = (buffer*)calloc(4, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = malloc(buffer_size);
    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}