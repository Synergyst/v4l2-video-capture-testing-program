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

struct buffer {
  void* start;
  size_t length;
};

struct buffer* buffers;
unsigned int n_buffers;

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