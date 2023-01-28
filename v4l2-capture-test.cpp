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

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define PORT 8888
#define MAX_CLIENTS 1

std::vector<int> client_sockets; // stores connected client sockets
std::atomic<bool> shouldLoop;
std::future<int> background_task;
int server_socket, client_socket;
struct sockaddr_in server_address;
struct sockaddr_in client_address;
socklen_t client_len = sizeof(client_address);

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
  unsigned char *outputFrame,
    *outputFrameGreyscaleScaled,
    *outputFrameGreyscaleUnscaled;
  char *device;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  // CHANGEME:
  int index;
};

struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;

int fdOut, ret = 1, retSize = 1, frame_number = 0;
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

void replace_pixels_below_val(const unsigned char* input, unsigned char* output, int width, int height, const int val) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Get the pixel value at the current position
      unsigned char pixel = input[y * width + x];
      // If the pixel value is below 63, replace it with a modified value
      if (pixel < val) {
        output[y * width + x] = (unsigned char)sqrt(output[y * width + x]);
        //output[y * width + x] = 0;
      }
      else {
        // Otherwise, copy the pixel value from the input to the output
        output[y * width + x] = pixel;
      }
    }
  }
}

void replace_pixels_above_val(const unsigned char* input, unsigned char* output, int width, int height, const int val) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Get the pixel value at the current position
      unsigned char pixel = input[y * width + x];
      // If the pixel value is below 63, replace it with a modified value
      if (pixel > val) {
        output[y * width + x] = 255;
        //output[y * width + x] = (unsigned char)sqrt(output[y * width + x]);
      }
      else {
        // Otherwise, copy the pixel value from the input to the output
        output[y * width + x] = pixel;
      }
    }
  }
}

void greyscale_to_sobel(const unsigned char* input, unsigned char* output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  // The maximum value for the Sobel operator
  //const int maxSobel = 4 * 255;
  // The Sobel operator as a 3x3 matrix
  const int sobelX[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
  const int sobelY[3][3] = { {-1, -2, -1}, {0, 0, 0}, {1, 2, 1} };
  // Iterate over each pixel in the image
#pragma omp parallel for simd
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      // Apply the Sobel kernel in the x and y directions
      int gx = sobelX[0][0] * input[(y - 1) * width + x - 1] + sobelX[0][1] * input[(y - 1) * width + x] + sobelX[0][2] * input[(y - 1) * width + x + 1] + sobelX[1][0] * input[y * width + x - 1] + sobelX[1][1] * input[y * width + x] + sobelX[1][2] * input[y * width + x + 1] + sobelX[2][0] * input[(y + 1) * width + x - 1] + sobelX[2][1] * input[(y + 1) * width + x] + sobelX[2][2] * input[(y + 1) * width + x + 1];
      int gy = sobelY[0][0] * input[(y - 1) * width + x - 1] + sobelY[0][1] * input[(y - 1) * width + x] + sobelY[0][2] * input[(y - 1) * width + x + 1] + sobelY[1][0] * input[y * width + x - 1] + sobelY[1][1] * input[y * width + x] + sobelY[1][2] * input[y * width + x + 1] + sobelY[2][0] * input[(y + 1) * width + x - 1] + sobelY[2][1] * input[(y + 1) * width + x] + sobelY[2][2] * input[(y + 1) * width + x + 1];
      // Compute the magnitude of the gradient at this pixel
      output[y * width + x] = (unsigned char)sqrt(gx * gx + gy * gy);
    }
  }
}

void uyvy_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 2) {
      int offset = (y * width + x) * 2;
      output[offset] = input[offset + 1];
      output[offset + 1] = input[offset];
      output[offset + 2] = input[offset + 3];
      output[offset + 3] = input[offset + 2];
    }
  }
}

void yuyv_to_uyvy(unsigned char* input, unsigned char* output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  // Iterate over each pixel in the input image
#pragma omp parallel for simd
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Extract the Y1, U, Y2, and V components from the YUYV pixel
      unsigned char y1 = input[offset];
      unsigned char u = input[offset + 1];
      unsigned char y2 = input[offset + 2];
      unsigned char v = input[offset + 3];
      // Pack the Y1 and U/V components into a UYVY pixel
      output[offset] = y1;
      output[offset + 1] = u;
      output[offset + 2] = y2;
      output[offset + 3] = v;
    }
  }
}

void grey_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  for (int i = 0, j = 0; i < width * height; i++, j += 2) {
    output[j] = input[i];  // Y
    output[j + 1] = 128; // U and V
  }
}

void rescale_bilinear(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd
  for (int y = 0; y < output_height; y++) {
    for (int x = 0; x < output_width; x++) {
      // Calculate the corresponding pixel coordinates in the input image.
      float in_x = x * input_width / output_width;
      float in_y = y * input_height / output_height;
      // Calculate the integer and fractional parts of the coordinates.
      int x1 = (int)in_x;
      int y1 = (int)in_y;
      float dx = in_x - x1;
      float dy = in_y - y1;
      // Clamp the coordinates to the edges of the input image.
      x1 = x1 < 0 ? 0 : x1;
      x1 = x1 >= input_width ? input_width - 1 : x1;
      y1 = y1 < 0 ? 0 : y1;
      y1 = y1 >= input_height ? input_height - 1 : y1;
      int x2 = x1 + 1;
      int y2 = y1 + 1;
      x2 = x2 < 0 ? 0 : x2;
      x2 = x2 >= input_width ? input_width - 1 : x2;
      y2 = y2 < 0 ? 0 : y2;
      y2 = y2 >= input_height ? input_height - 1 : y2;
      // Get the values of the four surrounding pixels in the input image.
      int index = y * output_width + x;
      int in_index1 = y1 * input_width + x1;
      int in_index2 = y1 * input_width + x2;
      int in_index3 = y2 * input_width + x1;
      int in_index4 = y2 * input_width + x2;
      float v1 = input[in_index1];
      float v2 = input[in_index2];
      float v3 = input[in_index3];
      float v4 = input[in_index4];
      // Use bilinear interpolation to estimate the value of the pixel in the input image.
      float value = (1 - dx) * (1 - dy) * v1 + dx * (1 - dy) * v2 + (1 - dx) * dy * v3 + dx * dy * v4;
      output[index] = (unsigned char)value;
    }
  }
}

/*void rescale_bilinear_from_yuyv(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
#pragma omp parallel for simd num_threads(4)
  for (int y = 0; y < output_height; y++) {
    for (int x = 0; x < output_width; x++) {
      // Calculate the corresponding pixel coordinates in the input image.
      float in_x = x * input_width / output_width;
      float in_y = y * input_height / output_height;
      // Calculate the integer and fractional parts of the coordinates.
      int x1 = (int)in_x;
      int y1 = (int)in_y;
      float dx = in_x - x1;
      float dy = in_y - y1;
      // Clamp the coordinates to the edges of the input image.
      x1 = std::clamp(x1, 0, input_width - 1);
      y1 = std::clamp(y1, 0, input_height - 1);
      int x2 = std::clamp(x1 + 1, 0, input_width - 1);
      int y2 = std::clamp(y1 + 1, 0, input_height - 1);
      // Get the luminance values of the four surrounding pixels in the input image.
      int index = y * output_width + x;
      int in_index1 = 2 * (y1 * input_width + x1);
      int in_index2 = 2 * (y1 * input_width + x2);
      int in_index3 = 2 * (y2 * input_width + x1);
      int in_index4 = 2 * (y2 * input_width + x2);
      float v1 = input[in_index1];
      float v2 = input[in_index2];
      float v3 = input[in_index3];
      float v4 = input[in_index4];
      // Use bilinear interpolation to estimate the luminance value of the pixel in the input image.
      float value = (1 - dx) * (1 - dy) * v1 + dx * (1 - dy) * v2 + (1 - dx) * dy * v3 + dx * dy * v4;
      if (value > 126 && value < 131) {
        value = 255.0F;
      } else {
        value = 0.0F;
      }
      //output[index] = (unsigned char)value;
      output[index] = (unsigned char)value;
    }
  }
}*/
void rescale_bilinear_from_yuyv(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd num_threads(4)
  for (int y = 0; y < output_height; y++) {
    for (int x = 0; x < output_width; x++) {
      // Calculate the corresponding pixel coordinates in the input image.
      float in_x = x * input_width / output_width;
      float in_y = y * input_height / output_height;
      // Calculate the integer and fractional parts of the coordinates.
      int x1 = (int)in_x;
      int y1 = (int)in_y;
      float dx = in_x - x1;
      float dy = in_y - y1;
      // Clamp the coordinates to the edges of the input image.
      x1 = x1 < 0 ? 0 : x1;
      x1 = x1 >= input_width ? input_width - 1 : x1;
      y1 = y1 < 0 ? 0 : y1;
      y1 = y1 >= input_height ? input_height - 1 : y1;
      int x2 = x1 + 1;
      int y2 = y1 + 1;
      x2 = x2 < 0 ? 0 : x2;
      x2 = x2 >= input_width ? input_width - 1 : x2;
      y2 = y2 < 0 ? 0 : y2;
      y2 = y2 >= input_height ? input_height - 1 : y2;
      // Get the luminance values of the four surrounding pixels in the input image.
      int index = y * output_width + x;
      int in_index1 = 2 * (y1 * input_width + x1);
      int in_index2 = 2 * (y1 * input_width + x2);
      int in_index3 = 2 * (y2 * input_width + x1);
      int in_index4 = 2 * (y2 * input_width + x2);
      float v1 = input[in_index1];
      float v2 = input[in_index2];
      float v3 = input[in_index3];
      float v4 = input[in_index4];
      // Use bilinear interpolation to estimate the luminance value of the pixel in the input image.
      float value = (1 - dx) * (1 - dy) * v1 + dx * (1 - dy) * v2 + (1 - dx) * dy * v3 + dx * dy * v4;
      /*if (value > 126 && value < 131) {
        value = 255.0F;
      } else {
        value = 0.0F;
      }*/
      //output[index] = (unsigned char)value;
      output[index] = (unsigned char)value;
    }
  }
}

std::vector<double> computeGaussianKernel(int kernelSize, double sigma) {
  std::vector<double> kernel(kernelSize);
  double sum = 0.0;
#pragma omp simd reduction(+:sum)
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] = exp(-0.5 * pow(i / sigma, 2.0)) / (sqrt(2.0 * M_PI) * sigma);
    sum += kernel[i];
  }
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] /= sum;
  }
  return kernel;
}

void gaussian_blur(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  std::vector<double> kernel = computeGaussianKernel(KERNEL_SIZE, SIGMA);
  // Perform the blur in the horizontal direction
#pragma omp parallel for simd num_threads(4)
  for (int y = 0; y < inputHeight; y++) {
    for (int x = 0; x < inputWidth; x++) {
      double sumY = 0.0;
      for (int i = -KERNEL_SIZE / 2; i <= KERNEL_SIZE / 2; i++) {
        int xi = x + i;
        // Handle edge cases by clamping the values
        if (xi < 0) xi = 0;
        if (xi >= inputWidth) xi = inputWidth - 1;
        sumY += kernel[i + KERNEL_SIZE / 2] * input[y * inputWidth + xi];
      }
      output[y * outputWidth + x] = sumY;
    }
  }
  // Perform the blur in the vertical direction
#pragma omp parallel for simd num_threads(4)
  for (int x = 0; x < inputWidth; x++) {
    for (int y = 0; y < inputHeight; y++) {
      double sumY = 0.0;
      for (int i = -KERNEL_SIZE / 2; i <= KERNEL_SIZE / 2; i++) {
        int yi = y + i;
        // Handle edge cases by clamping the values
        if (yi < 0) yi = 0;
        if (yi >= inputHeight) yi = inputHeight - 1;
        sumY += kernel[i + KERNEL_SIZE / 2] * output[yi * outputWidth + x];
      }
      output[y * outputWidth + x] = sumY;
    }
  }
}

void invert_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
#pragma omp parallel for simd num_threads(4)
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}

void crop_greyscale(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage, struct devInfo* devInfos) {
  if (image == nullptr || croppedImage == nullptr || crops == nullptr) {
    fprintf(stderr, "Fatal: Input or output or crops for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  if (devInfos->croppedWidth > 0 || devInfos->croppedHeight > 0) {
    devInfos->croppedWidth = devInfos->croppedWidth - crops[0] - crops[1];
    devInfos->croppedHeight = devInfos->croppedHeight - crops[2] - crops[3];
  } else {
    devInfos->croppedWidth = width - crops[0] - crops[1];
    devInfos->croppedHeight = height - crops[2] - crops[3];
  }
#pragma omp parallel for
  for (int y = crops[2]; y < crops[2] + devInfos->croppedHeight; y++) {
    for (int x = crops[0]; x < crops[0] + devInfos->croppedWidth; x++) {
      int croppedX = x - crops[0];
      int croppedY = y - crops[2];
      int croppedIndex = croppedY * devInfos->croppedWidth + croppedX;
      int index = y * width + x;
      croppedImage[croppedIndex] = image[index];
    }
  }
}

#ifdef USECUDA
__global__ void yuyv_to_greyscale(unsigned char* input, int width, int height, unsigned char* output, struct devInfo* devInfos) {
  int i = threadIdx.x;
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for CUDA operation is NULL..\nExiting now.\n");
    exit(1);
  }
  if (i < width * height) {
    output[i] = (input[i * 2] + input[i * 2 + 1]) / 2;
  }
  /*if (input == nullptr || output == nullptr) {
    return;
  }
  int len = width * height * 2;
  for (int i = 0; i < len; i += 2) {
    output[i / 2] = (input[i] + input[i + 1]) / 2;
  }*/
}
#else
void yuyv_to_greyscale(unsigned char* input, int width, int height, unsigned char* output, struct devInfo* devInfos) {
  if (input == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  int len = width * height * 2;
#pragma omp parallel for
  for (int i = 0; i < len; i += 2) {
    output[i / 2] = (input[i] + input[i + 1]) / 2;
  }
}
#endif

void frame_to_stdout(unsigned char* input, int size) {
  if (input == nullptr) {
    fprintf(stderr, "Fatal: Input for operation is NULL..\nExiting now.\n");
    exit(1);
  }
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

int init_dev_stage1(struct buffer* buffers, struct devInfo* devInfos) {
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
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 2) {
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 1) {
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
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

int init_dev_stage2(struct buffer* buffers, struct devInfo* devInfos) {
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

void convert_yuyv_to_yuv(unsigned char* yuyv_frame, unsigned char* yuv_frame, int width, int height) {
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

int get_frame(struct buffer *buffers, struct devInfo *devInfos, captureType capType) {
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
  //gaussian_blur(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
#ifdef USECUDA
  unsigned char* d_input;
  unsigned char* d_output;
  cudaMalloc(&d_input, devInfos->startingWidth * devInfos->startingHeight * 2 * sizeof(unsigned char));
  cudaMalloc(&d_output, devInfos->startingWidth * devInfos->startingHeight * sizeof(unsigned char));
  cudaMemcpy(d_input, (unsigned char*)buffers[buf.index].start, devInfos->startingWidth * devInfos->startingHeight * 2 * sizeof(unsigned char), cudaMemcpyHostToDevice);
  yuyv_to_greyscale << <1, (devInfos->scaledOutWidth * devInfos->scaledOutHeight) >> > (d_input, devInfos->scaledOutWidth, devInfos->scaledOutHeight, d_output, devInfoMain);
  cudaMemcpy(devInfos->outputFrameGreyscaleScaled, d_output, devInfos->scaledOutWidth * devInfos->scaledOutHeight * sizeof(unsigned char), cudaMemcpyDeviceToHost);
  cudaFree(d_input);
  cudaFree(d_output);
  frame_to_stdout(devInfos->outputFrameGreyscaleScaled, (devInfos->scaledOutWidth * devInfos->scaledOutHeight));
#else
  if (devInfos->startingWidth != devInfos->scaledOutWidth || devInfos->startingHeight != devInfos->scaledOutHeight) {
    yuyv_to_greyscale((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleUnscaled, devInfos);
    rescale_bilinear(devInfos->outputFrameGreyscaleUnscaled, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleScaled, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    //rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleScaled, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
  } else {
    yuyv_to_greyscale((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleUnscaled, devInfos);
  }
  /*if (write(fdOut, (unsigned char*)buffers[buf.index].start, (devInfos->startingWidth * devInfos->startingHeight * 2)) < 0) {
    fprintf(stderr, "Error writing frame\n");
  }*/
  //frame_to_stdout(devInfos->outputFrame, (devInfos->startingWidth * devInfos->startingHeight * 2));
#endif
  /*rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleScaled, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
  frame_to_stdout(devInfos->outputFrameGreyscaleScaled, (devInfos->scaledOutWidth * devInfos->scaledOutHeight));
  gaussian_blur(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);*/
  //crop_greyscale(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, cropMatrix[0], devInfos->outputFrameGreyscaleScaled, devInfos);
  //rescale_bilinear(devInfos->outputFrameGreyscaleScaled, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscaleScaled, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
  /*if (devInfos->frame_number % devInfos->framerateDivisor == 0) {
    rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    gaussian_blur(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    invert_greyscale(devInfos->outputFrameGreyscale, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    frame_to_stdout(devInfos->outputFrameGreyscale, (devInfos->scaledOutWidth * devInfos->scaledOutHeight));
  }
  devInfos->frame_number++;*/
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

int deinit_bufs(struct buffer *buffers, struct devInfo *devInfos) {
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

void did_memory_allocate_correctly(struct devInfo* devInfos) {
  if (devInfos->outputFrameGreyscaleScaled == NULL || devInfos->outputFrame == NULL || finalOutputFrame == NULL) {
    fprintf(stderr, "Fatal: Memory allocation failed for output frames..\nExiting now.\n");
    exit(1);
  }
}

int init_vars(struct devInfo*& devInfos, struct buffer*& bufs, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera, char *dev_name, int index) {
  devInfos->device = (char*)calloc(64, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->frame_number = 0,
    devInfos->framerate = 30,
    devInfos->framerateDivisor = 1,
    devInfos->startingWidth = 1280,
    devInfos->startingHeight = 720,
    devInfos->startingSize = (devInfos->startingWidth * devInfos->startingHeight * 2),
    devInfos->scaledOutSize = (scaledOutWidth * scaledOutHeight),
    devInfos->force_format = force_format,
    devInfos->scaledOutWidth = scaledOutWidth,
    devInfos->scaledOutHeight = scaledOutHeight,
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
  devInfos->outputFrameGreyscaleScaled = (unsigned char*)calloc((devInfos->scaledOutWidth * devInfos->scaledOutHeight), sizeof(unsigned char));
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * 2), sizeof(unsigned char));
  // allocate memory for combining frame buffers
  if (check_if_scaling(devInfos)) {
    devInfos->outputFrameGreyscaleUnscaled = (unsigned char*)calloc((devInfos->scaledOutWidth * devInfos->scaledOutHeight), sizeof(unsigned char));
    finalOutputFrameGreyscale = (unsigned char*)calloc((devInfos->scaledOutWidth * devInfos->scaledOutHeight * 2), sizeof(unsigned char));
    finalOutputFrame = (unsigned char*)calloc((devInfos->scaledOutWidth * devInfos->scaledOutHeight * 2 * 2), sizeof(unsigned char));
    did_memory_allocate_correctly(devInfos);
  } else {
    devInfos->outputFrameGreyscaleUnscaled = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight), sizeof(unsigned char));
    finalOutputFrameGreyscale = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * 2), sizeof(unsigned char));
    finalOutputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * 2 * 2), sizeof(unsigned char));
    did_memory_allocate_correctly(devInfos);
  }
  if (check_if_scaling(devInfos)) {
    retSize = (devInfos->scaledOutWidth * devInfos->scaledOutHeight * 2 * 2);
  } else {
    retSize = (devInfos->startingWidth * devInfos->startingHeight * 2 * 2);
  }
  return 0;
}

void combine_multiple_frames(unsigned char* input, unsigned char* inputAlt, unsigned char* output, int width, int height) {
  /*if (input == nullptr || inputAlt == nullptr || output == nullptr) {
    fprintf(stderr, "Fatal: Input or inputAlt or output for operation is NULL..\nExiting now.\n");
    exit(1);
  }*/
  int size = width * height;
  std::memcpy(output, input, size);
  std::memcpy(output + size, inputAlt, size);
}

void commandline_usage(int argcnt, char** args) {
  if (argcnt != 6) {
    fprintf(stderr, "[main] Usage: %s <V4L2 main device> <V4L2 alt device> <V4L2 out device> <scaled down width> <scaled down height>\n\nExample: %s /dev/video0 /dev/video1 /dev/video2 640 360\n", args[0], args[0]);
    exit(1);
  }
}

void init_output_v4l2_dev(struct devInfo* devInfos, const char* outDevName) {
  fdOut = v4l2_open(outDevName, O_RDWR, 0);
  if (fdOut < 0) {
    fprintf(stderr, "[out] Error opening video device\n");
  }
  memset(&fmtOut, 0, sizeof(fmtOut));
  fmtOut.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (check_if_scaling(devInfos)) {
    fmtOut.fmt.pix.width = devInfos->scaledOutWidth;
    fmtOut.fmt.pix.height = devInfos->scaledOutHeight * 2;
  } else {
    fmtOut.fmt.pix.width = devInfos->startingWidth;
    fmtOut.fmt.pix.height = devInfos->startingHeight * 2;
  }
  //switch (devInfos->force_format) {
  switch (1) {
  case 3:
    fmtOut.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmtOut.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    break;
  case 2:
    fmtOut.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmtOut.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    break;
  case 1:
    fmtOut.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmtOut.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    break;
  }
  if (xioctl(fdOut, VIDIOC_S_FMT, &fmtOut) < 0) {
    fprintf(stderr, "[out] Error setting format\n");
  }
  if (check_if_scaling(devInfos)) {
    fprintf(stderr, "[out] Output resolution for %s: %dx%d\n", outDevName, devInfos->scaledOutWidth, (devInfos->scaledOutHeight * 2));
  } else {
    fprintf(stderr, "[out] Output resolution for %s: %dx%d\n", outDevName, devInfos->startingWidth, (devInfos->startingHeight * 2));
  }
}

int net_sender(int retSz, int clisock, unsigned char* outData) {
  int retVal = send(clisock, outData, retSz, MSG_NOSIGNAL);
  if (retVal != retSz) {
    // if net_sender() in the background can't send the data (or enough data) to the client we will assume the connection is severed and set shallLoop to false
    shouldLoop.store(false);
    return 1;
  }
  return 0;
}

int main(int argc, char **argv) {
  commandline_usage(argc, argv);
  fprintf(stderr, "[main] Initializing..\n");
  // networking layer
  server_socket = socket(AF_INET, SOCK_STREAM, 0);
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(PORT);
  server_address.sin_addr.s_addr = INADDR_ANY;
  // tell the system that it can reuse the socket address
  setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &server_address, sizeof(server_address));
  bind(server_socket, (struct sockaddr*)&server_address, sizeof(server_address));
  listen(server_socket, MAX_CLIENTS);
  // allocate memory for structs
  devInfoMain = (devInfo*)calloc(1, sizeof(*devInfoMain));
  devInfoAlt = (devInfo*)calloc(1, sizeof(*devInfoAlt));
  init_vars(devInfoMain, buffersMain, 2, atoi(argv[4]), atoi(argv[5]), 10, true, true, argv[1], 0);
  init_vars(devInfoAlt, buffersAlt, 2, atoi(argv[4]), atoi(argv[5]), 10, true, true, argv[2], 1);
  init_output_v4l2_dev(devInfoMain, argv[3]);
  // start [main] loop
  while (true) {
    fprintf(stderr, "\n[main] Listening for client on TCP port 8888\n");
    // wait for a network client so we're not spinning our wheels maxing out the CPU when there's no client to view the data
    client_socket = accept(server_socket, (struct sockaddr*)&client_address, &client_len);
    // we've got a client. let's proceed..
    fprintf(stderr, "[net] Packet size: %d\n", retSize);
    sleep(1);
    fprintf(stderr, "\n[main] Starting main loop now\n");
    // shouldLoop allows the while loop below to loop while we have a client listening (explained further later below)
    shouldLoop.store(true);
    while (shouldLoop) {
      get_frame(buffersMain, devInfoMain, CHEAP_CONVERTER_BOX);
      get_frame(buffersAlt, devInfoAlt, CHEAP_CONVERTER_BOX);
      if (check_if_scaling(devInfoMain)) {
        invert_greyscale(devInfoAlt->outputFrameGreyscaleScaled, devInfoAlt->outputFrameGreyscaleScaled, devInfoAlt->scaledOutWidth, devInfoAlt->scaledOutHeight);
        combine_multiple_frames(devInfoMain->outputFrameGreyscaleScaled, devInfoAlt->outputFrameGreyscaleScaled, finalOutputFrameGreyscale, devInfoMain->scaledOutWidth, devInfoMain->scaledOutHeight);
        grey_to_yuyv(finalOutputFrameGreyscale, finalOutputFrame, devInfoMain->scaledOutWidth, (devInfoMain->scaledOutHeight * 2));
        if (write(fdOut, finalOutputFrame, (devInfoMain->scaledOutWidth * devInfoMain->scaledOutHeight * 2 * 2)) < 0)
          fprintf(stderr, "[main] Error writing frame to: %s\n", argv[3]);
        if (frame_number % 2 == 0)
          background_task = std::async(std::launch::async, net_sender, retSize, client_socket, finalOutputFrame);
        frame_number++;
      } else {
        invert_greyscale(devInfoAlt->outputFrameGreyscaleUnscaled, devInfoAlt->outputFrameGreyscaleUnscaled, devInfoAlt->startingWidth, devInfoAlt->startingHeight);
        combine_multiple_frames(devInfoMain->outputFrameGreyscaleUnscaled, devInfoAlt->outputFrameGreyscaleUnscaled, finalOutputFrameGreyscale, devInfoMain->startingWidth, devInfoMain->startingHeight);
        grey_to_yuyv(finalOutputFrameGreyscale, finalOutputFrame, devInfoMain->startingWidth, (devInfoMain->startingHeight * 2));
        if (write(fdOut, finalOutputFrame, (devInfoMain->startingWidth * devInfoMain->startingHeight * 2 * 2)) < 0)
          fprintf(stderr, "[main] Error writing frame to: %s\n", argv[3]);
        if (frame_number % 2 == 0)
          background_task = std::async(std::launch::async, net_sender, retSize, client_socket, finalOutputFrame);
        frame_number++;
      }
    }
  }
  deinit_bufs(buffersMain, devInfoMain);
  deinit_bufs(buffersAlt, devInfoAlt);
  close(fdOut);
  return 0;
}