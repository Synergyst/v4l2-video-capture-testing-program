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
//#include "v4l2capalt.h"

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

struct buffer {
    void* start;
    size_t length;
};

enum io_methodAlt {
  IO_METHOD_READALT,
  IO_METHOD_MMAPALT,
  IO_METHOD_USERPTRALT,
};

struct bufferAlt {
  void* start;
  size_t length;
};

char* dev_name;
char* dev_name_alt;
enum io_method io = IO_METHOD_MMAP;
enum io_methodAlt ioAlt = IO_METHOD_MMAPALT;
int fd = -1;
int fdAlt = -1;
struct buffer* buffers;
struct bufferAlt* buffersAlt;
unsigned int n_buffers;
unsigned int n_buffersAlt;
int out_buf;
int out_bufAlt;
// 1 = YUYV, 2 = UYVY, 3 = RGB24 - Do not use RGB24 as it will cause high CPU usage, YUYV/UYVY should be used instead
int force_format = 1;
int force_formatAlt = 1;
int frame_count = 0;
int frame_countAlt = 0;
int frame_number = 0;
int frame_numberAlt = 0;

void errno_exit(const char* s) {
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}
void errno_exitAlt(const char* s) {
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
int xioctlAlt(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}

// The width and height of the input video and any downscaled output video
const int startingWidth = 1280;
const int startingHeight = 720;
const int startingWidthAlt = 320;
const int startingHeightAlt = 180;
/*const int startingWidth = 1920;
const int startingHeight = 1080;
const int scaledOutWidth = 640;
const int scaledOutHeight = 360;*/
const int scaledOutWidth = 320;
const int scaledOutHeight = 180;
const int scaledOutWidthAlt = 320;
const int scaledOutHeightAlt = 180;
/*const int scaledOutWidth = 384;
const int scaledOutHeight = 216;*/
// Crop size matrix (scale up or down as needed)
int cropMatrix[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} };
int cropMatrixAlt[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} };
int croppedWidthAlt = 0;
int croppedHeightAlt = 0;
int croppedWidth = 0;
int croppedHeight = 0;

// The maximum value for the Sobel operator
static const int maxSobel = 4 * 255;
// The Sobel operator as a 3x3 matrix
static const int sobelX[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
static const int sobelY[3][3] = { {-1, -2, -1}, {0, 0, 0}, {1, 2, 1} };

// Allocate memory for the input and output frames
unsigned char* outputFrame = new unsigned char[startingWidth * startingHeight * 2];
unsigned char* outputFrame2 = new unsigned char[startingWidth * startingHeight * 2];
unsigned char* outputFrameRGB24 = new unsigned char[startingWidth * startingHeight * 3];
unsigned char* outputFrameGreyscale = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameGreyscale1 = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameGreyscale2 = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameGreyscale3 = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameGreyscale4 = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameGreyscale5 = new unsigned char[startingWidth * startingHeight];
unsigned char* redVals = new unsigned char[startingWidth * startingHeight];
unsigned char* greenVals = new unsigned char[startingWidth * startingHeight];
unsigned char* blueVals = new unsigned char[startingWidth * startingHeight];
unsigned char* outputFrameScaledR = new unsigned char[scaledOutWidth * scaledOutHeight];
unsigned char* outputFrameScaledG = new unsigned char[scaledOutWidth * scaledOutHeight];
unsigned char* outputFrameScaledB = new unsigned char[scaledOutWidth * scaledOutHeight];
unsigned char* outputFrameAlt = new unsigned char[startingWidthAlt * startingHeightAlt * 2];
unsigned char* outputFrame2Alt = new unsigned char[startingWidthAlt * startingHeightAlt * 2];
unsigned char* outputFrame3Alt = new unsigned char[startingWidthAlt * startingHeightAlt * 3];
unsigned char* outputFrameGreyscaleAlt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale1Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale2Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale3Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale4Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale5Alt = new unsigned char[startingWidthAlt * startingHeightAlt];

void yuyv_to_greyscale(const unsigned char* yuyv, unsigned char* grey, int width, int height) {
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      // YUYV format stores chroma (Cb and Cr) values interleaved with the luma (Y) values.
      // So, we need to skip every other pixel.
      int y_index = index * 2;
      grey[index] = yuyv[y_index];
    }
  }
}

void replace_pixels_below_val(const unsigned char* input, unsigned char* output, int width, int height, const int val) {
#pragma omp parallel for num_threads(4)
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
#pragma omp parallel for num_threads(4)
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

void uyvy_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Extract the Y component from the UYVY pixel
      output[y * width + x] = input[offset];
    }
  }
}

void greyscale_to_sobel(const unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the image
#pragma omp parallel for
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

void uyvy_sobel(unsigned char* input, unsigned char* output, int width, int height) {
  // Create buffers to hold the intermediate images
  unsigned char* greyscale = new unsigned char[width * height];
  short* gradient_x = new short[width * height];
  short* gradient_y = new short[width * height];
  // Convert the input frame to greyscale
  uyvy_to_greyscale(input, greyscale, width, height);
  // Iterate over each pixel in the greyscale image
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      // Calculate the gradient in the X and Y directions using Sobel filters
      gradient_x[y * width + x] = greyscale[(y - 1) * width + x - 1] + 2 * greyscale[y * width + x - 1] + greyscale[(y + 1) * width + x - 1] - greyscale[(y - 1) * width + x + 1] - 2 * greyscale[y * width + x + 1] - greyscale[(y + 1) * width + x + 1];
      gradient_y[y * width + x] = greyscale[(y - 1) * width + x - 1] + 2 * greyscale[(y - 1) * width + x] + greyscale[(y - 1) * width + x + 1] - greyscale[(y + 1) * width + x - 1] - 2 * greyscale[(y + 1) * width + x] - greyscale[(y + 1) * width + x + 1];
    }
  }
  // Iterate over each pixel in the output image
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the output buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Calculate the magnitude of the gradient using the X and Y gradients
      int gradient = abs(gradient_x[y * width + x]) + abs(gradient_y[y * width + x]);
      // Clamp the gradient to the range [0, 255]
      gradient = std::max(0, std::min(255, gradient));
      // Set the Y component of the output pixel to the gradient magnitude
      output[offset] = gradient;
      // Set the U and V components of the output pixel to 128 (neutral color)
      output[offset + 1] = 128;
    }
  }
  // Free the intermediate buffers
  delete[] greyscale;
  delete[] gradient_x;
  delete[] gradient_y;
}

void uyvy_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Extract the Y and U/V components from the UYVY pixel
      unsigned char y1 = input[offset];
      unsigned char u = input[offset + 1];
      unsigned char y2 = input[offset + 2];
      unsigned char v = input[offset + 3];
      // Pack the Y1, U, Y2, and V components into a YUYV pixel
      output[offset] = y1;
      output[offset + 1] = u;
      output[offset + 2] = y2;
      output[offset + 3] = v;
    }
  }
}

void yuyv_to_uyvy(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
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

/*void rgb24_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
//#pragma omp parallel for num_threads(4)
#pragma omp parallel for simd
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 3;
      // Extract the red, green, and blue components from the RGB pixel
      unsigned char r = input[offset];
      unsigned char g = input[offset + 1];
      unsigned char b = input[offset + 2];
      // Calculate the greyscale value using the formula:
      // greyscale = 0.299 * red + 0.587 * green + 0.114 * blue
      unsigned char greyscale = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      // Set the greyscale value as the intensity of the output pixel
      output[y * width + x] = greyscale;
    }
  }
}*/
void rgb24_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each row in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    // Get pointers to the beginning of the current row in the input and output arrays
    unsigned char* input_row = &input[y * width * 3];
    unsigned char* output_row = &output[y * width];
    // Iterate over each pixel in the current row
    for (int x = 0; x < width; x++) {
      // Extract the red, green, and blue components from the RGB pixel
      unsigned char r = input_row[x * 3];
      unsigned char g = input_row[x * 3 + 1];
      unsigned char b = input_row[x * 3 + 2];
      // Calculate the greyscale value using the formula:
      // greyscale = 0.299 * red + 0.587 * green + 0.114 * blue
      unsigned char greyscale = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      // Set the greyscale value as the intensity of the output pixel
      output_row[x] = greyscale;
    }
  }
}

void rgb24_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 3;
      // Extract the red, green, and blue components from the RGB pixel
      unsigned char r = input[offset];
      unsigned char g = input[offset + 1];
      unsigned char b = input[offset + 2];
      // Calculate the Y, U, and V components using the following formulas:
      // Y = 0.299 * red + 0.587 * green + 0.114 * blue
      // U = -0.147 * red - 0.289 * green + 0.436 * blue + 128
      // V = 0.615 * red - 0.515 * green - 0.100 * blue + 128
      unsigned char y = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      unsigned char u = static_cast<unsigned char>(-0.147 * r - 0.289 * g + 0.436 * b + 128);
      unsigned char v = static_cast<unsigned char>(0.615 * r - 0.515 * g - 0.100 * b + 128);
      // Pack the Y, U, and V components into a YUYV pixel
      output[offset] = y;
      output[offset + 1] = u;
      output[offset + 2] = y;
      output[offset + 3] = v;
    }
  }
}

void resize_image_nearest_neighbor(const uint8_t* src, int src_width, int src_height, uint8_t* dst, int dst_width, int dst_height) {
#pragma omp parallel for
  for (int y = 0; y < dst_height; ++y) {
    for (int x = 0; x < dst_width; ++x) {
      // Calculate the source image coordinates corresponding to the current
      // destination image coordinates
      float src_x = (x + 0.5f) * src_width / dst_width - 0.5f;
      float src_y = (y + 0.5f) * src_height / dst_height - 0.5f;
      // Round the source image coordinates to the nearest integer
      int src_x_int = std::round(src_x);
      int src_y_int = std::round(src_y);
      // Clamp the source image coordinates to the valid range
      src_x_int = std::max(0, std::min(src_x_int, src_width - 1));
      src_y_int = std::max(0, std::min(src_y_int, src_height - 1));
      // Copy the pixel value from the source image to the destination image
      dst[y * dst_width + x] = src[src_y_int * src_width + src_x_int];
    }
  }
}

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

void rescale_bilinear(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
#pragma omp parallel for num_threads(4)
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
#pragma omp critical
      output[index] = (unsigned char)value;
    }
  }
}
/*void rescale_bilinear(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
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
      x1 = std::clamp(x1, 0, input_width - 1);
      y1 = std::clamp(y1, 0, input_height - 1);
      int x2 = std::clamp(x1 + 1, 0, input_width - 1);
      int y2 = std::clamp(y1 + 1, 0, input_height - 1);
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
}*/

void rescale_bilinear_from_yuyv(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
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
}

const int KERNEL_SIZE = 7; // The kernel size of the Gaussian blur, default: 5
const double SIGMA = 2.0; // The sigma value of the Gaussian blur, default: 2.0

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
/*// A helper function to compute the Gaussian kernel
std::vector<double> computeGaussianKernel(int kernelSize, double sigma) {
  std::vector<double> kernel(kernelSize);
  double sum = 0.0;
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] = exp(-0.5 * pow(i / sigma, 2.0)) / (sqrt(2.0 * M_PI) * sigma);
    sum += kernel[i];
  }
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] /= sum;
  }
  return kernel;
}
// A helper function to compute the Gaussian kernel
double* computeGaussianKernel(int kernelSize, double sigma) {
  double* kernel = (double*)malloc(kernelSize * sizeof(double));
  double sum = 0.0;
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] = exp(-0.5 * pow(i / sigma, 2.0)) / (sqrt(2.0 * M_PI) * sigma);
    sum += kernel[i];
  }
  for (int i = 0; i < kernelSize; i++) {
    kernel[i] /= sum;
  }
  return kernel;
}*/

// The main function that performs the Gaussian blur
void gaussianBlur(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight) {
  std::vector<double> kernel = computeGaussianKernel(KERNEL_SIZE, SIGMA);
  // Perform the blur in the horizontal direction
#pragma omp parallel for
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
#pragma omp parallel for
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

void crop_greyscale_alt(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage) {
  if (croppedWidthAlt > 0 || croppedHeightAlt > 0) {
    croppedWidthAlt = croppedWidthAlt - crops[0] - crops[1];
    croppedHeightAlt = croppedHeightAlt - crops[2] - crops[3];
  }
  else {
    croppedWidthAlt = width - crops[0] - crops[1];
    croppedHeightAlt = height - crops[2] - crops[3];
  }
#pragma omp parallel for
  for (int y = crops[2]; y < crops[2] + croppedHeightAlt; y++) {
    for (int x = crops[0]; x < crops[0] + croppedWidthAlt; x++) {
      int croppedX = x - crops[0];
      int croppedY = y - crops[2];
      int croppedIndex = croppedY * croppedWidthAlt + croppedX;
      int index = y * width + x;
      croppedImage[croppedIndex] = image[index];
    }
  }
}

void separate_rgb24(unsigned char* rgb, unsigned char* red, unsigned char* green, unsigned char* blue, int width, int height) {
#pragma omp parallel for simd
  for (int i = 0; i < width * height * 3; i += 3) {
    //red[i / 3] = rgb[i];
    //green[i / 3] = rgb[i + 1];
    //blue[i / 3] = rgb[i + 2];
  }
}
/*void separate_rgb24(unsigned char* rgb, unsigned char* red, unsigned char* green, unsigned char* blue, int width, int height) {
#pragma omp parallel for num_threads(4) simd
  for (int i = 0; i < width * height; i++) {
    for (int j = 0; j < 3; j++) {
      (j == 0) ? red[i] = rgb[i * 3] : (j == 1) ? green[i] = rgb[i * 3 + 1] : blue[i] = rgb[i * 3 + 2];
    }
  }
}*/

void combine_rgb24(unsigned char* red, unsigned char* green, unsigned char* blue, unsigned char* rgb, int width, int height) {
#pragma omp parallel for num_threads(4)
//#pragma omp parallel for num_threads(4) simd
  for (int i = 0; i < width * height; i++) {
    rgb[i * 3] = red[i];
    rgb[i * 3 + 1] = green[i];
    rgb[i * 3 + 2] = blue[i];
  }
}
/*void combine_rgb24(unsigned char* red, unsigned char* green, unsigned char* blue, unsigned char* rgb, int width, int height) {
#pragma omp parallel for num_threads(4) simd
  for (int i = 0; i < width * height; i++) {
    for (int j = 0; j < 3; j++) {
      rgb[i * 3 + j] = (j == 0) ? red[i] : (j == 1) ? green[i] : blue[i];
    }
  }
}*/

void invert_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
#pragma omp parallel for num_threads(4)
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}

void frame_to_stdout(unsigned char* input, int size) {
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

void process_imageAlt(const void* p, int size) {
  unsigned char* preP = (unsigned char*)p;
  yuyv_to_greyscale(preP, outputFrameGreyscaleAlt, startingWidthAlt, startingHeightAlt);
  //frame_to_stdout(preP, (startingWidthAlt * startingHeightAlt * 2));
  //memcpy(outputFrameAlt, preP, (startingWidthAlt * startingHeightAlt * 2));
  //frame_to_stdout(outputFrameAlt, (startingWidthAlt * startingHeightAlt * 2));
  /*int status = write(1, preP, size);
  if (status == -1)
    perror("write");*/
    //frame_number++;
    /*if (force_format == 1) {
      //yuyv_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
      yuyv_to_rgb24(preP, outputFrame, startingWidth, startingHeight);
    } else if (force_format == 3) {
      rgb24_to_yuyv(preP, outputFrame, startingWidth, startingHeight);
    } else {
      yuyv_to_rgb24(preP, outputFrame, startingWidth, startingHeight);
    }*/
    //rgb24_to_yuyv(outputFrame, outputFrame2, startingWidth, startingHeight);
    /*rescale_bilinear(outputFrameGreyscale, startingWidth, startingHeight, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
    crop_greyscale(outputFrameGreyscale1, scaledOutWidth, scaledOutHeight, cropMatrix[0], outputFrameGreyscale2);
    //greyscale_to_sobel(outputFrameGreyscale2, outputFrameGreyscale3, croppedWidth, croppedHeight);
    crop_greyscale(outputFrameGreyscale2, croppedWidth, croppedHeight, cropMatrix[1], outputFrameGreyscale3);
    frame_to_stdout(outputFrameGreyscale3, (croppedWidth * croppedHeight));*/
    //yuyv_bilinearScale(preP, outputFrame, startingWidth, startingHeight, 640, 360);
    /*yuyv_to_rgb24(preP, outputFrame, startingWidth, startingHeight);
    rgb24_to_yuyv(outputFrame, outputFrame2, startingWidth, startingHeight);
    frame_to_stdout(outputFrame2, (startingWidth * startingHeight * 2));
    yuyv422_to_rgb24(preP, outputFrame3, startingWidth, startingHeight);
    frame_to_stdout(outputFrame3, (startingWidth * startingHeight * 3));*/
  croppedWidthAlt = 0;
  croppedHeightAlt = 0;
}

/*#define FPS_LIMITER_10_OF_30
#define FPS_LIMITER_30_OF_60*/
const bool noRGB = true;
const bool doMinimalGreyscaleOnly = true;
const bool doInvert = false;
void process_image(const void* p, int size) {
  unsigned char* preP = (unsigned char*)p;
  //yuyv_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
  rescale_bilinear_from_yuyv(preP, startingWidth, startingHeight, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
  // Values from 0 to 125 gets set to 0. Then ramp 125 through to 130 to 255. Finally we should set 131 to 255 to a value of 0
  frame_to_stdout(outputFrameGreyscale1, (scaledOutWidth * scaledOutHeight));
  //frame_to_stdout(preP, (startingWidth * startingHeight * 2));
  //rgb24_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
  /*rescale_bilinear(outputFrameGreyscale, startingWidth, startingHeight, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
  frame_to_stdout(outputFrameGreyscale1, scaledOutWidth * scaledOutHeight);*/
  /*//frame_to_stdout(outputFrameGreyscaleAlt, (startingWidthAlt * startingHeightAlt));
#ifdef FPS_LIMITER_10_OF_30
    if (frame_number % 3 == 0) {
#elseif FPS_LIMITER_30_OF_60
    if (frame_number % 6 == 0) {
#else
    if (true) {
#endif
      if (force_format == 1) {
        yuyv_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
        rescale_bilinear(outputFrameGreyscale, startingWidth, startingHeight, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
        frame_to_stdout(outputFrameGreyscale1, scaledOutWidth * scaledOutHeight);
      } else if (force_format == 3) {
        if (doMinimalGreyscaleOnly) {
          separate_rgb24(preP, redVals, greenVals, blueVals, startingWidth, startingHeight);
          //rgb24_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
          //rescale_bilinear(outputFrameGreyscale, startingWidth, startingHeight, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
          if (doInvert) {
            invert_greyscale(outputFrameGreyscale1, outputFrameGreyscale2, scaledOutWidth, scaledOutHeight);
            frame_to_stdout(outputFrameGreyscale2, scaledOutWidth * scaledOutHeight);
          } else {
            //frame_to_stdout(outputFrameGreyscale1, scaledOutWidth * scaledOutHeight);
        }
      } else {
        separate_rgb24(preP, redVals, greenVals, blueVals, startingWidth, startingHeight);
        rescale_bilinear(redVals, startingWidth, startingHeight, outputFrameScaledR, scaledOutWidth, scaledOutHeight);
        rescale_bilinear(greenVals, startingWidth, startingHeight, outputFrameScaledG, scaledOutWidth, scaledOutHeight);
        rescale_bilinear(blueVals, startingWidth, startingHeight, outputFrameScaledB, scaledOutWidth, scaledOutHeight);
        combine_rgb24(outputFrameScaledB, outputFrameScaledG, outputFrameScaledR, outputFrameRGB24, scaledOutWidth, scaledOutHeight);
        if (noRGB) {
          rgb24_to_greyscale(outputFrameRGB24, outputFrameGreyscale, scaledOutWidth, scaledOutHeight);
          if (doInvert) {
            invert_greyscale(outputFrameGreyscale, outputFrameGreyscale1, scaledOutWidth, scaledOutHeight);
            frame_to_stdout(outputFrameGreyscale1, (scaledOutWidth * scaledOutHeight));
          } else {
            frame_to_stdout(outputFrameGreyscale, (scaledOutWidth * scaledOutHeight));
          }
        } else {
          frame_to_stdout(outputFrameRGB24, (scaledOutWidth * scaledOutHeight * 3));
        }
      }
    } else {
      rgb24_to_greyscale(preP, outputFrameGreyscale, startingWidth, startingHeight);
    }
  } else if (frame_number == 2147483647) {
    frame_number = 0;
  }
  croppedWidth = 0;
  croppedHeight = 0;
  frame_number++;*/
}

int read_frameAlt(void) {
  struct v4l2_buffer bufAlt;
  unsigned int iAlt;
  switch (ioAlt) {
  case IO_METHOD_READALT:
    if (-1 == read(fdAlt, buffersAlt[0].start, buffersAlt[0].length)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exitAlt("read");
      }
    }
    process_imageAlt(buffersAlt[0].start, buffersAlt[0].length);
    break;
  case IO_METHOD_MMAPALT:
    CLEAR(bufAlt);
    bufAlt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufAlt.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_DQBUF, &bufAlt)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exitAlt("VIDIOC_DQBUF");
      }
    }
    assert(bufAlt.index < n_buffersAlt);
    //process_imageAlt(buffersAlt[bufAlt.index].start, bufAlt.bytesused);
    frame_to_stdout((unsigned char*)buffersAlt[bufAlt.index].start, bufAlt.bytesused);
    if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &bufAlt))
      errno_exitAlt("VIDIOC_QBUF");
    break;
  case IO_METHOD_USERPTRALT:
    CLEAR(bufAlt);
    bufAlt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufAlt.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_DQBUF, &bufAlt)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exitAlt("VIDIOC_DQBUF");
      }
    }
    for (iAlt = 0; iAlt < n_buffersAlt; ++iAlt)
      if (bufAlt.m.userptr == (unsigned long)buffersAlt[iAlt].start && bufAlt.length == buffersAlt[iAlt].length)
        break;
    assert(iAlt < n_buffersAlt);
    process_imageAlt((void*)bufAlt.m.userptr, bufAlt.bytesused);
    if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &bufAlt))
      errno_exitAlt("VIDIOC_QBUF");
    break;
  }

  struct v4l2_buffer buf;
  unsigned int i;
  switch (io) {
  case IO_METHOD_READ:
    if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("read");
      }
    }
    process_image(buffers[0].start, buffers[0].length);
    break;
  case IO_METHOD_MMAP:
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    frame_to_stdout(outputFrameGreyscaleAlt, (startingWidthAlt * startingHeightAlt));
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }
    assert(buf.index < n_buffers);
    process_image(buffers[buf.index].start, buf.bytesused);
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  case IO_METHOD_USERPTR:
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }
    for (i = 0; i < n_buffers; ++i)
      if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
        break;
    assert(i < n_buffers);
    process_image((void*)buf.m.userptr, buf.bytesused);
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  }

  //frame_to_stdout(outputFrameGreyscaleAlt, (startingWidthAlt* startingHeightAlt));
  return 1;
}

void stop_capturingAlt(void) {
  enum v4l2_buf_type type;
  switch (ioAlt) {
  case IO_METHOD_READALT:
    /* Nothing to do. */
    break;
  case IO_METHOD_MMAPALT:
  case IO_METHOD_USERPTRALT:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_STREAMOFF, &type))
      errno_exitAlt("VIDIOC_STREAMOFF");
    break;
  }
}

void start_capturingAlt(void) {
  unsigned int i;
  enum v4l2_buf_type type;
  switch (ioAlt) {
  case IO_METHOD_READALT:
    /* Nothing to do. */
    break;
  case IO_METHOD_MMAPALT:
    for (i = 0; i < n_buffersAlt; ++i) {
      struct v4l2_buffer buf;
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &buf))
        errno_exitAlt("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_STREAMON, &type))
      errno_exitAlt("VIDIOC_STREAMON");
    break;
  case IO_METHOD_USERPTRALT:
    for (i = 0; i < n_buffersAlt; ++i) {
      struct v4l2_buffer buf;
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long)buffersAlt[i].start;
      buf.length = buffersAlt[i].length;
      if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &buf))
        errno_exitAlt("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_STREAMON, &type))
      errno_exitAlt("VIDIOC_STREAMON");
    break;
  }
}

void uninit_deviceAlt(void) {
  unsigned int i;
  switch (ioAlt) {
  case IO_METHOD_READALT:
    free(buffersAlt[0].start);
    break;
  case IO_METHOD_MMAPALT:
    for (i = 0; i < n_buffersAlt; ++i)
      if (-1 == munmap(buffersAlt[i].start, buffersAlt[i].length))
        errno_exitAlt("munmap");
    break;
  case IO_METHOD_USERPTRALT:
    for (i = 0; i < n_buffersAlt; ++i)
      free(buffersAlt[i].start);
    break;
  }
  free(buffersAlt);
}

void init_readAlt(unsigned int buffer_size) {
  buffersAlt = (bufferAlt*)calloc(1, sizeof(*buffersAlt));
  if (!buffersAlt) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  buffersAlt[0].length = buffer_size;
  buffersAlt[0].start = malloc(buffer_size);
  if (!buffersAlt[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

void init_mmapAlt(void) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctlAlt(fdAlt, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support memory mapping\n", dev_name_alt);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exitAlt("VIDIOC_REQBUFS");
    }
  }
  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name_alt);
    exit(EXIT_FAILURE);
  }
  buffersAlt = (bufferAlt*)calloc(req.count, sizeof(*buffersAlt));
  if (!buffersAlt) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffersAlt = 0; n_buffersAlt < req.count; ++n_buffersAlt) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffersAlt;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_QUERYBUF, &buf))
      errno_exitAlt("VIDIOC_QUERYBUF");
    buffersAlt[n_buffersAlt].length = buf.length;
    buffersAlt[n_buffersAlt].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fdAlt, buf.m.offset);
    if (MAP_FAILED == buffersAlt[n_buffersAlt].start)
      errno_exitAlt("mmap");
  }
}

void init_userpAlt(unsigned int buffer_size) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  if (-1 == xioctlAlt(fdAlt, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support user pointer i/o\n", dev_name_alt);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exitAlt("VIDIOC_REQBUFS");
    }
  }
  buffersAlt = (bufferAlt*)calloc(4, sizeof(*buffersAlt));
  if (!buffersAlt) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffersAlt = 0; n_buffersAlt < 4; ++n_buffersAlt) {
    buffersAlt[n_buffersAlt].length = buffer_size;
    buffersAlt[n_buffersAlt].start = malloc(buffer_size);
    if (!buffersAlt[n_buffersAlt].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}

void set_framerateAlt(void) {
  struct v4l2_fract* tpf;
  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctlAlt(fdAlt, VIDIOC_G_PARM, &streamparm) != 0) {
    // Error
    fprintf(stderr, "IOCTL ERROR!\n");
  }
  if (streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
    //streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
    tpf = &streamparm.parm.capture.timeperframe;
    int num = 1, denom = 30;
    fprintf(stderr, "Setting time per frame on %s to: %d/%d\n", dev_name_alt, denom, num);
    tpf->denominator = denom;
    tpf->numerator = num;
  }
  if (xioctlAlt(fdAlt, VIDIOC_S_PARM, &streamparm) != 0) {
    fprintf(stderr, "Failed to set custom frame rate\n");
  }
}

void init_deviceAlt(void) {
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctlAlt(fdAlt, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n", dev_name_alt);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exitAlt("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", dev_name_alt);
    exit(EXIT_FAILURE);
  }
  switch (ioAlt) {
  case IO_METHOD_READALT:
    if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
      fprintf(stderr, "%s does not support read i/o\n", dev_name_alt);
      exit(EXIT_FAILURE);
    }
    break;
  case IO_METHOD_MMAPALT:
  case IO_METHOD_USERPTRALT:
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", dev_name_alt);
      exit(EXIT_FAILURE);
    }
    break;
  }
  /* Select video input, video standard and tune here. */
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctlAlt(fdAlt, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */
    if (-1 == xioctlAlt(fdAlt, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        /* Cropping not supported. */
        break;
      default:
        /* Errors ignored. */
        break;
      }
    }
  }
  else {
    /* Errors ignored. */
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "Forced format for alt (%s) to: %d\n", dev_name_alt, force_formatAlt);
  if (force_formatAlt) {
    if (force_formatAlt == 3) {
      fmt.fmt.pix.width = startingWidthAlt;
      fmt.fmt.pix.height = startingHeightAlt;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    }
    else if (force_formatAlt == 2) {
      fmt.fmt.pix.width = startingWidthAlt;
      fmt.fmt.pix.height = startingHeightAlt;
      //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    }
    else if (force_formatAlt == 1) {
      fmt.fmt.pix.width = startingWidthAlt;
      fmt.fmt.pix.height = startingHeightAlt;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      //fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;
    }
    if (-1 == xioctlAlt(fdAlt, VIDIOC_S_FMT, &fmt))
      errno_exitAlt("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  }
  else {
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctlAlt(fdAlt, VIDIOC_G_FMT, &fmt))
      errno_exitAlt("VIDIOC_G_FMT");
  }
  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  switch (ioAlt) {
  case IO_METHOD_READALT:
    init_readAlt(fmt.fmt.pix.sizeimage);
    break;
  case IO_METHOD_MMAPALT:
    init_mmapAlt();
    break;
  case IO_METHOD_USERPTRALT:
    init_userpAlt(fmt.fmt.pix.sizeimage);
    break;
  }
  set_framerateAlt();
}

void close_deviceAlt(void) {
  if (-1 == close(fdAlt))
    errno_exitAlt("close");
  fdAlt = -1;
}

void open_deviceAlt(void) {
  struct stat st;
  if (-1 == stat(dev_name_alt, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name_alt, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no device\n", dev_name_alt);
    exit(EXIT_FAILURE);
  }
  fdAlt = open(dev_name_alt, O_RDWR /* required */ | O_NONBLOCK, 0);
  if (-1 == fdAlt) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name_alt, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void crop_greyscale(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage) {
  if (croppedWidth > 0 || croppedHeight > 0) {
    croppedWidth = croppedWidth - crops[0] - crops[1];
    croppedHeight = croppedHeight - crops[2] - crops[3];
  }
  else {
    croppedWidth = width - crops[0] - crops[1];
    croppedHeight = height - crops[2] - crops[3];
  }
#pragma omp parallel for
  for (int y = crops[2]; y < crops[2] + croppedHeight; y++) {
    for (int x = crops[0]; x < crops[0] + croppedWidth; x++) {
      int croppedX = x - crops[0];
      int croppedY = y - crops[2];
      int croppedIndex = croppedY * croppedWidth + croppedX;
      int index = y * width + x;
      croppedImage[croppedIndex] = image[index];
    }
  }
}

int read_frame(void) {
    struct v4l2_buffer buf;
    unsigned int i;
    switch (io) {
    case IO_METHOD_READ:
        if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
            case EAGAIN:
                return 0;
            case EIO:
                // Could ignore EIO, see spec.
                // fall through
            default:
                errno_exit("read");
            }
        }
        process_image(buffers[0].start, buffers[0].length);
        break;
    case IO_METHOD_MMAP:
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;
            case EIO:
                // Could ignore EIO, see spec.
                // fall through
            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }
        assert(buf.index < n_buffers);
        process_image(buffers[buf.index].start, buf.bytesused);
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
        break;
    case IO_METHOD_USERPTR:
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;
            case EIO:
                // Could ignore EIO, see spec.
                // fall through
            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }
        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
                break;
        assert(i < n_buffers);
        process_image((void*)buf.m.userptr, buf.bytesused);
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
        break;
    }
    return 1;
}

void mainloop(void) {
    unsigned int count;
    unsigned int loopIsInfinite = 0;
    if (frame_count == 0) loopIsInfinite = 1; //infinite loop
    count = frame_count;
    while ((count-- > 0) || loopIsInfinite) {
      for (;;) {
        /*fd_set fdsAlt;
        struct timeval tvAlt;
        int rAlt;
        FD_ZERO(&fdsAlt);
        FD_SET(fdAlt, &fdsAlt);
        // Timeout.
        tvAlt.tv_sec = 2;
        tvAlt.tv_usec = 0;
        rAlt = select(fdAlt + 1, &fdsAlt, NULL, NULL, &tvAlt);
        if (-1 == rAlt) {
          if (EINTR == errno)
            continue;
          errno_exitAlt("select");
        }
        if (0 == rAlt) {
          fprintf(stderr, "select timeout\n");
          exit(EXIT_FAILURE);
        }
        if (read_frameAlt())
          break;
        // EAGAIN - continue select loop. */

        fd_set fds;
        struct timeval tv;
        int r;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        // Timeout.
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (-1 == r) {
          if (EINTR == errno)
            continue;
          errno_exit("select");
        }
        if (0 == r) {
          fprintf(stderr, "select timeout\n");
          exit(EXIT_FAILURE);
        }
        if (read_frame())
          break;
        // EAGAIN - continue select loop
      }
    }
}

void stop_capturing(void) {
    enum v4l2_buf_type type;
    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");
        break;
    }
}

void start_capturing(void) {
    unsigned int i;
    enum v4l2_buf_type type;
    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;
    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");
        break;
    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index = i;
            buf.m.userptr = (unsigned long)buffers[i].start;
            buf.length = buffers[i].length;
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");
        break;
    }
}

void uninit_device(void) {
    unsigned int i;
    switch (io) {
    case IO_METHOD_READ:
        free(buffers[0].start);
        break;
    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
                errno_exit("munmap");
        break;
    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free(buffers[i].start);
        break;
    }
    free(buffers);
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

void init_mmap(void) {
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support memory mapping\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }
    if (req.count < 2) {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
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

void init_userp(unsigned int buffer_size) {
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support user pointer i/o\n", dev_name);
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

void set_framerate(void) {
    struct v4l2_fract* tpf;
    struct v4l2_streamparm streamparm;
    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_PARM, &streamparm) != 0) {
        // Error
        fprintf(stderr, "IOCTL ERROR!\n");
    }
    if (streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
        //streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
        tpf = &streamparm.parm.capture.timeperframe;
        int num = 1, denom = 30;
        fprintf(stderr, "Setting time per frame to: %d/%d\n", denom, num);
        tpf->denominator = denom;
        tpf->numerator = num;
    }
    if (xioctl(fd, VIDIOC_S_PARM, &streamparm) != 0) {
        fprintf(stderr, "Failed to set custom frame rate\n");
    }
}

void init_device(void) {
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s is no video capture device\n", dev_name);
        exit(EXIT_FAILURE);
    }
    switch (io) {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
            fprintf(stderr, "%s does not support read i/o\n", dev_name);
            exit(EXIT_FAILURE);
        }
        break;
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
            exit(EXIT_FAILURE);
        }
        break;
    }
    /* Select video input, video standard and tune here. */
    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else {
        /* Errors ignored. */
    }
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fprintf(stderr, "Forced format for main (%s) to: %d\n", dev_name, force_formatAlt);
    if (force_format) {
        if (force_format == 3) {
            //fmt.fmt.pix.width = 1280;
            //fmt.fmt.pix.height = 720;
            fmt.fmt.pix.width = startingWidth;
            fmt.fmt.pix.height = startingHeight;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
            fmt.fmt.pix.field = V4L2_FIELD_NONE;
            //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        } else if (force_format == 2) {
            //fmt.fmt.pix.width = 1280;
            //fmt.fmt.pix.height = 720;
            fmt.fmt.pix.width = startingWidth;
            fmt.fmt.pix.height = startingHeight;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
            fmt.fmt.pix.field = V4L2_FIELD_NONE;
            //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        } else if (force_format == 1) {
            //fmt.fmt.pix.width = 1280;
            //fmt.fmt.pix.height = 720;
            fmt.fmt.pix.width = startingWidth;
            fmt.fmt.pix.height = startingHeight;
            //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
            fmt.fmt.pix.field = V4L2_FIELD_NONE;
            //fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;
        }
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");
        /* Note VIDIOC_S_FMT may change width and height. */
    } else {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            errno_exit("VIDIOC_G_FMT");
    }
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    switch (io) {
    case IO_METHOD_READ:
        init_read(fmt.fmt.pix.sizeimage);
        break;
    case IO_METHOD_MMAP:
        init_mmap();
        break;
    case IO_METHOD_USERPTR:
        init_userp(fmt.fmt.pix.sizeimage);
        break;
    }
    //set_framerate();
}

void close_device(void) {
    if (-1 == close(fd))
        errno_exit("close");
    fd = -1;
}

void open_device(void) {
    struct stat st;
    if (-1 == stat(dev_name, &st)) {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }
    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
    if (-1 == fd) {
        fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

int start_main(char *device_name, char* device_name_alt) {
    memset(outputFrame, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrame2, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale1, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale2, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale3, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale4, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameGreyscale5, 0, startingWidth * startingHeight * sizeof(unsigned char));
    memset(outputFrameAlt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrame2Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrame3Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscaleAlt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscale1Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscale2Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscale3Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscale4Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    memset(outputFrameGreyscale5Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
    dev_name = (char*)calloc(64, sizeof(char));
    strcpy(dev_name, device_name);
    dev_name_alt = (char*)calloc(64, sizeof(char));
    strcpy(dev_name_alt, device_name_alt);
    fprintf(stderr, "Potential output resolutions for alt (%s): %dx%d (unscaled)", dev_name_alt, (startingWidthAlt), (startingHeightAlt));
    int tempCropAmtWidthAlt = scaledOutWidthAlt;
    int tempCropAmtHeightAlt = scaledOutHeightAlt;
    for (int i = 0; i < (sizeof(cropMatrixAlt) / sizeof(*cropMatrixAlt)); i++) {
      tempCropAmtWidthAlt = (tempCropAmtWidthAlt - (cropMatrixAlt[i][0] + cropMatrixAlt[i][1]));
      tempCropAmtHeightAlt = (tempCropAmtHeightAlt - (cropMatrixAlt[i][2] + cropMatrixAlt[i][3]));
      fprintf(stderr, " >> %dx%d (L:%d,R:%d,T:%d,B:%d)", tempCropAmtWidthAlt, tempCropAmtHeightAlt, cropMatrixAlt[i][0], cropMatrixAlt[i][1], cropMatrixAlt[i][2], cropMatrixAlt[i][3]);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Potential output resolutions for main (%s): %dx%d (unscaled)", dev_name, (startingWidth), (startingHeight));
    int tempCropAmtWidth = scaledOutWidth;
    int tempCropAmtHeight = scaledOutHeight;
    for (int i = 0; i < (sizeof(cropMatrix) / sizeof(*cropMatrix)); i++) {
      tempCropAmtWidth = (tempCropAmtWidth - (cropMatrix[i][0] + cropMatrix[i][1]));
      tempCropAmtHeight = (tempCropAmtHeight - (cropMatrix[i][2] + cropMatrix[i][3]));
      fprintf(stderr, " >> %dx%d (L:%d,R:%d,T:%d,B:%d)", tempCropAmtWidth, tempCropAmtHeight, cropMatrix[i][0], cropMatrix[i][1], cropMatrix[i][2], cropMatrix[i][3]);
    }
    fprintf(stderr, "\n");
    /*open_deviceAlt();
    init_deviceAlt();
    start_capturingAlt();
    fprintf(stderr, "Initialized alt (%s)..\n", dev_name_alt);*/
    open_device();
    init_device();
    start_capturing();
    fprintf(stderr, "Initialized main (%s)..\n", dev_name);
    //fprintf(stderr, "Starting loop with the following device(s): %s and %s\n", dev_name, dev_name_alt);
    fprintf(stderr, "Starting loop with the following device(s): %s\n", dev_name);
    mainloop();
    /* main loop
    for (;;) {
      fd_set fdsAlt;
      struct timeval tvAlt;
      int rAlt;
      FD_ZERO(&fdsAlt);
      FD_SET(fdAlt, &fdsAlt);
      // Timeout.
      tvAlt.tv_sec = 2;
      tvAlt.tv_usec = 0;
      rAlt = select(fdAlt + 1, &fdsAlt, NULL, NULL, &tvAlt);
      if (-1 == rAlt) {
        if (EINTR == errno)
          continue;
        errno_exitAlt("select");
      }
      if (0 == rAlt) {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
      }
      if (read_frameAlt())
        break;
      // EAGAIN - continue select loop.

      fd_set fds;
      struct timeval tv;
      int r;
      FD_ZERO(&fds);
      FD_SET(fd, &fds);
      // Timeout.
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      r = select(fd + 1, &fds, NULL, NULL, &tv);
      if (-1 == r) {
        if (EINTR == errno)
          continue;
        errno_exit("select");
      }
      if (0 == r) {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
      }
      if (read_frame())
        break;
      // EAGAIN - continue select loop.
    }
    // main loop end
    // alt loop
    fprintf(stderr, "Started alt-loop.\n");
    for (;;) {
      fd_set fds;
      struct timeval tv;
      int r;
      FD_ZERO(&fds);
      FD_SET(fdAlt, &fds);
      // Timeout.
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      r = select(fdAlt + 1, &fds, NULL, NULL, &tv);
      if (-1 == r) {
        if (EINTR == errno)
          continue;
        errno_exitAlt("select");
      }
      if (0 == r) {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
      }
      if (read_frameAlt())
        break;
      // EAGAIN - continue select loop.
    }
    // alt loop end */
    /*stop_capturingAlt();
    uninit_deviceAlt();
    close_deviceAlt();
    fprintf(stderr, "\n");*/
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}