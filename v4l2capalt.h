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

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

enum io_methodAlt {
  IO_METHOD_READALT,
  IO_METHOD_MMAPALT,
  IO_METHOD_USERPTRALT,
};

struct bufferAlt {
  void* start;
  size_t length;
};

char* dev_name_alt;
enum io_methodAlt ioAlt = IO_METHOD_MMAPALT;
int fdAlt = -1;
struct bufferAlt* buffersAlt;
unsigned int n_buffersAlt;
int out_bufAlt;
int force_formatAlt = 1; // YUYV, hard-coded
//static int force_formatAlt = 3; // RGB24, hard-coded
//static int force_formatAlt = 0; // allow-user-specification/override
int frame_countAlt = 0;
int frame_numberAlt = 0;

void errno_exitAlt(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

int xioctlAlt(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}

// The width and height of the input video and any downscaled output video
const int startingWidthAlt = 320;
const int startingHeightAlt = 180;
const int scaledOutWidthAlt = 452;
const int scaledOutHeightAlt = 254;
int croppedWidthAlt = 0;
int croppedHeightAlt = 0;
// Crop size matrix (scale up or down as needed)
int cropMatrixAlt[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} };
// The maximum value for the Sobel operator
/*static const int maxSobel = 4 * 255;
// The Sobel operator as a 3x3 matrix
static const int sobelX[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
static const int sobelY[3][3] = { {-1, -2, -1}, {0, 0, 0}, {1, 2, 1} };*/

// Allocate memory for the input and output frames
unsigned char* outputFrameAlt = new unsigned char[startingWidthAlt * startingHeightAlt * 2];
unsigned char* outputFrame2Alt = new unsigned char[startingWidthAlt * startingHeightAlt * 2];
unsigned char* outputFrame3Alt = new unsigned char[startingWidthAlt * startingHeightAlt *3];
unsigned char* outputFrameGreyscaleAlt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale1Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale2Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale3Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale4Alt = new unsigned char[startingWidthAlt * startingHeightAlt];
unsigned char* outputFrameGreyscale5Alt = new unsigned char[startingWidthAlt * startingHeightAlt];


/*
static void yuyv_to_greyscale(const unsigned char* yuyv, unsigned char* grey, int width, int height) {
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

static void replace_pixels_below_val(const unsigned char* input, unsigned char* output, int width, int height, const int val) {
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Get the pixel value at the current position
      unsigned char pixel = input[y * width + x];
      // If the pixel value is below 63, replace it with a modified value
      if (pixel < val) {
        output[y * width + x] = (unsigned char)sqrt(output[y * width + x]);
      }
      else {
        // Otherwise, copy the pixel value from the input to the output
        output[y * width + x] = pixel;
      }
    }
  }
}

static void replace_pixels_above_val(const unsigned char* input, unsigned char* output, int width, int height, const int val) {
#pragma omp parallel for
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

static void uyvy_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
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

static void greyscale_to_sobel(const unsigned char* input, unsigned char* output, int width, int height) {
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

static void uyvy_sobel(unsigned char* input, unsigned char* output, int width, int height) {
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

static void uyvy_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
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

static void yuyv_to_uyvy(unsigned char* input, unsigned char* output, int width, int height) {
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

static void rgb24_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
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
      // Calculate the greyscale value using the formula:
      // greyscale = 0.299 * red + 0.587 * green + 0.114 * blue
      unsigned char greyscale = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      // Set the greyscale value as the intensity of the output pixel
      output[y * width + x] = greyscale;
    }
  }
}

static void rgb24_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
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

static void yuyv_to_rgb24(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Extract the Y, U, and V components from the YUYV pixel
      unsigned char y1 = input[offset];
      unsigned char u = input[offset + 1];
      unsigned char y2 = input[offset + 2];
      unsigned char v = input[offset + 3];
      // Calculate the R, G, and B components using the following formulas:
      // R = y1 + 1.140 * (v - 128)
      // G = y1 - 0.394 * (u - 128) - 0.581 * (v - 128)
      // B = y1 + 2.032 * (u - 128)
      unsigned char r = static_cast<unsigned char>(y1 + 1.140 * (v - 128));
      unsigned char g = static_cast<unsigned char>(y1 - 0.394 * (u - 128) - 0.581 * (v - 128));
      unsigned char b = static_cast<unsigned char>(y1 + 2.032 * (u - 128));
      // Pack the R, G, and B components into an RGB24 pixel
      output[offset] = r;
      output[offset + 1] = g;
      output[offset + 2] = b;
    }
  }
}

static void yuyv422_to_rgb24(unsigned char* yuyv, unsigned char* rgb, int width, int height) {
#pragma omp parallel for
  for (int i = 0; i < width * height * 2; i += 4) {
    int y0 = yuyv[i];
    int u = yuyv[i + 1];
    int y1 = yuyv[i + 2];
    int v = yuyv[i + 3];

    int c = y0 - 16;
    int d = u - 128;
    int e = v - 128;

    int r = (298 * c + 409 * e + 128) >> 8;
    int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int b = (298 * c + 516 * d + 128) >> 8;

    if (r < 0) r = 0; else if (r > 255) r = 255;
    if (g < 0) g = 0; else if (g > 255) g = 255;
    if (b < 0) b = 0; else if (b > 255) b = 255;

    rgb[i] = r;
    rgb[i + 1] = g;
    rgb[i + 2] = b;

    c = y1 - 16;
    r = (298 * c + 409 * e + 128) >> 8;
    g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b = (298 * c + 516 * d + 128) >> 8;

    if (r < 0) r = 0; else if (r > 255) r = 255;
    if (g < 0) g = 0; else if (g > 255) g = 255;
    if (b < 0) b = 0; else if (b > 255) b = 255;

    rgb[i + 3] = r;
    rgb[i + 4] = g;
    rgb[i + 5] = b;
  }
}


static void frame_to_stdout(unsigned char* input, int size) {
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

static void resize_image_nearest_neighbor(const uint8_t* src, int src_width, int src_height, uint8_t* dst, int dst_width, int dst_height) {
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

static void rescale_bilinear(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
#pragma omp parallel for
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
// The kernel size of the Gaussian blur
//const int KERNEL_SIZE = 5;
const int KERNEL_SIZE = 7;

// The sigma value of the Gaussian blur
const double SIGMA = 2.0;

// A helper function to compute the Gaussian kernel
static std::vector<double> computeGaussianKernel(int kernelSize, double sigma) {
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

// The main function that performs the Gaussian blur
static void gaussianBlur(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight) {
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

static void rgb24_bilinearScale(unsigned char* inputFrame, int inputWidth, int inputHeight, unsigned char* outputFrame, int outputWidth, int outputHeight) {
  // Calculate the scaling factors
  float xScale = (float)inputWidth / outputWidth;
  float yScale = (float)inputHeight / outputHeight;

  // Iterate over the output image pixels
#pragma omp parallel for
  for (int y = 0; y < outputHeight; y++) {
    for (int x = 0; x < outputWidth; x++) {
      // Calculate the coordinates of the corresponding input pixel
      float inX = x * xScale;
      float inY = y * yScale;

      // Calculate the integer and fractional parts of the coordinates
      int x1 = (int)inX;
      int y1 = (int)inY;
      float dx = inX - x1;
      float dy = inY - y1;

      // Calculate the four neighboring pixels
      int x2 = x1 + 1;
      int y2 = y1 + 1;
      if (x2 >= inputWidth) x2 = x1;
      if (y2 >= inputHeight) y2 = y1;

      // Get the pixel values
      unsigned char r1 = inputFrame[(y1 * inputWidth + x1) * 3 + 0];
      unsigned char g1 = inputFrame[(y1 * inputWidth + x1) * 3 + 1];
      unsigned char b1 = inputFrame[(y1 * inputWidth + x1) * 3 + 2];
      unsigned char r2 = inputFrame[(y1 * inputWidth + x2) * 3 + 0];
      unsigned char g2 = inputFrame[(y1 * inputWidth + x2) * 3 + 1];
      unsigned char b2 = inputFrame[(y1 * inputWidth + x2) * 3 + 2];
      unsigned char r3 = inputFrame[(y2 * inputWidth + x1) * 3 + 0];
      unsigned char g3 = inputFrame[(y2 * inputWidth + x1) * 3 + 1];
      unsigned char b3 = inputFrame[(y2 * inputWidth + x1) * 3 + 2];
      unsigned char r4 = inputFrame[(y2 * inputWidth + x2) * 3 + 0];
      unsigned char g4 = inputFrame[(y2 * inputWidth + x2) * 3 + 1];
      unsigned char b4 = inputFrame[(y2 * inputWidth + x2) * 3 + 2];

      // Interpolate the pixel value using bilinear interpolation
      unsigned char r = (unsigned char)((1.0 - dx) * (1.0 - dy) * r1 + dx * (1.0 - dy) * r2 + (1.0 - dx) * dy * r3 + dx * dy * r4);
      unsigned char g = (unsigned char)((1.0 - dx) * (1.0 - dy) * g1 + dx * (1.0 - dy) * g2 + (1.0 - dx) * dy * g3 + dx * dy * g4);
      unsigned char b = (unsigned char)((1.0 - dx) * (1.0 - dy) * b1 + dx * (1.0 - dy) * b2 + (1.0 - dx) * dy * b3 + dx * dy * b4);

      // Set the pixel value in the output image
      outputFrame[(y * outputWidth + x) * 3 + 0] = r;
      outputFrame[(y * outputWidth + x) * 3 + 1] = g;
      outputFrame[(y * outputWidth + x) * 3 + 2] = b;
    }
  }
}

static void yuyv_bilinearScale(unsigned char* input, unsigned char* output, int inputWidth, int inputHeight, int outputWidth, int outputHeight) {
  // Calculate the scaling factors
  float scaleWidth = (float)outputWidth / inputWidth;
  float scaleHeight = (float)outputHeight / inputHeight;

  // Loop through the pixels of the output image
#pragma omp parallel for
  for (int y = 0; y < outputHeight; y++) {
    for (int x = 0; x < outputWidth; x++) {
      // Calculate the corresponding coordinates in the input image
      float inX = x / scaleWidth;
      float inY = y / scaleHeight;

      // Calculate the weights for the surrounding pixels
      int x1 = floor(inX);
      int x2 = ceil(inX);
      int y1 = floor(inY);
      int y2 = ceil(inY);
      float w1 = (x2 - inX) * (y2 - inY);
      float w2 = (inX - x1) * (y2 - inY);
      float w3 = (x2 - inX) * (inY - y1);
      float w4 = (inX - x1) * (inY - y1);

      // Perform the bilinear interpolation
      int yuyvIndex = (y * outputWidth + x) * 2;
      output[yuyvIndex] = (w1 * input[(y1 * inputWidth + x1) * 2] + w2 * input[(y1 * inputWidth + x2) * 2] + w3 * input[(y2 * inputWidth + x1) * 2] + w4 * input[(y2 * inputWidth + x2) * 2]) / (w1 + w2 + w3 + w4);
      output[yuyvIndex + 1] = (w1 * input[(y1 * inputWidth + x1) * 2 + 1] + w2 * input[(y1 * inputWidth + x2) * 2 + 1] + w3 * input[(y2 * inputWidth + x1) * 2 + 1] + w4 * input[(y2 * inputWidth + x2) * 2 + 1]) / (w1 + w2 + w3 + w4);
    }
  }
}

static void crop_greyscale(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage) {
  if (croppedWidth > 0 || croppedHeight > 0) {
    croppedWidth = croppedWidth - crops[0] - crops[1];
    croppedHeight = croppedHeight - crops[2] - crops[3];
  } else {
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
*/
void process_imageAlt(const void* p, int size) {
  unsigned char* preP = (unsigned char*)p;
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

int read_frameAlt(void) {
  struct v4l2_buffer buf;
  unsigned int i;
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
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_DQBUF, &buf)) {
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
    assert(buf.index < n_buffersAlt);
    process_imageAlt(buffersAlt[buf.index].start, buf.bytesused);
    if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &buf))
      errno_exitAlt("VIDIOC_QBUF");
    break;
  case IO_METHOD_USERPTRALT:
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctlAlt(fdAlt, VIDIOC_DQBUF, &buf)) {
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
    for (i = 0; i < n_buffersAlt; ++i)
      if (buf.m.userptr == (unsigned long)buffersAlt[i].start && buf.length == buffersAlt[i].length)
        break;
    assert(i < n_buffersAlt);
    process_imageAlt((void*)buf.m.userptr, buf.bytesused);
    if (-1 == xioctlAlt(fdAlt, VIDIOC_QBUF, &buf))
      errno_exitAlt("VIDIOC_QBUF");
    break;
  }
  return 1;
}

void mainloopAlt(void) {
  unsigned int count;
  unsigned int loopIsInfinite = 0;
  if (frame_countAlt == 0) loopIsInfinite = 1; //infinite loop
  count = frame_countAlt;
  while ((count-- > 0) || loopIsInfinite) {
    for (;;) {
      fd_set fds;
      struct timeval tv;
      int r;
      FD_ZERO(&fds);
      FD_SET(fdAlt, &fds);
      /* Timeout. */
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
      /* EAGAIN - continue select loop. */
    }
  }
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
    } else {
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
    } else {
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
  } else {
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
    } else if (force_formatAlt == 2) {
      fmt.fmt.pix.width = startingWidthAlt;
      fmt.fmt.pix.height = startingHeightAlt;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    } else if (force_formatAlt == 1) {
      fmt.fmt.pix.width = startingWidthAlt;
      fmt.fmt.pix.height = startingHeightAlt;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      //fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;
    }
    if (-1 == xioctlAlt(fdAlt, VIDIOC_S_FMT, &fmt))
      errno_exitAlt("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  } else {
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

/*int init_mainAlt() {
  memset(outputFrameAlt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrame2Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrame3Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscaleAlt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscale1Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscale2Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscale3Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscale4Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  memset(outputFrameGreyscale5Alt, 0, startingWidthAlt * startingHeightAlt * sizeof(unsigned char));
  open_deviceAlt();
  init_deviceAlt();
  start_capturingAlt();
  mainloopAlt();
  stop_capturingAlt();
  uninit_deviceAlt();
  close_deviceAlt();
  fprintf(stderr, "\n");
  return 0;
}*/