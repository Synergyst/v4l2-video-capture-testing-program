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
//#include "imgproc.h"

using namespace std;

const int cropMatrix[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} }; // Crop size matrix (scale up or down as needed)
int croppedWidth = 0, croppedHeight = 0;
const int KERNEL_SIZE = 3; // The kernel size of the Gaussian blur, default: 5
const double SIGMA = 2.0; // The sigma value of the Gaussian blur, default: 2.0

void yuyv_to_greyscale(const unsigned char* input, unsigned char* grey, int width, int height) {
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      // YUYV format stores chroma (Cb and Cr) values interleaved with the luma (Y) values.
      // So, we need to skip every other pixel.
      int y_index = index * 2;
      grey[index] = input[y_index];
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
  // The maximum value for the Sobel operator
  const int maxSobel = 4 * 255;
  // The Sobel operator as a 3x3 matrix
  const int sobelX[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
  const int sobelY[3][3] = { {-1, -2, -1}, {0, 0, 0}, {1, 2, 1} };
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

void rescale_bilinear(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height) {
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
}

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
      }
      else {
        value = 0.0F;
      }
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

void gaussianBlur(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight) {
  std::vector<double> kernel = computeGaussianKernel(KERNEL_SIZE, SIGMA);
  // Perform the blur in the horizontal direction
#pragma omp parallel for simd
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
#pragma omp parallel for simd
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
#pragma omp parallel for
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}

void frame_to_stdout(unsigned char* input, int size) {
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
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