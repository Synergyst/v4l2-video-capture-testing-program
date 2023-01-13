/*
 *
 *  V4L2 video capture test program
 *
 *  This is not meant to be representative of a production-ready program, it may not work without heavy modification..
 *  Lots of variables, functions, etc may be named incorrectly, have issues, etc.
 *  This file gets updated frequently with test code; please understand that a lot of parts of it may not make any sense. :)
 *
 *  Understanding that this is really meant for internal-use only/testing; feel free to modify/reuse/distribute this code in any way without restrictions.
 *
 *  Example command-line usage:
 *    Usage: ./build.sh
 *    This will attempt to build the source once uploaded by Visual Studio 2022 to the Raspberry Pi
 * 
 *    Usage: ./v4l2-capture-test | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -i pipe:0
 *    This will run the main program and then output the processed video data to FFPlay to test the processed frames
 *
 */
#include <iostream>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
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

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

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
    fd;
  unsigned int n_buffers;
  bool isTC358743 = true,
    isThermalCamera = true;
  unsigned char *outputFrameGreyscale;
  char *device;
};

struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;

const int cropMatrix[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} }; // Crop size matrix (scale up or down as needed)
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

void yuyv_to_greyscale(const unsigned char* input, unsigned char* grey, int width, int height) {
#pragma omp parallel for simd
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

void uyvy_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for simd
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
  // Iterate over each pixel in the input image
#pragma omp parallel for simd
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

void gaussian_blur(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight) {
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
#pragma omp parallel for simd num_threads(4)
  for (int i = 0; i < width * height; i++) {
    output[i] = 255 - input[i];
  }
}

void frame_to_stdout(unsigned char* input, int size) {
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

int init_dev(struct buffer* buffers, struct devInfo *devInfos) {
  // We need to initialize our device and then configure our TC358743 if we are even using one
  unsigned int i;
  enum v4l2_buf_type type;
  fprintf(stderr, "\n[cap] Starting V4L2 capture testing program with the following V4L2 device: %s\n", devInfos->device);

  struct stat st;
  if (-1 == stat(devInfos->device, &st)) {
    fprintf(stderr, "[cap] Cannot identify '%s': %d, %s\n", devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "[cap] %s is no device\n", devInfos->device);
    exit(EXIT_FAILURE);
  }
  devInfos->fd = open(devInfos->device, O_RDWR | O_NONBLOCK, 0);
  if (-1 == devInfos->fd) {
    fprintf(stderr, "[cap] Cannot open '%s': %d, %s\n", devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "[cap] Opened V4L2 device: %s\n", devInfos->device);

  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap] %s is no V4L2 device\n", devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "[cap] %s is no video capture device\n", devInfos->device);
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
  fprintf(stderr, "[cap] Forcing format for %s to: %d\n", devInfos->device, devInfos->force_format);
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
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap] %s does not support memory mapping\n", devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (req.count < 2) {
    fprintf(stderr, "[cap] Insufficient buffer memory on %s\n", devInfos->device);
    exit(EXIT_FAILURE);
  }
  buffers = (buffer*)calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "[cap] Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (devInfos->n_buffers = 0; devInfos->n_buffers < req.count; ++devInfos->n_buffers) {
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

  // TODO: Have some way of passing flags from main() so we can handle settings in this area
  struct v4l2_dv_timings timings;
  v4l2_std_id std;
  int ret;

  memset(&timings, 0, sizeof timings);
  ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
  if (ret >= 0) {
    fprintf(stderr, "[cap] QUERY_DV_TIMINGS for %s: %ux%u pixclk %llu\n", devInfos->device, timings.bt.width, timings.bt.height, timings.bt.pixelclock);
    devInfos->startingWidth = timings.bt.width;
    devInfos->startingHeight = timings.bt.height;
    devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * sizeof(unsigned char);
    devInfos->scaledOutSize = devInfos->scaledOutWidth * devInfos->scaledOutHeight;
    devInfos->outputFrameGreyscale = (unsigned char*)malloc(devInfos->startingSize);
    memset(devInfos->outputFrameGreyscale, 0, devInfos->startingSize);
    // Can read DV timings, so set them.
    ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
    if (ret < 0) {
      fprintf(stderr, "[cap] Failed to set DV timings\n");
      return 1;
    } else {
      double tot_height, tot_width;
      const struct v4l2_bt_timings* bt = &timings.bt;
      tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
      tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
      devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));
      devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
      int rawInputThroughput = (float)((float)(devInfos->framerate * devInfos->startingSize * 2.0F) / 125000.0F); // Measured in megabits/sec based on input framerate
      int rawOutputThroughput = (float)((((float)devInfos->framerate / devInfos->framerateDivisor) * devInfos->scaledOutSize) / 125000.0F); // Measured in megabits/sec based on output framerate
      fprintf(stderr, "[cap] device_name: %s, isTC358743: %d, isThermalCamera: %d, startingWidth: %d, startingHeight: %d, startingSize: %d, scaledOutWidth: %d, scaledOutHeight: %d, scaledOutSize: %d, framerate: %u, framerateDivisor: %d, targetFramerate: %d, rawInputThroughput: ~%dMb/sec, rawOutputThroughput: ~%dMb/sec\n", devInfos->device, devInfos->isTC358743, devInfos->isThermalCamera, devInfos->startingWidth, devInfos->startingHeight, devInfos->startingSize, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->scaledOutSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, rawInputThroughput, rawOutputThroughput);
    }
  } else {
    memset(&std, 0, sizeof std);
    ret = ioctl(devInfos->fd, VIDIOC_QUERYSTD, &std);
    if (ret >= 0) {
      // Can read standard, so set it.
      ret = xioctl(devInfos->fd, VIDIOC_S_STD, &std);
      if (ret < 0) {
        fprintf(stderr, "[cap] Failed to set standard\n");
        return 1;
      } else {
        // SD video - assume 50Hz / 25fps
        devInfos->framerate = 25;
      }
    }
  }
  // TODO: Have some way of passing flags from main() so we can handle settings in this area above
  fprintf(stderr, "[cap] Initialized V4L2 device: %s\n", devInfos->device);
  for (i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
  }
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &type))
    errno_exit("VIDIOC_STREAMON");
  fprintf(stderr, "[cap] Started capturing from V4L2 device: %s\n", devInfos->device);
  return 0;
}

int get_frame(struct buffer *buffers, struct devInfo *devInfos, captureType capType) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period to wait for device to respond
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  r = select(devInfos->fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) {
    if (EINTR == errno)
      // Do nothing
      //continue;
    errno_exit("select");
  }
  if (0 == r) {
    fprintf(stderr, "select timeout\n");
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
      fprintf(stderr, "[cap] EAGAIN\n");
      return 0;
    case EIO:
      // Could ignore EIO, see spec.
      // fall through
    default:
      errno_exit("VIDIOC_DQBUF");
    }
  }
  assert(buf.index < devInfos->n_buffers);
  if (devInfos->frame_number % devInfos->framerateDivisor == 0) {
    rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    gaussian_blur(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    frame_to_stdout(devInfos->outputFrameGreyscale, (devInfos->scaledOutWidth * devInfos->scaledOutHeight));
  }
  devInfos->frame_number++;
  if (devInfos->frame_number % devInfos->framerateDivisor == 0) {
    rescale_bilinear_from_yuyv((unsigned char*)buffers[buf.index].start, devInfos->startingWidth, devInfos->startingHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    gaussian_blur(devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    invert_greyscale(devInfos->outputFrameGreyscale, devInfos->outputFrameGreyscale, devInfos->scaledOutWidth, devInfos->scaledOutHeight);
    devInfos->frame_number++;
  }
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
    errno_exit("VIDIOC_QBUF");
  // EAGAIN - continue select loop
  return 0;
}

int deinit_bufs(struct buffer *buffers, struct devInfo *devInfos) {
  // We are using DMA (Direct-Memory-Access), so we shouldn't have much to cleanup
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
      errno_exit("munmap");
  free(buffers);
  fprintf(stderr, "[cap] Uninitialized V4L2 device: %s\n", devInfos->device);

  if (-1 == close(devInfos->fd))
    errno_exit("close");
  devInfos->fd = -1;
  fprintf(stderr, "[cap] Closed V4L2 device: %s\n", devInfos->device);
  fprintf(stderr, "\n");
  return 0;
}

int init_vars(struct devInfo* devInfos, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera, char *dev_name) {
  devInfos->device = (char*)calloc(64, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->frame_number = 0,
    devInfos->framerate = -1,
    devInfos->framerateDivisor = 1,
    devInfos->startingWidth = -1,
    devInfos->startingHeight = -1,
    devInfos->startingSize = -1,
    devInfos->scaledOutSize = (scaledOutWidth * scaledOutHeight),
    devInfos->force_format = force_format,
    devInfos->scaledOutWidth = scaledOutWidth,
    devInfos->scaledOutHeight = scaledOutHeight,
    devInfos->targetFramerate = targetFramerate,
    devInfos->fd = -1,
    devInfos->isTC358743 = isTC358743,
    devInfos->isThermalCamera = isThermalCamera;
  return 0;
}

int main(int argc, char **argv) {
  devInfoMain = (devInfo*)calloc(1, sizeof(*devInfoMain));
  devInfoAlt = (devInfo*)calloc(1, sizeof(*devInfoAlt));
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <V4L2 main device> <V4L2 alt device>\n\nExample: %s /dev/video0 /dev/video1\n", argv[0], argv[0]);
    return 1;
  }
  init_vars(devInfoMain, 2, 640, 360, 10, true, true, argv[1]);
  init_vars(devInfoAlt, 2, 640, 360, 10, true, true, argv[2]);
  fprintf(stderr, "Starting V4L2 capture program using device(s): %s %s\n", devInfoMain->device, devInfoAlt->device);
  init_dev(buffersMain, devInfoMain);
  init_dev(buffersAlt, devInfoAlt);
  while (true) {
    get_frame(buffersMain, devInfoMain, CHEAP_CONVERTER_BOX);
    //get_frame(buffersMain, devInfoMain, EXPENSIVE_CONVERTER_BOX);
    //get_frame(buffersAlt, devInfoAlt, CHEAP_CONVERTER_BOX);
    //get_frame(buffersAlt, devInfoAlt, EXPENSIVE_CONVERTER_BOX);
  }
  deinit_bufs(buffersMain, devInfoMain);
  deinit_bufs(buffersAlt, devInfoAlt);
  return 0;
}