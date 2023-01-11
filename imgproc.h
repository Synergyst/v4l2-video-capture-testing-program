#ifndef IMGPROC_H_INCLUDED
#define IMGPROC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
  const int startingWidth = 1280, startingHeight = 720, scaledOutWidth = 640, scaledOutHeight = 360; // The width and height of the input video and any downscaled output video
  // Allocate memory for the input and output frames
  unsigned char* outputFrame = new unsigned char[startingWidth * startingHeight * 2];
  unsigned char* outputFrameGreyscale = new unsigned char[startingWidth * startingHeight];

  void (*frame_to_stdout)(unsigned char* input, int size);
  void (*rescale_bilinear_from_yuyv)(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height);
  void (*yuyv_to_greyscale)(const unsigned char* input, unsigned char* grey, int width, int height);
  void (*uyvy_to_greyscale)(const unsigned char* input, unsigned char* grey, int width, int height);
  void (*gaussianBlur)(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight);
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
#ifdef __cplusplus
}
#endif

#endif // IMGPROC_H_INCLUDED