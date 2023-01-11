#ifndef V4L2CAP_H_INCLUDED
#define V4L2CAP_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
  void start_main(int fd, char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, unsigned char* outputFrameGreyscale);
#ifdef __cplusplus
}
#endif

#endif // V4L2CAP_H_INCLUDED