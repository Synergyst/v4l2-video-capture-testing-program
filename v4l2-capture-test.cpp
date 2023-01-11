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
#include <condition_variable>
#include <mutex>
#include <thread>
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
#include <dlfcn.h>

void (*start_main)(int fd, char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, unsigned char* outputFrameGreyscale, bool isTC358743, bool isThermalCamera);
void (*start_alt)(int fd, char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, unsigned char* outputFrameGreyscale, bool isTC358743, bool isThermalCamera);

int main(int argc, char **argv) {
  int fd = -1, fdAlt = -1;
  void *handle;
  char *device, *deviceAlt;
  unsigned char* outputFrameGreyscale, *outputFrameGreyscaleAlt;
  fprintf(stderr, "Attempting to load: libv4l2cap.so\n");
  char* error;
  handle = dlopen("libv4l2cap.so", RTLD_NOW);
  if (!handle) {
    fprintf(stderr, "%s\n", dlerror());
    exit(1);
  }
  fprintf(stderr, "Successfully loaded: libv4l2cap.so\n");
  dlerror(); // Clear any existing error
  start_main = (void(*)(int fd, char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, unsigned char* outputFrameGreyscale, bool isTC358743, bool isThermalCamera)) dlsym(handle, "start_main");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "%s\n", error);
    exit(1);
  }
  start_alt = (void(*)(int fd, char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, unsigned char* outputFrameGreyscale, bool isTC358743, bool isThermalCamera)) dlsym(handle, "start_main");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "%s\n", error);
    exit(1);
  }
  fprintf(stderr, "Successfully imported functions from: libv4l2cap.so\n");

  // Initialize device names
  device = (char*)calloc(64, sizeof(char));
  strcpy(device, "/dev/video2");
  deviceAlt = (char*)calloc(64, sizeof(char));
  strcpy(deviceAlt, "/dev/video2");
  // Start streaming thread(s)
  start_main(fd, device, 2, 640, 360, 6, outputFrameGreyscale, true, true);
  start_alt(fdAlt, deviceAlt, 2, 640, 360, 6, outputFrameGreyscaleAlt, true, false);
  // Cleanup imported shared library
  dlclose(handle);
  return 0;
}