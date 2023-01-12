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
#include <thread>
#include <atomic>
#include <dlfcn.h>

using namespace std;
//std::atomic<bool> running;
//unsigned char *outputFrameGreyscale, *outputGreyscaleAlt;
extern unsigned char* outputFrameGreyscale;
extern std::mutex data_mutex;

void (*start_main)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera);
void (*start_alt)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera);

// Declare the function in the shared library
//extern "C" void update_variable(unsigned char** variable_address);


int main(int argc, char **argv) {
  int fd = -1, fdAlt = -1;
  char *device, *deviceAlt;
  outputFrameGreyscale = (unsigned char*)malloc(1280 * 720 * sizeof(unsigned char));
  memset(outputFrameGreyscale, 0, (1280 * 720));
  void *handle;
  fprintf(stderr, "[main] Attempting to load: libv4l2cap.so\n");
  char* error;
  handle = dlopen("libv4l2cap.so", RTLD_NOW);
  if (!handle) {
    fprintf(stderr, "[main] %s\n", dlerror());
    exit(1);
  }
  fprintf(stderr, "[main] Successfully loaded: libv4l2cap.so\n");
  dlerror(); // Clear any existing error
  start_main = (void(*)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera)) dlsym(handle, "start_main");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "[main] %s\n", error);
    exit(1);
  }
  start_alt = (void(*)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera)) dlsym(handle, "start_main");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "[main] %s\n", error);
    exit(1);
  }
  fprintf(stderr, "[main] Successfully imported functions from: libv4l2cap.so\n");

  // Initialize device names
  device = (char*)calloc(64, sizeof(char));
  strcpy(device, "/dev/video2");
  deviceAlt = (char*)calloc(64, sizeof(char));
  strcpy(deviceAlt, "/dev/video3");
  // Start streaming thread(s)
  //std::mutex data_mutex;
  //std::mutex data_mutex_alt;
  // Pass the address of the variable to the function
  //update_variable(&outputGreyscale);
  // outputFrameGreyscale
  std::thread thread1(start_main, fd, device, 2, 640, 360, 15, true, true);
  usleep(50 * 10000);
  std::thread thread2(start_alt, fdAlt, deviceAlt, 2, 640, 360, 15, true, false);
  thread1.detach();
  thread2.detach();
  usleep(50 * 10000);
  fprintf(stderr, "\n[main] Started threads for devices\n");
  while (true) {
    //usleep(250 * 10000);
    std::unique_lock<std::mutex> lock(data_mutex);
    // access the modified data
    //update_variable(&outputGreyscale);
    int status = write(1, outputFrameGreyscale, (640 * 360));
    if (status == -1)
      perror("write");
    //fprintf(stderr, "\n[main] looping..\n");
  }
  // Cleanup imported shared library
  dlclose(handle);
  return 0;
}