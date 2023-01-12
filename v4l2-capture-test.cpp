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
#include <chrono>
#include <atomic>
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

using namespace std;
//std::atomic<bool> running;
extern unsigned char *outputFrameGreyscale, *outputFrameGreyscaleAlt;
std::mutex data_mutex;
std::condition_variable cv;
bool ready = false;

void (*start_main)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera);
void (*start_alt)(int fd, const char* device_name, const int force_format, const int scaledOutWidth, const int scaledOutHeight, const int targetFramerate, const bool isTC358743, const bool isThermalCamera);

int main(int argc, char **argv) {
  int fd = -1, fdAlt = -1;
  char *device, *deviceAlt, *error;
  void *handle;
  fprintf(stderr, "[main] Attempting to load: libv4l2cap.so\n");
  handle = dlopen("libv4l2cap.so", RTLD_NOW);
  if (!handle) {
    fprintf(stderr, "[main] %s\n", dlerror());
    exit(1);
  }
  fprintf(stderr, "[main] Successfully loaded: libv4l2cap.so\n");
  dlerror(); // Clear any existing error
  /*
   * Load start_main function from dynamic shared library file(libv4l2cap.so) as start_mainand start_alt
   * We will pass our parameters to the library for each device, so importing the same function again under a different name should be fine and seems to work too
   * The different camera types (thermal/LWIR, visible/VIS, and night vision/NIR/UV/etc wavelength cameras will have their own switch statement in the loop which runs in libv4l2cap.so)
   */
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
  // Initialize V4L2 device names to pass to [cap] thread(s) next
  device = (char*)calloc(64, sizeof(char));
  strcpy(device, "/dev/video2");
  deviceAlt = (char*)calloc(64, sizeof(char));
  strcpy(deviceAlt, "/dev/video3");
  // Start the [cap] streaming thread(s) and run it/them in the background (TODO: have a way to choose between dual and single-camera modes, for now we will assume stereo is preferred so we start both)
  std::thread thread1(start_main, fd, device, 2, 640, 360, 6, true, true);
  // thread1 has started
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  thread1.detach();
  std::thread thread2(start_alt, fdAlt, deviceAlt, 2, 640, 360, 6, true, false);
  // thread2 has started
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  thread2.detach();
  fprintf(stderr, "\n[main] Started threads for [cap] devices, starting loop in [main] now..\n");
  // The [main] loop where we will eventually process the frame data from [cap] thread(s)
  while (true) {
    /* 
     * TODO: Wait for and then pause the background loop(s) so we can access the data they stored into outputFrameGreyscale
     * 
     * Psuedo-code: 
     * waitForThenPause.thread1();
     * 
     * thread1 is at the end of the loop and done storing the image frame to outputFrameGreyscale,
     * the [cap] loop in thread1 will pause. Frames in the [cap] loop are accessed from the capture hardware using DMA,
     * so while [main] is working on processing the frame the [cap] loop should continue just fine once [main] resumes the [cap] thread which was paused.
     * 
     * The same as described happens above with thread2, though with outputFrameGreyscaleAlt instead of outputFrameGreyscale of course.
     * 
     * Psuedo-code: 
     * waitFor.thread2();
     */
    
    /*
     * TODO: Process frame data from[cap] thread(s)
     * We will need to determine if we are running in single or stereo camera mode
     * If we are running in stereo camera mode then we will need to process the frame data from each camera to account for lens warping/fisheye effect, etc
     * Once the frame data is processed for each camera then we need to know where our 2 cameras are physically in the real world before we can calibrate and test for things like depth/etc using OpenCV
     * blah blah blah.. this is really not a priority as I still need to know how to fix the issue above
     * TODO: write a more detailed comment area for this section later when it becomes the priority. ;)
     */
    
    // Write the scaled down contents of outputFrameGreyscale to stdout, this variable can hold up to the full resolution (which is a known-value of 1280x720 during development), though we scale this down in the [cap] thread to save on resources
    int status = write(1, outputFrameGreyscale, (640 * 360));
    if (status == -1)
      perror("write");
    // Zero out frame buffers for the size of the original resolution (this can be improved of course later, the input resolution is a known-value of 1280x720 during development, so this is fine for now)
    memset(outputFrameGreyscale, 0, (1280 * 720));
    memset(outputFrameGreyscaleAlt, 0, (1280 * 720));
  }
  // Cleanup imported shared library
  dlclose(handle);
  return 0;
}