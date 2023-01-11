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
 *    Usage: ./v4l2-video-capture-testing-program | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -i pipe:0
 *    This will run the main program and then output the processed video data to FFPlay to test the processed frames
 *
 */
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

int fd = -1;
void* handle;
char* device;
int start_main(int fd, void* handle, char *device_name, const int);
int run_loop(int fd, void* handle, char* device_name);
int stop_main(int fd, void* handle, char* device_name);

int main(int argc, char **argv) {
  device = (char*)calloc(64, sizeof(char));
  strcpy(device, "/dev/video2");
  start_main(fd, handle, device, 2);
  run_loop(fd, handle, device);
  stop_main(fd, handle, device);
  return 0;
}