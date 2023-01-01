/*
 *
 *  V4L2 video capture test program
 *  This is a modified version of the V4L2 API program, see: http://linuxtv.org/docs.php for more information
 *
 *  This is not meant to be representative of a production-ready program, it may not work without heavy modification..
 *  Lots of variables, functions, etc may be named incorrectly, have issues, etc.
 *  This file gets updated frequently with test code; please understand that a lot of parts of it may not make any sense. :)
 *
 *  Understanding that this is really meant for internal-use only/testing; feel free to modify/reuse/distribute this code in any way without restrictions.
 *
 *  Example command-line usage:
 *    nano capture.cpp ; g++ capture.cpp -std=c++20 -lv4l2 -fopenmp -o capture && time ./capture -d /dev/video0 -R -c 0 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 1280x720 -i pipe:0
 *    nano capture.cpp ; g++ capture.cpp -std=c++20 -lv4l2 -fopenmp -o capture && time ./capture -d /dev/video0 -Y -c 0 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -i pipe:0
 *    nano capture.cpp ; g++ capture.cpp -std=c++20 -lv4l2 -fopenmp -o /usr/local/bin/capture-frames && capture-frames -R -d /dev/video2 -c 0 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 435x246 -i pipe:0
 *
 */
//#include "imageproc.h"
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

char* device;
//char* devicealt;

int start_main(char *device_name);
//int start_mainalt(char *device_name);

int main(int argc, char **argv) {
  device = (char*)calloc(64, sizeof(char));
  strcpy(device, "/dev/video1");
  start_main(device);
  /*devicealt = (char*)calloc(64, sizeof(char));
  strcpy(devicealt, "/dev/video0");
  start_mainalt(devicealt);
  return 0;*/
}