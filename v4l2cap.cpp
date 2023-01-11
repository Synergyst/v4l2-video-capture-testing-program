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
#include <dlfcn.h>
#include "imgproc.h"

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
};

struct buffer {
  void* start;
  size_t length;
};

enum io_method io = IO_METHOD_MMAP;
struct buffer* buffers;
unsigned int n_buffers;

void process_image(const void* p, int size) {
  unsigned char* preP = (unsigned char*)p;
  //frame_to_stdout(preP, size);
  rescale_bilinear_from_yuyv(preP, startingWidth, startingHeight, outputFrameGreyscale, scaledOutWidth, scaledOutHeight);
  gaussianBlur(outputFrameGreyscale, scaledOutWidth, scaledOutHeight, outputFrameGreyscale, scaledOutWidth, scaledOutHeight);
  // Values from 0 to 125 gets set to 0. Then ramp 125 through to 130 to 255. Finally we should set 131 to 255 to a value of 0
  frame_to_stdout(outputFrameGreyscale, (scaledOutWidth * scaledOutHeight));
  memset(outputFrameGreyscale, 0, startingWidth * startingHeight * sizeof(unsigned char));
  //croppedWidth = 0, croppedHeight = 0;
}

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

int read_frame(int fd) {
  struct v4l2_buffer buf;
  unsigned int i;
  switch (io) {
  case IO_METHOD_READ:
    if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("read");
      }
    }
    process_image(buffers[0].start, buffers[0].length);
    break;
  case IO_METHOD_MMAP:
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }
    assert(buf.index < n_buffers);
    process_image(buffers[buf.index].start, buf.bytesused);
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  case IO_METHOD_USERPTR:
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;
      case EIO:
        // Could ignore EIO, see spec.
        // fall through
      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }
    for (i = 0; i < n_buffers; ++i)
      if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length)
        break;
    assert(i < n_buffers);
    process_image((void*)buf.m.userptr, buf.bytesused);
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  }
  //frame_to_stdout(outputFrameGreyscale, (startingWidth * startingHeight));
  return 1;
}

void init_read(unsigned int buffer_size) {
  buffers = (buffer*)calloc(1, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);
  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

void init_mmap(int fd, char* device_name) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support memory mapping\n", device_name);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", device_name);
    exit(EXIT_FAILURE);
  }
  buffers = (buffer*)calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);
    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

void init_userp(unsigned int buffer_size, int fd, char* device_name) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support user pointer i/o\n", device_name);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  buffers = (buffer*)calloc(4, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = malloc(buffer_size);
    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}

int start_main(char *device_name, const int force_format /* 1 = YUYV, 2 = UYVY, 3 = RGB24 - Do not use RGB24 as it will cause high CPU usage, YUYV/UYVY should be used instead */) {
  int fd = -1;
  unsigned int i;
  enum v4l2_buf_type type;
  memset(outputFrame, 0, startingWidth * startingHeight * sizeof(unsigned char));
  memset(outputFrameGreyscale, 0, startingWidth * startingHeight * sizeof(unsigned char));
  fprintf(stderr, "Starting V4L2 capture testing program with the following V4L2 device: %s\n", device_name);

  void* handle;
  char* error;
  handle = dlopen("libimgproc.so", RTLD_LAZY);
  if (!handle) {
    fprintf(stderr, "%s\n", dlerror());
    exit(1);
  } else {
    fprintf(stderr, "Loaded libimgproc.so successfully\n");
  }
  dlerror(); // Clear any existing error
  frame_to_stdout = (void(*)(unsigned char* input, int size)) dlsym(handle, "frame_to_stdout");
  rescale_bilinear_from_yuyv = (void(*)(const unsigned char* input, int input_width, int input_height, unsigned char* output, int output_width, int output_height)) dlsym(handle, "rescale_bilinear_from_yuyv");
  yuyv_to_greyscale = (void(*)(const unsigned char* input, unsigned char* grey, int width, int height)) dlsym(handle, "yuyv_to_greyscale");
  uyvy_to_greyscale = (void(*)(const unsigned char* input, unsigned char* grey, int width, int height)) dlsym(handle, "uyvy_to_greyscale");
  gaussianBlur = (void(*)(unsigned char* input, int inputWidth, int inputHeight, unsigned char* output, int outputWidth, int outputHeight)) dlsym(handle, "gaussianBlur");
  crop_greyscale = (void(*)(unsigned char* image, int width, int height, int* crops, unsigned char* croppedImage)) dlsym(handle, "crop_greyscale");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "%s\n", error);
    exit(1);
  } else {
    fprintf(stderr, "Successfully imported functions from libimgproc.so\n");
  }

  struct stat st;
  if (-1 == stat(device_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", device_name, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no device\n", device_name);
    exit(EXIT_FAILURE);
  }
  fd = open(device_name, O_RDWR | O_NONBLOCK, 0);
  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", device_name, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "Opened V4L2 device: %s\n", device_name);

  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n", device_name);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", device_name);
    exit(EXIT_FAILURE);
  }
  switch (io) {
  case IO_METHOD_READ:
    if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
      fprintf(stderr, "%s does not support read i/o\n", device_name);
      exit(EXIT_FAILURE);
    }
    break;
  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", device_name);
      exit(EXIT_FAILURE);
    }
    break;
  }
  // Select video input, video standard and tune here.
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default
    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        // Cropping not supported.
        break;
      default:
        // Errors ignored.
        break;
      }
    }
  }
  else {
    // Errors ignored.
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "Forced format for main (%s) to: %d\n", device_name, force_format);
  if (force_format) {
    if (force_format == 3) {
      fmt.fmt.pix.width = startingWidth;
      fmt.fmt.pix.height = startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    }
    else if (force_format == 2) {
      fmt.fmt.pix.width = startingWidth;
      fmt.fmt.pix.height = startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    }
    else if (force_format == 1) {
      fmt.fmt.pix.width = startingWidth;
      fmt.fmt.pix.height = startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    }
    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  }
  else {
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }
  // Buggy driver paranoia.
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  switch (io) {
  case IO_METHOD_READ:
    init_read(fmt.fmt.pix.sizeimage);
    break;
  case IO_METHOD_MMAP:
    init_mmap(fd, device_name);
    break;
  case IO_METHOD_USERPTR:
    init_userp(fmt.fmt.pix.sizeimage, fd, device_name);
    break;
  }
  
  // TODO: Have some way of passing flags from main() so we can handle settings in this area
  struct v4l2_dv_timings timings;
  v4l2_std_id std;
  int ret;
  int fps = -1;

  memset(&timings, 0, sizeof timings);
  ret = xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
  if (ret >= 0) {
    fprintf(stderr, "QUERY_DV_TIMINGS returned %ux%u pixclk %llu\n", timings.bt.width, timings.bt.height, timings.bt.pixelclock);
    // Can read DV timings, so set them.
    ret = xioctl(fd, VIDIOC_S_DV_TIMINGS, &timings);
    if (ret < 0) {
      fprintf(stderr, "Failed to set DV timings\n");
      return -1;
    } else {
      double tot_height, tot_width;
      const struct v4l2_bt_timings* bt = &timings.bt;

      tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
      tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
      fps = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));
      fprintf(stderr, "Framerate is %u\n", fps);
    }
  } else {
    memset(&std, 0, sizeof std);
    ret = ioctl(fd, VIDIOC_QUERYSTD, &std);
    if (ret >= 0) {
      // Can read standard, so set it.
      ret = xioctl(fd, VIDIOC_S_STD, &std);
      if (ret < 0) {
        fprintf(stderr, "Failed to set standard\n");
        return -1;
      } else {
        // SD video - assume 50Hz / 25fps
        fps = 25;
      }
    }
  }
  fprintf(stderr, "Initialized V4L2 device: %s\n", device_name);

  switch (io) {
  case IO_METHOD_READ:
    // Nothing to do.
    break;
  case IO_METHOD_MMAP:
    for (i = 0; i < n_buffers; ++i) {
      struct v4l2_buffer buf;
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");
    break;
  case IO_METHOD_USERPTR:
    for (i = 0; i < n_buffers; ++i) {
      struct v4l2_buffer buf;
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long)buffers[i].start;
      buf.length = buffers[i].length;
      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");
    break;
  }
  i = 0;
  fprintf(stderr, "Started capturing from V4L2 device: %s\n", device_name);

  fprintf(stderr, "Starting loop for V4L2 device: %s\n", device_name);
  while (true) {
    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    // Timeout.
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    r = select(fd + 1, &fds, NULL, NULL, &tv);
    if (-1 == r) {
      if (EINTR == errno)
        continue;
      errno_exit("select");
    }
    if (0 == r) {
      fprintf(stderr, "select timeout\n");
      exit(EXIT_FAILURE);
    }
    read_frame(fd);
    //if (read_frame())
    //  break;
    // EAGAIN - continue select loop

    //frame_to_stdout(outputFrameGreyscale, (startingWidth * startingHeight));
  }
  //enum v4l2_buf_type type;
  switch (io) {
  case IO_METHOD_READ:
    // Nothing to do.
    break;
  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
      errno_exit("VIDIOC_STREAMOFF");
    break;
  }
  fprintf(stderr, "Stopped capturing from V4L2 device: %s\n", device_name);

  //unsigned int i;
  switch (io) {
  case IO_METHOD_READ:
    free(buffers[0].start);
    break;
  case IO_METHOD_MMAP:
    for (i = 0; i < n_buffers; ++i)
      if (-1 == munmap(buffers[i].start, buffers[i].length))
        errno_exit("munmap");
    break;
  case IO_METHOD_USERPTR:
    for (i = 0; i < n_buffers; ++i)
      free(buffers[i].start);
    break;
  }
  free(buffers);
  fprintf(stderr, "Uninitialized V4L2 device: %s\n", device_name);

  if (-1 == close(fd))
    errno_exit("close");
  fd = -1;
  fprintf(stderr, "Closed V4L2 device: %s\n", device_name);
  fprintf(stderr, "\n");
  dlclose(handle);
  return 0;
}