#ifndef _V4L2_CTL_H
#define _V4L2_CTL_H

/*#ifdef ANDROID
#include <android-config.h>
#else
#include <config.h>
#endif*/

#include <string>

#include <linux/videodev2.h>

#ifndef NO_LIBV4L2
#include <libv4l2.h>
#else
#define v4l2_open(file, oflag, ...) (-1)
#define v4l2_close(fd) (-1)
#define v4l2_ioctl(fd, request, ...) (-1)
#define v4l2_mmap(start, length, prot, flags, fd, offset) (MAP_FAILED)
#define v4l2_munmap(_start, length) (-1)
#endif

extern unsigned capabilities;
extern unsigned out_capabilities;
extern unsigned priv_magic;
extern unsigned out_priv_magic;
extern bool is_multiplanar;
extern __u32 vidcap_buftype;
extern __u32 vidout_buftype;
extern int verbose;

typedef struct {
	unsigned flag;
	const char* str;
} flag_def;

// fmts specified
#define FmtWidth		(1L<<0)
#define FmtHeight		(1L<<1)
#define FmtChromaKey		(1L<<2)
#define FmtGlobalAlpha		(1L<<3)
#define FmtPixelFormat		(1L<<4)
#define FmtLeft			(1L<<5)
#define FmtTop			(1L<<6)
#define FmtField		(1L<<7)
#define FmtColorspace		(1L<<8)
#define FmtYCbCr		(1L<<9)
#define FmtQuantization		(1L<<10)
#define FmtFlags		(1L<<11)
#define FmtBytesPerLine		(1L<<12)
#define FmtXferFunc		(1L<<13)

void edid_cmd(int ch, char* optarg);
void edid_set(int fd);
void edid_get(int fd);

/* v4l2-ctl-modes.cpp */
bool calc_cvt_modeline(int image_width, int image_height, int refresh_rate, int reduced_blanking, bool interlaced, bool reduced_fps, struct v4l2_bt_timings* cvt);

bool calc_gtf_modeline(int image_width, int image_height, int refresh_rate, bool reduced_blanking, bool interlaced, struct v4l2_bt_timings* gtf);
#endif