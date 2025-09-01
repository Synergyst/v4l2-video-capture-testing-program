// v4l2_streamer.cpp
// Notes in brief:
// - Stream RAW RGB24 or MJPEG with lazy-send.
// - Optional BGR->RGB swap if capture ordering is BGR (via --bgr).
// - Encoder thread pool for MJPEG.
// - CRT filter condensed into helpers (apply to stale frames).
// - RGB static "no-signal" generator that inherits resolution/framerate from devInfoMain.
// - Dedicated recovery thread handles V4L2 select timeouts: deinit/reinit with specified delays.
// - Secondary loop outputs "no-signal" frames while capture is down.
// - Memory leak fixes preserved; deinit and free helpers intact.
#include <cmath>
#include <vector>
#include <chrono>
#include <cstdio>
#include <future>
#include <popt.h>
#include <atomic>
#include <string>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <cassert>
#include <unistd.h>
#include <stdexcept>
#include <libv4l2.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <jpeglib.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>
#include <optional>
#include <system_error>
// CRT filter
#include "crt_filter.h"
// Vignette filter
#include "vignette_filter.h"
// Gaussian blur filter (fast)
#include "gaussian_blur_fast.hpp"
// Gaussian blur filter (fast)
#include "gaussian_blur_neon.hpp"
// Stack blur filter (faster)
#include "stack_blur_rgb24.hpp"
// PNG loader
#include "png_loader.h"
// --------- V4L2 "v4l2-ctl" equivalents for HDMI setup (EDID/DV/Format/Status) ---------
#ifndef VIDIOC_LOG_STATUS
#define VIDIOC_LOG_STATUS _IO('V', 70)
#endif
#include <dirent.h>
#include <sys/stat.h>
#include <glob.h>
using namespace std;
#define CLEAR(x) memset(&(x), 0, sizeof(x))
// Globals/config
int byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, numPixels = defaultWidth * defaultHeight;
double allDevicesTargetFramerate = 240;
bool isDualInput = false;
std::atomic<bool> shouldLoop;          // true => main capture loop active; false => no-signal loop active
std::atomic<bool> programRunning{true};
std::future<int> background_task_cap_main;
std::future<int> background_task_cap_alt;
std::vector<std::string> devNames;
// TCP server globals
int listenPort = 0;
int listen_fd = -1;
std::vector<int> client_fds;
// Stream encoding globals
enum StreamCodec { CODEC_RAW = 0, CODEC_MJPEG = 1 };
StreamCodec g_streamCodec = CODEC_RAW;
int g_jpeg_quality = 80;   // 1-100
bool g_inputIsBGR = false; // set by --bgr
// Lazy-send toggle
bool g_lazy_send = true;
int g_lazy_threshold = 1;
// New: encoder threading controls
int g_encode_threads = 0;       // 0 => auto
size_t g_encode_queue_max = 0;  // later relative to threads
// Latest JPEG encoded by workers
std::shared_ptr<std::vector<unsigned char>> g_latest_jpeg;
std::atomic<uint64_t> g_latest_jpeg_id{0};
// Simple synch for V4L2 calls and device (re)init sequences
static std::mutex g_v4l2_mtx;
// Types
struct buffer {
  void* start;
  size_t length;
};
struct devInfo {
  int frame_number,
    framerate,
    startingWidth,
    startingHeight,
    startingSize,
    force_format,
    fd;
  unsigned int n_buffers;
  double frameDelayMicros,
    frameDelayMillis,
    targetFrameDelayMicros,
    targetFrameDelayMillis,
    framerateDivisor,
    targetFramerate;
  bool isTC358743 = true,
    realAndTargetRatesMatch = true;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type type;
  int index;
  unsigned char* outputFrame;
  char* device;
};
// Globals for devices
struct buffer* buffersMain = nullptr;
struct buffer* buffersAlt = nullptr;
struct devInfo* devInfoMain = nullptr;
struct devInfo* devInfoAlt = nullptr;
// Helpers
void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}
int xioctl(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && errno == EINTR);
  return r;
}

// ---- New: Easily-callable V4L2 DV timings helper (matches: v4l2-ctl -d X --query-dv-timings; sleep 2; v4l2-ctl -d X --set-dv-bt-timings query)
namespace v4l2util {

// Returns std::nullopt on success, or an error message on failure.
// If outTimings is provided, it will be filled with the queried timings on success.
// sleepSeconds controls the delay between query and set (default 2 seconds).
inline std::optional<std::string>
queryAndSetDvBtTimings(const std::string& devnode, v4l2_dv_timings* outTimings = nullptr, unsigned sleepSeconds = 2) {
  int fd = open(devnode.c_str(), O_RDWR);
  if (fd < 0) {
    return std::string("open(") + devnode + ") failed: " + std::strerror(errno);
  }

  v4l2_dv_timings timings{};
  if (xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &timings) < 0) {
    int saved = errno;
    close(fd);
    return std::string("VIDIOC_QUERY_DV_TIMINGS failed: ") + std::strerror(saved);
  }

  if (outTimings) {
    *outTimings = timings;
  }

  if (sleepSeconds > 0) {
    std::this_thread::sleep_for(std::chrono::seconds(sleepSeconds));
  }

  if (xioctl(fd, VIDIOC_S_DV_TIMINGS, &timings) < 0) {
    int saved = errno;
    close(fd);
    return std::string("VIDIOC_S_DV_TIMINGS failed: ") + std::strerror(saved);
  }

  close(fd);
  return std::nullopt;
}

} // namespace v4l2util

// Expand "~" in paths (simple HOME-based expansion)
static std::string expand_user_path(const char* p) {
  if (!p || !*p) return std::string();
  std::string s(p);
  if (s[0] == '~') {
    const char* home = getenv("HOME");
    if (home && *home) s = std::string(home) + s.substr(1);
  }
  return s;
}
// Tag formatter and device index parsing for /dev/videoN
static void make_tag(char* out, size_t cap, const char* base, int idx) {
  if (idx >= 0) snprintf(out, cap, "[%s%d]", base, idx);
  else snprintf(out, cap, "[%s]", base);
}
static int parse_video_index_from_path(const char* dev) {
  if (!dev) return -1;
  size_t len = strlen(dev);
  if (len == 0) return -1;
  int end = (int)len - 1;
  int start = end;
  while (start >= 0 && isdigit((unsigned char)dev[start])) start--;
  start++;
  if (start <= end) {
    const char* p = strstr(dev, "video");
    if (p) {
      return atoi(dev + start);
    }
    return atoi(dev + start);
  }
  return -1;
}
static bool read_file_binary(const std::string& path, std::vector<uint8_t>& out, int dev_index = -1) {
  out.clear();
  FILE* f = fopen(path.c_str(), "rb");
  char tag[32]; make_tag(tag, sizeof(tag), "edid", dev_index);
  if (!f) {
    fprintf(stderr, "%s Failed to open EDID file: %s (%s)\n", tag, path.c_str(), strerror(errno));
    return false;
  }
  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  if (len < 0) { fclose(f); fprintf(stderr, "%s ftell failed\n", tag); return false; }
  fseek(f, 0, SEEK_SET);
  out.resize((size_t)len);
  size_t rd = fread(out.data(), 1, (size_t)len, f);
  fclose(f);
  if (rd != (size_t)len) {
    fprintf(stderr, "%s Read size mismatch for %s\n", tag, path.c_str());
    out.clear();
    return false;
  }
  return true;
}
// Fix EDID checksum for each 128-byte block
static void fix_edid_checksums_inplace(std::vector<uint8_t>& edid, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "edid", dev_index);
  if (edid.size() % 128 != 0) {
    fprintf(stderr, "%s Warning: EDID size (%zu) is not a multiple of 128, cannot fix checksums\n", tag, edid.size());
    return;
  }
  const size_t blocks = edid.size() / 128;
  for (size_t b = 0; b < blocks; ++b) {
    uint8_t sum = 0;
    uint8_t* blk = edid.data() + b * 128;
    for (int i = 0; i < 127; ++i) sum = (uint8_t)(sum + blk[i]);
    blk[127] = (uint8_t)(0x100 - sum); // so that total % 256 == 0
  }
}
static int open_video_node_rw(const char* dev, int dev_index = -1) {
  int fd = open(dev, O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    char tag[32]; make_tag(tag, sizeof(tag), "v4l2", dev_index);
    fprintf(stderr, "%s Cannot open '%s': %d, %s\n", tag, dev, errno, strerror(errno));
  }
  return fd;
}
// 1) List /dev/video* and query caps (like: v4l2-ctl --list-devices plus quick info)
static void list_video_nodes_and_caps() {
  glob_t g;
  if (glob("/dev/video*", 0, nullptr, &g) != 0) {
    fprintf(stderr, "[list] No /dev/video* nodes\n");
    return;
  }
  for (size_t i = 0; i < g.gl_pathc; ++i) {
    const char* dev = g.gl_pathv[i];
    struct stat st{};
    if (stat(dev, &st) != 0 || !S_ISCHR(st.st_mode)) continue;
    int idx = parse_video_index_from_path(dev);
    char tag[32];
    if (idx >= 0) snprintf(tag, sizeof(tag), "[list%d]", idx);
    else snprintf(tag, sizeof(tag), "[list]");
    int fd = open_video_node_rw(dev, idx);
    if (fd < 0) { continue; }
    struct v4l2_capability cap;
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      fprintf(stderr, "%s %s: driver=%s, card=%s, bus=%s, version=%u.%u.%u, caps=0x%08x\n",
              tag, dev, (const char*)cap.driver, (const char*)cap.card, (const char*)cap.bus_info,
              (cap.version >> 16) & 0xFF, (cap.version >> 8) & 0xFF, cap.version & 0xFF,
              cap.capabilities);
    } else {
      fprintf(stderr, "%s %s: VIDIOC_QUERYCAP failed: %s\n", tag, dev, strerror(errno));
    }
    close(fd);
  }
  globfree(&g);
}
// 2) Set EDID from file on a video node (like: v4l2-ctl --set-edid=file<path> --fix-edid-checksums)
static bool set_device_edid_from_file(const char* dev, const char* edid_path, bool fix_checksums, int dev_index = -1) {
  std::string path = expand_user_path(edid_path);
  std::vector<uint8_t> edid;
  char tag[32]; make_tag(tag, sizeof(tag), "edid", dev_index);
  if (!read_file_binary(path, edid, dev_index)) return false;
  if (edid.size() < 128 || (edid.size() % 128) != 0) {
    fprintf(stderr, "%s Invalid EDID size %zu (must be 128*N)\n", tag, edid.size());
    return false;
  }
  if (fix_checksums) fix_edid_checksums_inplace(edid, dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return false;
  struct v4l2_edid vedid;
  CLEAR(vedid);
  vedid.pad = 0;             // for subdevs with pads; 0 for video node
  vedid.start_block = 0;
  vedid.blocks = (uint32_t)(edid.size() / 128);
  vedid.edid = edid.data();
  int rc = xioctl(fd, VIDIOC_S_EDID, &vedid);
  if (rc < 0) {
    fprintf(stderr, "%s VIDIOC_S_EDID failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fprintf(stderr, "%s Set %u EDID block(s) on %s from %s\n", tag, vedid.blocks, dev, path.c_str());
  close(fd);
  return true;
}
// 3) Query and set DV timings to current detected timings (legacy helper; kept for compatibility)
/*static bool query_and_set_dv_timings(const char* dev, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "dvt", dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return false;
  struct v4l2_dv_timings timings;
  CLEAR(timings);
  if (xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &timings) < 0) {
    fprintf(stderr, "%s VIDIOC_QUERY_DV_TIMINGS failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fprintf(stderr, "%s Query timings: %ux%u, pixelclock=%llu\n",
          tag, timings.bt.width, timings.bt.height, (unsigned long long)timings.bt.pixelclock);
  if (xioctl(fd, VIDIOC_S_DV_TIMINGS, &timings) < 0) {
    fprintf(stderr, "%s VIDIOC_S_DV_TIMINGS failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fprintf(stderr, "%s Set DV timings to current detected timings on %s\n", tag, dev);
  close(fd);
  return true;
}*/
// 4) Print current format (like v4l2-ctl -V)
static void print_current_format(const char* dev, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "fmt", dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return;
  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    char fcc[5] = { (char)(fourcc & 0xFF), (char)((fourcc >> 8) & 0xFF), (char)((fourcc >> 16) & 0xFF), (char)((fourcc >> 24) & 0xFF), 0 };
    fprintf(stderr, "%s %s: %ux%u, pix=%s, bytesperline=%u, sizeimage=%u, field=%u, colorspace=%u\n",
            tag,
            dev,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            fcc, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage,
            fmt.fmt.pix.field, fmt.fmt.pix.colorspace);
  } else {
    fprintf(stderr, "%s VIDIOC_G_FMT failed on %s: %s\n", tag, dev, strerror(errno));
  }
  close(fd);
}
// 5) Set pixel format to RGB3 (RGB24) like: v4l2-ctl -v pixelformat=RGB3
static bool set_pixfmt_rgb3(const char* dev, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "fmt", dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return false;
  // First get current to preserve width/height
  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
    fprintf(stderr, "%s VIDIOC_G_FMT failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    fprintf(stderr, "%s VIDIOC_S_FMT(RGB24) failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fprintf(stderr, "%s %s: set pixelformat to RGB3 (RGB24)\n", tag, dev);
  close(fd);
  return true;
}
// Optionally set to UYVY instead
static bool set_pixfmt_uyvy(const char* dev, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "fmt", dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return false;
  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
    fprintf(stderr, "%s VIDIOC_G_FMT failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    fprintf(stderr, "%s VIDIOC_S_FMT(UYVY) failed on %s: %s\n", tag, dev, strerror(errno));
    close(fd);
    return false;
  }
  fprintf(stderr, "%s %s: set pixelformat to UYVY\n", tag, dev);
  close(fd);
  return true;
}
// 6) Log status (like v4l2-ctl --log-status)
static void log_device_status(const char* dev, int dev_index = -1) {
  char tag[32]; make_tag(tag, sizeof(tag), "status", dev_index);
  int fd = open_video_node_rw(dev, dev_index);
  if (fd < 0) return;
  if (xioctl(fd, VIDIOC_LOG_STATUS, NULL) < 0) {
    fprintf(stderr, "%s VIDIOC_LOG_STATUS failed on %s: %s\n", tag, dev, strerror(errno));
  } else {
    fprintf(stderr, "%s Logged driver status for %s (check dmesg/journal)\n", tag, dev);
  }
  close(fd);
}
// High-level runner replicating your script sequence for a device node
// Script steps replicated:
//   list video nodes
//   for m in 1:
//     set EDID (with fix checksums), sleep 2
//     query DV timings, sleep 2 inside helper, set DV timings to query
//     print format, sleep 1
//     set format to RGB3, sleep 1
//     log status
static void run_hdmi_setup_for_device(const char* dev_path, const char* edid_file_path, bool fix_checksums, bool set_rgb3_fmt = true, int dev_index = -1) {
  // Print available devices (equivalent to: ls /dev/video* && v4l2-ctl --list-devices)
  list_video_nodes_and_caps();
  char tag[32]; make_tag(tag, sizeof(tag), "setup", dev_index);
  if (!set_device_edid_from_file(dev_path, edid_file_path, fix_checksums, dev_index)) {
    fprintf(stderr, "%s Skipping further steps for %s due to EDID failure\n", tag, dev_path);
    return;
  }
  usleep(2 * 1000 * 1000); // sleep 2
  {
    v4l2_dv_timings t{};
    auto err = v4l2util::queryAndSetDvBtTimings(dev_path, &t, 2); // query; sleep 2; set
    if (err) {
      fprintf(stderr, "%s DV timings query/set failed for %s: %s\n", tag, dev_path, err->c_str());
    } else {
      fprintf(stderr, "%s DV timings set from current detected timings: %ux%u, pixelclock=%llu\n",
              tag, t.bt.width, t.bt.height, (unsigned long long)t.bt.pixelclock);
    }
  }
  usleep(1 * 1000 * 1000); // sleep 1
  print_current_format(dev_path, dev_index);
  usleep(1 * 1000 * 1000); // sleep 1
  if (set_rgb3_fmt) {
    set_pixfmt_rgb3(dev_path, dev_index);
  } else {
    set_pixfmt_uyvy(dev_path, dev_index);
  }
  usleep(1 * 1000 * 1000); // sleep 1
  log_device_status(dev_path, dev_index);
}

// Convenience: run for multiple device nodes
/*static void run_hdmi_setup_for_devices(const std::vector<std::string>& devs, const char* edid_file_path, bool fix_checksums, bool set_rgb3_fmt = true) {
  for (const auto& dev : devs) {
    int idx = parse_video_index_from_path(dev.c_str());
    run_hdmi_setup_for_device(dev.c_str(), edid_file_path, fix_checksums, set_rgb3_fmt, idx);
  }
}*/

static inline std::string trim_copy(const std::string& s) {
  const char* ws = " \t\r\n";
  const auto b = s.find_first_not_of(ws);
  if (b == std::string::npos) return std::string();
  const auto e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}
static inline size_t frame_bytes(const devInfo* d) {
  return (size_t)d->startingWidth * (size_t)d->startingHeight * (size_t)byteScaler;
}
static inline pair<int,int> get_resolution(const devInfo* d) {
  return { d->startingWidth, d->startingHeight };
}
static inline double frame_delay_us(const devInfo* d) {
  return d->targetFrameDelayMicros;
}
// Networking helpers
static int set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return -1;
  return 0;
}
static int setup_server_socket(int port) {
  signal(SIGPIPE, SIG_IGN);
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    perror("[net] socket");
    return -1;
  }
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (set_nonblocking(fd) < 0) {
    perror("[net] set_nonblocking(listen)");
    close(fd);
    return -1;
  }
  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)port);
  if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("[net] bind");
    close(fd);
    return -1;
  }
  if (listen(fd, 16) < 0) {
    perror("[net] listen");
    close(fd);
    return -1;
  }
  fprintf(stderr, "[net] Listening on port %d\n", port);
  return fd;
}
static bool encode_rgb24_to_jpeg_mem(const unsigned char* rgb, int width, int height, int quality, std::vector<unsigned char>& out) {
  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  unsigned char* jpegBuf = nullptr;
  unsigned long jpegSize = 0;
  jpeg_mem_dest(&cinfo, &jpegBuf, &jpegSize);
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);
  const int stride = width * 3;
  JSAMPROW row[1];
  while (cinfo.next_scanline < cinfo.image_height) {
    row[0] = (JSAMPROW)(rgb + cinfo.next_scanline * stride);
    jpeg_write_scanlines(&cinfo, row, 1);
  }
  jpeg_finish_compress(&cinfo);
  out.assign(jpegBuf, jpegBuf + jpegSize);
  jpeg_destroy_compress(&cinfo);
  free(jpegBuf);
  return true;
}
static bool send_all_or_drop(int fd, const unsigned char* data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += (size_t)n;
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}
static void seed_initial_frame_for_fd(int fd, int width, int height) {
  if (g_streamCodec == CODEC_MJPEG) {
    std::vector<unsigned char> rgb((size_t)width * (size_t)height * 3ull, 0);
    std::vector<unsigned char> jpeg;
    if (encode_rgb24_to_jpeg_mem(rgb.data(), width, height, g_jpeg_quality, jpeg) && !jpeg.empty()) {
      send_all_or_drop(fd, jpeg.data(), jpeg.size());
    }
  } else {
    size_t frame_len = (size_t)width * (size_t)height * (size_t)byteScaler;
    std::vector<unsigned char> seed(frame_len, 0);
    send_all_or_drop(fd, seed.data(), seed.size());
  }
}
static void accept_new_clients() {
  while (true) {
    sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    int cfd = accept(listen_fd, (sockaddr*)&cliaddr, &len);
    if (cfd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      perror("[net] accept");
      break;
    }
    int one = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    client_fds.push_back(cfd);
    int w = (devInfoMain ? devInfoMain->startingWidth : defaultWidth);
    int h = (devInfoMain ? devInfoMain->startingHeight : defaultHeight);
    for (int i = 0; i < 60; i++) seed_initial_frame_for_fd(cfd, w, h);
    char ip[64]; inet_ntop(AF_INET, &cliaddr.sin_addr, ip, sizeof(ip));
    fprintf(stderr, "[net] Client connected: %s:%d (fd=%d). Total clients: %zu\n",
            ip, ntohs(cliaddr.sin_port), cfd, client_fds.size());
  }
}
static void broadcast_frame(const unsigned char* data, size_t len) {
  if (client_fds.empty()) return;
  for (size_t i = 0; i < client_fds.size();) {
    int fd = client_fds[i];
    if (!send_all_or_drop(fd, data, len)) {
      fprintf(stderr, "[net] Dropping client fd=%d\n", fd);
      close(fd);
      client_fds.erase(client_fds.begin() + i);
      continue;
    }
    ++i;
  }
}
// CLI
void parse_cli_or_die(int argc, const char** argv) {
  double opt_fps = allDevicesTargetFramerate;
  const char* opt_devices = nullptr;
  int opt_port = 0;
  int opt_mjpeg = 0;
  int opt_raw = 0;
  int opt_q = g_jpeg_quality;
  int opt_bgr = 0;
  int opt_enc_threads = 0;
  int opt_lazy = 0;
  int opt_no_lazy = 0;
  int opt_lazy_thresh = 1;
  struct poptOption optionsTable[] = {
    { "fps",     'f',    POPT_ARG_DOUBLE,   &opt_fps,      0,    "Target framerate for all devices",                        "FPS" },
    { "devices", 'd',    POPT_ARG_STRING,   &opt_devices,  0,    "V4L2 video device(s): /dev/video0 or /dev/video0,/dev/video1",  "DEV[,DEV]" },
    { "port",    'p',    POPT_ARG_INT,      &opt_port,     0,    "TCP listen port for streaming frames",                    "PORT" },
    { "mjpeg",   0,      POPT_ARG_NONE,     &opt_mjpeg,    0,    "Encode and stream Motion-JPEG instead of raw RGB24",      nullptr },
    { "raw",     0,      POPT_ARG_NONE,     &opt_raw,      0,    "Force raw-RGB24 stream (default)",                         nullptr },
    { "jpeg-quality", 'q', POPT_ARG_INT,    &opt_q,        0,    "JPEG quality when --mjpeg (1-100, default 80)",           "Q" },
    { "bgr",     0,      POPT_ARG_NONE,     &opt_bgr,      0,    "Capture is BGR24; server will swap to RGB24 before sending", nullptr },
    { "encode-threads",  0, POPT_ARG_INT,   &opt_enc_threads, 0, "MJPEG encoder threads (0=auto)",                          "N" },
    { "lazy",    0,      POPT_ARG_NONE,     &opt_lazy,     0,    "Enable lazy-send (skip identical frames)",                nullptr },
    { "no-lazy", 0,      POPT_ARG_NONE,     &opt_no_lazy,  0,    "Disable lazy-send",                                       nullptr },
    { "lazy-threshold", 0, POPT_ARG_INT,    &opt_lazy_thresh, 0, "Consecutive identical frames to mark stale (default 1)",  "N" },
    { "help",    'h',    POPT_ARG_NONE,     nullptr,       'h',  "Show help and exit",                                      nullptr },
    { nullptr,   0,      0,                 nullptr,       0,    nullptr,                                                   nullptr }
  };
  poptContext pc = poptGetContext(argv[0], argc, argv, optionsTable, 0);
  int rc;
  while ((rc = poptGetNextOpt(pc)) >= 0) {}
  if (rc < -1) {
    fprintf(stderr, "[main] Error parsing options: %s: %s\n", poptBadOption(pc, POPT_BADOPTION_NOALIAS), poptStrerror(rc));
    poptPrintUsage(pc, stderr, 0);
    exit(1);
  }
  devNames.clear();
  if (opt_devices && *opt_devices) {
    std::string list(opt_devices);
    size_t start = 0;
    while (start <= list.size()) {
      size_t pos = list.find(',', start);
      std::string token = (pos == std::string::npos)
                            ? list.substr(start)
                            : list.substr(start, pos - start);
      token = trim_copy(token);
      if (!token.empty()) devNames.emplace_back(token);
      if (pos == std::string::npos) break;
      start = pos + 1;
    }
    const char* extra = nullptr;
    while ((extra = poptGetArg(pc)) != nullptr) {
      fprintf(stderr, "[main] Warning: ignoring extra device '%s' because --devices was specified\n", extra);
    }
  } else {
    const char* arg = nullptr;
    while ((arg = poptGetArg(pc)) != nullptr) {
      devNames.emplace_back(arg);
    }
  }
  poptFreeContext(pc);
  if (devNames.size() == 1) {
    isDualInput = false;
  } else if (devNames.size() == 2) {
    isDualInput = true;
  } else {
    fprintf(stderr, "[main] Usage:\n");
    fprintf(stderr, "  %s [options] --devices=</dev/video0>[,/dev/video1] --port=<PORT>\n", argv[0]);
    fprintf(stderr, "  %s [options] </dev/video0> [/dev/video1] --port=<PORT>\n", argv[0]);
    fprintf(stderr, "Options: ...\n");
    exit(1);
  }
  if (opt_port <= 0 || opt_port > 65535) {
    fprintf(stderr, "[main] Error: valid --port=<1-65535> is required\n");
    exit(1);
  }
  // Apply parsed options
  allDevicesTargetFramerate = opt_fps;
  listenPort = opt_port;
  if (opt_q < 1) opt_q = 1;
  if (opt_q > 100) opt_q = 100;
  g_jpeg_quality = opt_q;
  g_inputIsBGR = (opt_bgr != 0);
  g_streamCodec = (opt_mjpeg && !opt_raw) ? CODEC_MJPEG : CODEC_RAW;
  g_encode_threads = opt_enc_threads;
  if (opt_no_lazy) g_lazy_send = false;
  else if (opt_lazy) g_lazy_send = true;
  if (opt_lazy_thresh < 1) opt_lazy_thresh = 1;
  g_lazy_threshold = opt_lazy_thresh;
  fprintf(stderr, "[main] Parsed options: --fps=%.3f, --devices=%s%s%s, --port=%d, --stream=%s%s, --bgr=%d, --encode-threads=%d, --lazy=%d, --lazy-threshold=%d\n",
          allDevicesTargetFramerate,
          devNames[0].c_str(),
          isDualInput ? "," : "",
          isDualInput ? devNames[1].c_str() : "",
          listenPort,
          (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG"),
          (g_streamCodec == CODEC_MJPEG ? (std::string(" (q=") + std::to_string(g_jpeg_quality) + ")").c_str() : ""),
          g_inputIsBGR ? 1 : 0,
          g_encode_threads,
          g_lazy_send ? 1 : 0,
          g_lazy_threshold);
}
bool doubles_equal(double a, double b, double epsilon = 0.001) { return std::fabs(a - b) < epsilon; }
class MicroStopwatch {
  std::chrono::high_resolution_clock::time_point start_time;
public:
  void start() {
    start_time = std::chrono::high_resolution_clock::now();
  }
  double elapsedMicros() const {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> diff = now - start_time;
    return diff.count();
  }
};
// R/B swap
static inline void swap_rb_inplace(unsigned char* buf, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    unsigned char* p = buf + i * 3;
    unsigned char t = p[0];
    p[0] = p[2];
    p[2] = t;
  }
}
static inline void swap_rb_inplace_mt(unsigned char* buf, size_t pixels, int threads) {
  if (threads <= 1 || pixels < 512ull * 512ull) {
    swap_rb_inplace(buf, pixels);
    return;
  }
  threads = std::min<int>(threads, std::thread::hardware_concurrency() ? (int)std::thread::hardware_concurrency() : 2);
  size_t chunk = (pixels + threads - 1) / threads;
  std::vector<std::thread> ths;
  ths.reserve(threads);
  for (int t = 0; t < threads; ++t) {
    size_t startPix = (size_t)t * chunk;
    if (startPix >= pixels) break;
    size_t endPix = std::min(pixels, startPix + chunk);
    ths.emplace_back([buf, startPix, endPix]() {
      unsigned char* p = buf + startPix * 3;
      for (size_t i = startPix; i < endPix; ++i) {
        unsigned char tmp = p[0];
        p[0] = p[2];
        p[2] = tmp;
        p += 3;
      }
    });
  }
  for (auto& th : ths) th.join();
}
// --------------- V4L2 init/deinit/capture -----------------
int init_dev_stage1(struct devInfo*& devInfos) {
  fprintf(stderr, "[cap%d] Starting V4L2 capture testing program with the following V4L2 video device: %s\n", devInfos->index, devInfos->device);
  struct stat st;
  if (-1 == stat(devInfos->device, &st)) {
    fprintf(stderr, "[cap%d] Cannot identify '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "[cap%d] %s is no device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  devInfos->fd = open(devInfos->device, O_RDWR | O_NONBLOCK, 0);
  if (-1 == devInfos->fd) {
    fprintf(stderr, "[cap%d] Cannot open '%s': %d, %s\n", devInfos->index, devInfos->device, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "[cap%d] Opened V4L2 video device: %s\n", devInfos->index, devInfos->device);
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s is no V4L2 video device\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "[cap%d] %s is no video capture device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(devInfos->fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_CROP, &crop)) {
      // ignore
    }
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "[cap%d] Forcing format for %s to: %d ", devInfos->index, devInfos->device, devInfos->force_format);
  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "(RGB24)\n");
    } else if (devInfos->force_format == 2) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "(UYVY)\n");
    } else if (devInfos->force_format == 1) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "(GREY)\n");
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
  } else {
    if (-1 == xioctl(devInfos->fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }
  // Buggy driver paranoia
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;
  CLEAR(devInfos->req);
  devInfos->req.count = 4;
  devInfos->req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  devInfos->req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_REQBUFS, &devInfos->req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s does not support memory mapping\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }
  if (devInfos->req.count < 2) {
    fprintf(stderr, "[cap%d] Insufficient buffer memory on %s\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  return 0;
}
int init_dev_stage2(struct buffer*& buffers, struct devInfo*& devInfos) {
  if (!buffers) {
    fprintf(stderr, "[cap%d] Out of memory\n", devInfos->index);
    exit(EXIT_FAILURE);
  }
  for (devInfos->n_buffers = 0; devInfos->n_buffers < devInfos->req.count; ++devInfos->n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = devInfos->n_buffers;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");
    buffers[devInfos->n_buffers].length = buf.length;
    buffers[devInfos->n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, devInfos->fd, buf.m.offset);
    if (MAP_FAILED == buffers[devInfos->n_buffers].start)
      errno_exit("mmap");
  }
  if (devInfos->isTC358743) {
    struct v4l2_dv_timings timings;
    memset(&timings, 0, sizeof timings);
    int ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
    if (ret >= 0) {
      fprintf(stderr, "[cap%d] QUERY_DV_TIMINGS for %s: %ux%ux%d pixclk %llu\n", devInfos->index, devInfos->device, timings.bt.width, timings.bt.height, byteScaler, timings.bt.pixelclock);
      devInfos->startingWidth = timings.bt.width;
      devInfos->startingHeight = timings.bt.height;
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;
      ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
      if (ret < 0) {
        fprintf(stderr, "[cap%d] Failed to set DV timings\n", devInfos->index);
        return 1;
      } else {
        const struct v4l2_bt_timings* bt = &timings.bt;
        double tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
        double tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
        devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));
        if (devInfos->framerate < devInfos->targetFramerate) devInfos->targetFramerate = devInfos->framerate;
        devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
        devInfos->frameDelayMicros = (1000000.0 / devInfos->framerate);
        devInfos->frameDelayMillis = (1000.0 / devInfos->framerate);
        devInfos->targetFrameDelayMicros = (1000000.0 / devInfos->framerate) * devInfos->framerateDivisor;
        devInfos->targetFrameDelayMillis = (1000.0 / devInfos->framerate) * devInfos->framerateDivisor;
        int rawInputThroughput = (int)((devInfos->framerate * devInfos->startingSize) / 125000.0);
        int rawOutputThroughput = (int)(((devInfos->framerate * devInfos->startingSize) / 125000.0) / devInfos->framerateDivisor);
        devInfos->realAndTargetRatesMatch = doubles_equal(devInfos->frameDelayMicros, devInfos->targetFrameDelayMicros);
        fprintf(stderr, "[cap%d] device_name: %s, startingWidth: %d, startingHeight: %d, byteScaler: %d, startingSize: %d, framerate(actual): %u, framerateDivisor: %f, targetFramerate: %f, frameDelayMicros: %f, frameDelayMillis: %f, targetFrameDelayMicros: %f, targetFrameDelayMillis: %f, realAndTargetRatesMatch: %d\n",
          devInfos->index, devInfos->device, devInfos->startingWidth, devInfos->startingHeight, byteScaler, devInfos->startingSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, devInfos->frameDelayMicros, devInfos->frameDelayMillis, devInfos->targetFrameDelayMicros, devInfos->targetFrameDelayMillis, devInfos->realAndTargetRatesMatch);
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, rawInputThroughput: ~%dMb/~%dMiB/~%dMB/sec, rawOutputThroughput: ~%dMb/~%dMiB/~%dMB/sec\n",
          devInfos->index, devInfos->device, devInfos->isTC358743, rawInputThroughput, (int)((double)rawInputThroughput / 8.389), rawInputThroughput / 8, rawOutputThroughput, (int)((double)rawOutputThroughput / 8.389), rawOutputThroughput / 8);
      }
    } else {
      // Fallback to SD standard query if DV timings not available
      v4l2_std_id std = 0;
      ret = ioctl(devInfos->fd, VIDIOC_QUERYSTD, &std);
      if (ret >= 0) {
        ret = xioctl(devInfos->fd, VIDIOC_S_STD, &std);
        if (ret < 0) {
          fprintf(stderr, "[cap%d] Failed to set standard\n", devInfos->index);
          return 1;
        } else {
          devInfos->framerate = 25;
        }
      }
    }
  } else {
    fprintf(stderr, "[cap%d] Fatal: Only the TC358743 is supported for now. Support for general camera inputs (%s) will need to be added in the future..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
  // Queue and stream on
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf)) {
      // ignore for robustness
    }
  }
  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type)) {
    // ignore for robustness
  }
  fprintf(stderr, "[cap%d] Initialized V4L2 video device: %s\n", devInfos->index, devInfos->device);
  return 0;
}
int get_frame(struct buffer* buffers, struct devInfo* devInfos) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period: 2x frame delay
  tv.tv_sec = 0;
  tv.tv_usec = (suseconds_t)(devInfos->frameDelayMicros * 2.0);
  r = select(devInfos->fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) {
    if (EINTR == errno) {
      return 0;
    }
    errno_exit("select");
  }
  if (0 == r) {
    fprintf(stderr, "[cap%d] No V4L2 video signal, select timeout\n", devInfos->index);
    shouldLoop.store(false);
    return 1;
  }
  struct v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
    case EAGAIN:
      fprintf(stderr, "[cap%d] EAGAIN\n", devInfos->index);
      return 0;
    case EIO:
    default:
      fprintf(stderr, "%s error %d, %s\n", "VIDIOC_DQBUF", errno, strerror(errno));
      shouldLoop.store(false);
      return 1;
    }
  }
  assert(buf.index < devInfos->n_buffers);
  size_t valid = buf.bytesused ? buf.bytesused : buffers[buf.index].length;
  size_t copy_len = std::min<size_t>(valid, (size_t)devInfos->startingSize);
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, copy_len);
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
    errno_exit("VIDIOC_QBUF");
  return 0;
}
static void free_devinfo(struct devInfo*& d) {
  if (!d) return;
  if (d->outputFrame) { free(d->outputFrame); d->outputFrame = nullptr; }
  if (d->device)      { free(d->device);      d->device = nullptr; }
  free(d);
  d = nullptr;
}
int deinit_bufs(struct buffer*& buffers, struct devInfo*& devInfos) {
  if (!devInfos) return 0;
  if (buffers) {
    for (unsigned int i = 0; i < devInfos->n_buffers; ++i) {
      if (buffers[i].start && buffers[i].length) {
        if (-1 == munmap(buffers[i].start, buffers[i].length))
          errno_exit("munmap");
      }
    }
    free(buffers);
    buffers = nullptr;
  }
  fprintf(stderr, "[cap%d] Uninitialized V4L2 video device: %s\n", devInfos->index, devInfos->device);
  if (devInfos->fd >= 0) {
    if (-1 == close(devInfos->fd)) errno_exit("close");
    devInfos->fd = -1;
    fprintf(stderr, "[cap%d] Closed V4L2 video device: %s\n", devInfos->index, devInfos->device);
  }
  fprintf(stderr, "\n");
  return 0;
}
void did_memory_allocate_correctly(struct devInfo*& devInfos) {
  if (devInfos->outputFrame == NULL) {
    fprintf(stderr, "[cap%d] Fatal: Memory allocation failed of output frame for device: %s..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
}
int init_vars(struct devInfo*& devInfos, struct buffer*& bufs, const int force_format, const double targetFramerate, const bool isTC358743, const bool /*isThermalCamera*/, const char* dev_name, int index) {
  devInfos = (devInfo*)calloc(1, sizeof(*devInfos));
  devInfos->device = (char*)calloc(strlen(dev_name) + 1, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->frame_number = 0;
  devInfos->framerate = 60;
  devInfos->framerateDivisor = 1;
  devInfos->startingWidth = defaultWidth;
  devInfos->startingHeight = defaultHeight;
  devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;
  devInfos->force_format = force_format;
  devInfos->targetFramerate = targetFramerate;
  devInfos->fd = -1;
  devInfos->isTC358743 = isTC358743;
  devInfos->index = index;
  init_dev_stage1(devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  devInfos->outputFrame = (unsigned char*)calloc((size_t)devInfos->startingWidth * (size_t)devInfos->startingHeight * (size_t)byteScaler, sizeof(unsigned char));
  did_memory_allocate_correctly(devInfos);
  return 0;
}
// Reinit in-place: no free of devInfo; safe for fallback loop using devInfo fields.
int reinit_device_inplace(struct devInfo*& devInfos, struct buffer*& bufs) {
  // Stage1 establishes req.count; then allocate bufs; then stage2; ensure outputFrame sized
  init_dev_stage1(devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  size_t needed = (size_t)devInfos->startingWidth * (size_t)devInfos->startingHeight * (size_t)byteScaler;
  if (!devInfos->outputFrame) {
    devInfos->outputFrame = (unsigned char*)calloc(needed, sizeof(unsigned char));
    did_memory_allocate_correctly(devInfos);
  } else {
    // Resize if needed
    if ((size_t)devInfos->startingSize != needed) {
      free(devInfos->outputFrame);
      devInfos->outputFrame = (unsigned char*)calloc(needed, sizeof(unsigned char));
      did_memory_allocate_correctly(devInfos);
    }
  }
  devInfos->startingSize = (int)needed;
  return 0;
}
void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
  if (isDualInput) deinit_bufs(buffersAlt, devInfoAlt);
  free_devinfo(devInfoMain);
  if (isDualInput) free_devinfo(devInfoAlt);
  // Close network sockets
  for (int fd : client_fds) close(fd);
  client_fds.clear();
  if (listen_fd >= 0) { close(listen_fd); listen_fd = -1; }
}
void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt) {
  fprintf(stderr, "[main] Initializing..");
  init_vars(deviMain, bufMain, 3, allDevicesTargetFramerate, true, true, devNames[0].c_str(), 0);
  if (isDualInput) init_vars(deviAlt, bufAlt, 3, allDevicesTargetFramerate, true, true, devNames[1].c_str(), 1);
  shouldLoop.store(true);
  numPixels = devInfoMain->startingWidth * devInfoMain->startingHeight;
  usleep(1000);
}
// ----------------- MJPEG Encoder Thread Pool -----------------
struct EncodeJob {
  std::vector<unsigned char> frame;
  int width = 0;
  int height = 0;
  uint64_t id = 0;
};
static std::mutex s_enc_mtx;
static std::condition_variable s_enc_cv;
static std::deque<EncodeJob> s_enc_queue;
static std::vector<std::thread> s_enc_workers;
static std::atomic<bool> s_enc_stop{false};
static std::atomic<uint64_t> s_frame_id_counter{0};
static void encoder_worker_loop() {
  thread_local std::vector<unsigned char> tls_rgb;
  while (true) {
    EncodeJob job;
    {
      std::unique_lock<std::mutex> lk(s_enc_mtx);
      s_enc_cv.wait(lk, []{ return s_enc_stop.load() || !s_enc_queue.empty(); });
      if (s_enc_stop.load() && s_enc_queue.empty()) return;
      job = std::move(s_enc_queue.front());
      s_enc_queue.pop_front();
    }
    const unsigned char* src_ptr = job.frame.data();
    const size_t pixels = (size_t)job.width * (size_t)job.height;
    if (g_inputIsBGR) {
      tls_rgb.resize(job.frame.size());
      const unsigned char* s = job.frame.data();
      unsigned char* d = tls_rgb.data();
      for (size_t i = 0; i < pixels; ++i) {
        d[0] = s[2];
        d[1] = s[1];
        d[2] = s[0];
        s += 3; d += 3;
      }
      src_ptr = tls_rgb.data();
    }
    std::vector<unsigned char> out;
    if (!encode_rgb24_to_jpeg_mem(src_ptr, job.width, job.height, g_jpeg_quality, out)) {
      continue;
    }
    uint64_t prev = g_latest_jpeg_id.load(std::memory_order_acquire);
    while (job.id > prev) {
      if (g_latest_jpeg_id.compare_exchange_weak(prev, job.id, std::memory_order_acq_rel)) {
        std::atomic_store_explicit(&g_latest_jpeg, std::make_shared<std::vector<unsigned char>>(std::move(out)), std::memory_order_release);
        break;
      }
    }
  }
}
static void start_encoder_pool_if_needed(int width, int height) {
  if (g_streamCodec != CODEC_MJPEG) return;
  if (!g_encode_threads || g_encode_threads < 0) {
    unsigned hc = std::thread::hardware_concurrency();
    g_encode_threads = (hc > 1) ? std::max(1u, hc - 1) : 1u;
  }
  if (!s_enc_workers.empty()) return; // already started
  if (g_encode_threads > 32) g_encode_threads = 32;
  g_encode_queue_max = std::max<size_t>(g_encode_threads * 2, 2);
  fprintf(stderr, "[enc] Starting encoder pool: threads=%d, queue_max=%zu, frame=%dx%d, quality=%d, bgr=%d\n",
          g_encode_threads, g_encode_queue_max, width, height, g_jpeg_quality, g_inputIsBGR ? 1 : 0);
  s_enc_stop.store(false);
  s_enc_workers.reserve((size_t)g_encode_threads);
  for (int i = 0; i < g_encode_threads; ++i) {
    s_enc_workers.emplace_back(encoder_worker_loop);
  }
}
static void stop_encoder_pool() {
  if (s_enc_workers.empty()) return;
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    s_enc_stop.store(true);
  }
  s_enc_cv.notify_all();
  for (auto& t : s_enc_workers) t.join();
  s_enc_workers.clear();
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    s_enc_queue.clear();
  }
}
static void enqueue_encode_job(const unsigned char* frameData, size_t bytes, int width, int height) {
  EncodeJob job;
  job.frame.assign(frameData, frameData + bytes);
  job.width = width;
  job.height = height;
  job.id = s_frame_id_counter.fetch_add(1, std::memory_order_relaxed) + 1;
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    if (s_enc_queue.size() >= g_encode_queue_max) {
      s_enc_queue.pop_front();
    }
    s_enc_queue.emplace_back(std::move(job));
  }
  s_enc_cv.notify_one();
}
PNGImage png_ctx {
  .width = 1920,
  .height = 1080,
  .rgb = {}
};
// --------------- CRT helpers (condensed) -----------------
struct CRTContext {
  CRTParams params{};
  std::unique_ptr<CRTFilter> filter;
  size_t threads = 0;
  int fps = 60;
  int w = 0, h = 0;
};
static void crtctx_init_defaults(CRTContext& ctx) {
  // Example params; tweak as desired
  ctx.params.flicker_60hz = 0.904f;
  ctx.params.flicker_noise = 0.093f;
  ctx.params.scanline_strength = 0.25f; // 0..1
  ctx.params.mask_strength = 0.83f;     // 0..1
  ctx.params.grain_strength = 0.025f;   // 0..1
  ctx.params.h_warp_amp = 0.33f;        // pixels
  ctx.params.h_warp_freq_y = 0.03f;     // per-line
  ctx.params.h_warp_freq_t = 0.8f;      // per-second
  ctx.params.v_shake_amp = 1.2f;        // lines
  ctx.params.wobble_line_noise = 0.33f; // pixels
  ctx.params.block_rows = 32;           // multithread chunk
  size_t th = std::thread::hardware_concurrency();
  ctx.threads = (th == 0 ? 4 : th);
  ctx.fps = 60;
}
static void ensure_crt_filter(CRTContext& ctx, const devInfo* d) {
  auto [w, h] = get_resolution(d);
  if (!ctx.filter || ctx.w != w || ctx.h != h) {
    ctx.w = w; ctx.h = h;
    ctx.filter = std::make_unique<CRTFilter>(w, h, ctx.params, ctx.threads, ctx.fps);
  }
}
// ------------ Vignette helpers (condensed) ---------------
struct VignetteContext {
  VignetteParams params{};
  std::unique_ptr<VignetteFilter> filter;
  int w = 0, h = 0;
};

static void vignettectx_init_defaults(VignetteContext& ctx) {
  ctx.params.center_x = 0.5f;
  ctx.params.center_y = 0.5f;
  ctx.params.inner_radius = 0.073;
  ctx.params.outer_radius = 0.098f;
  ctx.params.axis_scale_x = 20.57f;
  ctx.params.axis_scale_y = 11.71f;
  ctx.params.angle_deg = 0.0f;
  ctx.params.strength = 1.85f;
  ctx.params.mode = VignetteBlendMode::Multiply; // darken
  // ctx.params.color defaults to black, gamma_correct=true, gamma=2.2
}

static void ensure_vignette_filter(VignetteContext& ctx, const devInfo* d) {
  auto [w, h] = get_resolution(d);
  if (!ctx.filter || ctx.w != w || ctx.h != h) {
    ctx.w = w; ctx.h = h;
    ctx.filter = std::make_unique<VignetteFilter>(w, h, ctx.params, /*threads=*/1);
  } else {
    // keep size, push updated parameters
    ctx.filter->setParams(ctx.params);
  }
}
// Convert BGR->RGB if needed for filter
static void make_rgb_view_for_filter(const uint8_t* src_in, bool is_bgr, size_t bytes, int w, int h, std::vector<uint8_t>& tmp_rgb, const uint8_t*& src_rgb_out) {
  if (!is_bgr) {
    src_rgb_out = src_in;
    return;
  }
  tmp_rgb.resize(bytes);
  const uint8_t* s = src_in;
  uint8_t* d = tmp_rgb.data();
  size_t pixels = (size_t)w * (size_t)h;
  for (size_t i = 0; i < pixels; ++i) {
    d[0] = s[2];
    d[1] = s[1];
    d[2] = s[0];
    s += 3; d += 3;
  }
  src_rgb_out = tmp_rgb.data();
}
// Apply CRT and send via RAW or MJPEG
static void apply_crt_and_broadcast(CRTContext& ctx, const devInfo* d, const uint8_t* src_bgr_or_rgb, bool is_bgr) {
  ensure_crt_filter(ctx, d);
  auto [w, h] = get_resolution(d);
  size_t bytes = (size_t)w * (size_t)h * 3ull;
  const uint8_t* src_rgb = nullptr;
  std::vector<uint8_t> tmp_rgb;
  make_rgb_view_for_filter(src_bgr_or_rgb, is_bgr, bytes, w, h, tmp_rgb, src_rgb);
  std::vector<uint8_t> filtered_rgb;
  filtered_rgb.reserve(bytes);
  ctx.filter->apply(src_rgb, filtered_rgb); // produces RGB24
  if (g_streamCodec == CODEC_MJPEG) {
    std::vector<unsigned char> jpeg;
    if (encode_rgb24_to_jpeg_mem(filtered_rgb.data(), w, h, g_jpeg_quality, jpeg) && !jpeg.empty()) {
      broadcast_frame(jpeg.data(), jpeg.size());
    }
  } else {
    broadcast_frame(filtered_rgb.data(), filtered_rgb.size());
  }
}
// Send current frame (changed frame path)
static void send_current_frame(const devInfo* d) {
  const size_t bytes = frame_bytes(d);
  if (g_streamCodec == CODEC_MJPEG) {
    enqueue_encode_job(d->outputFrame, bytes, d->startingWidth, d->startingHeight);
    auto sp = std::atomic_load_explicit(&g_latest_jpeg, std::memory_order_acquire);
    if (sp && !sp->empty()) broadcast_frame(sp->data(), sp->size());
  } else {
    if (g_inputIsBGR) {
      size_t pixels = (size_t)d->startingWidth * (size_t)d->startingHeight;
      swap_rb_inplace_mt(d->outputFrame, pixels, std::max(1, g_encode_threads));
    }
    broadcast_frame(d->outputFrame, bytes);
  }
}
// --------------- RGB static "no-signal" generator ---------------
static inline uint32_t xorshift32(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}
/*static void generate_rgb_static_frame(const devInfo* d, uint64_t frame_index, std::vector<uint8_t>& out_rgb24) {
  auto [w, h] = get_resolution(d);
  out_rgb24.resize((size_t)w * (size_t)h * 3ull);
  uint32_t seed = (uint32_t)((frame_index * 1664525u) ^ 1013904223u) ^ (uint32_t)w ^ ((uint32_t)h << 1);
  uint8_t* p = out_rgb24.data();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      uint32_t s = seed ^ (uint32_t)(x * 374761393u + y * 668265263u);
      uint8_t r = (uint8_t)(xorshift32(s) & 0xFF);
      uint8_t g = (uint8_t)(xorshift32(s) & 0xFF);
      uint8_t b = (uint8_t)(xorshift32(s) & 0xFF);
      p[0] = r; p[1] = g; p[2] = b;
      p += 3;
    }
  }
}
static void broadcast_rgb24_buffer(const devInfo* d, const std::vector<uint8_t>& rgb) {
  auto [w, h] = get_resolution(d);
  if (g_streamCodec == CODEC_MJPEG) {
    std::vector<unsigned char> jpeg;
    if (encode_rgb24_to_jpeg_mem(rgb.data(), w, h, g_jpeg_quality, jpeg) && !jpeg.empty()) {
      broadcast_frame(jpeg.data(), jpeg.size());
    }
  } else {
    broadcast_frame(rgb.data(), rgb.size());
  }
}*/
// --------------- Recovery thread ---------------
static std::thread g_recover_thread;
static void recovery_worker() {
  // This dedicated thread handles reinit when shouldLoop==false
  while (programRunning.load()) {
    if (shouldLoop.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    // No-signal state: perform deinit/reinit cycles with specified delays.
    // 0.5s before deinit, then deinit, perform HDMI/EDID/DV config while device is idle,
    // then reinit, then 0.5s delay BEFORE probing get_frame.
    usleep(500000);
    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      if (devInfoMain) {
        deinit_bufs(buffersMain, devInfoMain);
      }
    }

    // Perform HDMI/EDID/DV/format setup while device is closed (avoids EBUSY on S_DV_TIMINGS/S_FMT)
    const char* edid_path = "/root/v4l2-video-capture-testing-program/customgoodhdmiedid_binary.txt";
    int idx = devInfoMain ? devInfoMain->index : 0;
    run_hdmi_setup_for_device(devInfoMain ? devInfoMain->device : "/dev/video0", edid_path, true, true, idx);

    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      // Reinit in place (do not free devInfoMain, preserve device path and allocation pattern)
      reinit_device_inplace(devInfoMain, buffersMain);
    }
    usleep(500000);
    // Probe one get_frame; if success => signal restored; else loop again
    int probe_rc = 1;
    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      if (devInfoMain && buffersMain) {
        probe_rc = get_frame(buffersMain, devInfoMain);
      }
    }
    if (probe_rc == 0) {
      fprintf(stderr, "[recovery] Signal restored. Resuming primary capture loop.\n");
      shouldLoop.store(true);
    } else {
      fprintf(stderr, "[recovery] Still no signal; will retry reinit cycle.\n");
      // Loop continues.
    }
  }
}
// In-place Gaussian blur on RGB24 buffer (width*height*3).
// - buf: pointer to RGB24 pixels
// - width, height: frame dimensions
// - sigma: blur strength in pixels (<=0 => no-op). Typical range: 0.5 .. 5.0
// - strideBytes: bytes per row; 0 means tightly packed (width * 3)
// - iterations: apply multiple times (e.g., 2 for stronger blur with same sigma)
static void applyGaussianBlurRGB24(uint8_t* buf, int width, int height, float sigma, size_t strideBytes = 0, int iterations = 1) {
  if (!buf || width <= 0 || height <= 0 || sigma <= 0.f || iterations <= 0) return;

  const size_t stride = (strideBytes == 0) ? (size_t)width * 3u : strideBytes;

  // Build 1D Gaussian kernel
  const int radius = std::max(1, (int)std::ceil(3.f * sigma));   // ~99.7% coverage
  const int ksize = 2 * radius + 1;
  std::vector<float> kernel(ksize);
  {
    const float inv2s2 = 1.f / (2.f * sigma * sigma);
    float sum = 0.f;
    for (int i = -radius; i <= radius; ++i) {
      float w = std::exp(- (i * i) * inv2s2);
      kernel[i + radius] = w;
      sum += w;
    }
    // Normalize
    for (float& v : kernel) v /= sum;
  }

  std::vector<uint8_t> tmp((size_t)height * stride); // intermediate buffer

  auto clampi = [](int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  };

  for (int it = 0; it < iterations; ++it) {
    // Horizontal pass: src = buf, dst = tmp
    for (int y = 0; y < height; ++y) {
      const uint8_t* srcRow = buf + (size_t)y * stride;
      uint8_t* dstRow = tmp.data() + (size_t)y * stride;

      for (int x = 0; x < width; ++x) {
        // Accumulate per channel
        float accR = 0.f, accG = 0.f, accB = 0.f;
        for (int k = -radius; k <= radius; ++k) {
          const int xi = clampi(x + k, 0, width - 1);
          const float w = kernel[k + radius];
          const uint8_t* p = srcRow + (size_t)xi * 3u;
          accR += w * p[0];
          accG += w * p[1];
          accB += w * p[2];
        }
        uint8_t* q = dstRow + (size_t)x * 3u;
        q[0] = (uint8_t)std::lround(std::clamp(accR, 0.f, 255.f));
        q[1] = (uint8_t)std::lround(std::clamp(accG, 0.f, 255.f));
        q[2] = (uint8_t)std::lround(std::clamp(accB, 0.f, 255.f));
      }
    }

    // Vertical pass: src = tmp, dst = buf
    for (int y = 0; y < height; ++y) {
      uint8_t* dstRow = buf + (size_t)y * stride;
      for (int x = 0; x < width; ++x) {
        float accR = 0.f, accG = 0.f, accB = 0.f;
        for (int k = -radius; k <= radius; ++k) {
          const int yi = clampi(y + k, 0, height - 1);
          const float w = kernel[k + radius];
          const uint8_t* p = tmp.data() + (size_t)yi * stride + (size_t)x * 3u;
          accR += w * p[0];
          accG += w * p[1];
          accB += w * p[2];
        }
        uint8_t* q = dstRow + (size_t)x * 3u;
        q[0] = (uint8_t)std::lround(std::clamp(accR, 0.f, 255.f));
        q[1] = (uint8_t)std::lround(std::clamp(accG, 0.f, 255.f));
        q[2] = (uint8_t)std::lround(std::clamp(accB, 0.f, 255.f));
      }
    }
  }
}
// ----------------- main -----------------
int main(const int argc, char** argv) {
  // Parse options and devices
  parse_cli_or_die(argc, (const char**)argv);
  load_png_rgb24("/root/v4l2-video-capture-testing-program/nosignal.png", png_ctx, false);
  // Init devices
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt);
  // Setup TCP server
  listen_fd = setup_server_socket(listenPort);
  if (listen_fd < 0) {
    fprintf(stderr, "[main] Failed to create TCP listening socket on port %d\n", listenPort);
    cleanup_vars();
    return 1;
  }
  // Start encoder pool if MJPEG mode
  if (g_streamCodec == CODEC_MJPEG) {
    start_encoder_pool_if_needed(devInfoMain->startingWidth, devInfoMain->startingHeight);
  }
  // Start recovery thread
  g_recover_thread = std::thread(recovery_worker);
  // Warm-up: ignore a short burst of frames
  for (int i = 0; i < 60; i++) {
    accept_new_clients();
    std::lock_guard<std::mutex> lk(g_v4l2_mtx);
    if (get_frame(buffersMain, devInfoMain) != 0) break;
  }
  fprintf(stderr, "[main] Starting main loops (stream=%s%s, inputIsBGR=%d, lazy=%d)\n", (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG"), (g_streamCodec == CODEC_MJPEG ? (std::string(", q=") + std::to_string(g_jpeg_quality)).c_str() : ""), g_inputIsBGR ? 1 : 0, g_lazy_send ? 1 : 0);
  // Lazy-send state
  std::vector<unsigned char> prev_raw_frame;
  bool have_prev_frame = false;
  int consecutive_same_count = 0;
  // CRT context
  CRTContext crt;
  crtctx_init_defaults(crt);
  // CRT context
  VignetteContext vignette;
  vignettectx_init_defaults(vignette);
  ensure_vignette_filter(vignette, devInfoMain);
  vignette.filter->apply(png_ctx.rgb.data(), png_ctx.rgb);
  applyGaussianBlurRGB24_neon_inplace(png_ctx.rgb.data(), png_ctx.width, png_ctx.height, 14.5f);
  // Timing helpers
  MicroStopwatch sw;
  // OUTER loop: alternate between primary capture loop and no-signal loop
  while (programRunning.load()) {
    // Primary capture loop
    while (programRunning.load() && shouldLoop.load()) {
      accept_new_clients();
      if (!devInfoMain->realAndTargetRatesMatch) sw.start();
      int rc = 0;
      {
        std::lock_guard<std::mutex> lk(g_v4l2_mtx);
        rc = get_frame(buffersMain, devInfoMain);
      }
      if (rc != 0) {
        // get_frame sets shouldLoop=false on select timeout or error
        break;
      }
      const size_t frame_bytes_cnt = frame_bytes(devInfoMain);
      bool frame_changed = true;
      bool is_same = false;
      if (g_lazy_send && have_prev_frame && prev_raw_frame.size() == frame_bytes_cnt) {
        is_same = (std::memcmp(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes_cnt) == 0);
      }
      if (!have_prev_frame) {
        prev_raw_frame.resize(frame_bytes_cnt);
        std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes_cnt);
        have_prev_frame = true;
        frame_changed = true;
        consecutive_same_count = 0;
      } else {
        if (is_same) {
          frame_changed = false;
          consecutive_same_count++;
        } else {
          frame_changed = true;
          consecutive_same_count = 0;
          if (prev_raw_frame.size() == frame_bytes_cnt) {
            std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes_cnt);
          } else {
            prev_raw_frame.resize(frame_bytes_cnt);
            std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes_cnt);
          }
        }
      }
      // Stale frame path with CRT filter once threshold reached
      if (g_lazy_send && !frame_changed) {
        if (consecutive_same_count >= g_lazy_threshold) {
          applyGaussianBlurRGB24_neon_inplace(devInfoMain->outputFrame, devInfoMain->startingWidth, devInfoMain->startingHeight, 2);
          //vignette.filter->apply(devInfoMain->outputFrame, devInfoMain->outputFrame);
          apply_crt_and_broadcast(crt, devInfoMain, devInfoMain->outputFrame, g_inputIsBGR);
        }
      } else {
        // Changed frame path
        send_current_frame(devInfoMain);
      }
      if (!devInfoMain->realAndTargetRatesMatch) {
        double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
        if (us > 0) usleep((useconds_t)us);
      }
    }
    // No-signal loop: output RGB static placeholder until recovery thread flips shouldLoop=true
    //uint64_t static_frame_idx = 0;
    while (programRunning.load() && !shouldLoop.load()) {
      accept_new_clients();
      apply_crt_and_broadcast(crt, devInfoMain, png_ctx.rgb.data(), false);
      // Pace according to target frame delay
      double us = frame_delay_us(devInfoMain);
      if (us < 1000.0) us = 1000.0; // safety minimum 1ms
      usleep((useconds_t)us);
    }
  }
  // Shutdown
  programRunning.store(false);
  if (g_recover_thread.joinable()) g_recover_thread.join();
  usleep(1000000);
  stop_encoder_pool();
  cleanup_vars();
  return 0;
}
