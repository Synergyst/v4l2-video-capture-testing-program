// v4l2-capture-test.cpp
// Capture-side server:
// - Captures frames from V4L2 (RGB24/UYVY/GREY but network payload is RGB24 for now).
// - Encapsulates into packets and streams to TCP clients.
// - Sends DEVINFO on connect/changes, FRAME continuously, SIGNAL on up/down.
// - Retains V4L2 EDID/DV/format/recovery logic.
// - No BGR->RGB swap here; the client handles it (reported via meta flags).

#include <cmath>
#include <vector>
#include <chrono>
#include <cstdio>
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
#include <memory>
#include <optional>
#include <system_error>
#include <cctype>
#include <glob.h>
#include <dirent.h>

using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

// ---------------- Common Packet Protocol ----------------
namespace netpkt {

static inline uint64_t htonll(uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (((uint64_t)htonl((uint32_t)(v & 0xFFFFFFFFULL))) << 32) | htonl((uint32_t)(v >> 32));
#else
  return v;
#endif
}
static inline uint64_t ntohll(uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (((uint64_t)ntohl((uint32_t)(v & 0xFFFFFFFFULL))) << 32) | ntohl((uint32_t)(v >> 32));
#else
  return v;
#endif
}

static const uint32_t MAGIC = 0x56344C32; // "V4L2"
static const uint16_t PROTO_VER = 1;

enum PacketType : uint16_t {
  PT_DEVINFO  = 1,
  PT_FRAME    = 2,
  PT_HEARTBEAT= 3,
  PT_SIGNAL   = 4
};

enum CaptureSignalState : uint8_t {
  SIGNAL_UP   = 1,
  SIGNAL_DOWN = 2
};

#pragma pack(push, 1)
struct Header {
  uint32_t magic;       // MAGIC
  uint16_t version;     // PROTO_VER
  uint16_t type;        // PacketType
  uint32_t headerSize;  // header+meta bytes
  uint64_t payloadSize; // payload bytes
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DevInfoMeta {
  uint16_t dev_index;
  uint16_t byteScaler;     // 1=GREY, 2=UYVY, 3=RGB24
  uint32_t width;
  uint32_t height;
  uint32_t framerate;      // nominal
  uint32_t target_fps_x1000; // target FPS * 1000
  uint8_t  inputIsBGR;     // 0/1 (channel order from capture)
  uint8_t  isTC358743;     // 0/1
  uint8_t  streamCodec;    // 0=RAW,1=MJPEG
  uint8_t  reserved;
  uint64_t frameDelayMicros_x1000; // micros*1000
  uint32_t deviceNameLen;  // bytes follow
};
#pragma pack(pop)

#pragma pack(push, 1)
struct FrameMeta {
  uint64_t frame_id;
  uint64_t timestamp_us;
  uint32_t width;
  uint32_t height;
  uint32_t stride;       // bytes per row
  uint8_t  codec;        // 0=RAW,1=MJPEG
  uint8_t  is_bgr;       // capture ordering
  uint16_t reserved;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SignalMeta {
  uint8_t state;   // CaptureSignalState
  uint8_t reserved[7];
};
#pragma pack(pop)

} // namespace netpkt

// ---------------- Globals/config ----------------
int byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, numPixels = defaultWidth * defaultHeight;
double allDevicesTargetFramerate = 240;
bool isDualInput = false;

std::atomic<bool> shouldLoop;
std::atomic<bool> programRunning{true};

std::vector<std::string> devNames;

int listenPort = 0;
int listen_fd = -1;
std::vector<int> client_fds;

enum StreamCodec { CODEC_RAW = 0, CODEC_MJPEG = 1 };
StreamCodec g_streamCodec = CODEC_RAW;
int g_jpeg_quality = 80;
bool g_inputIsBGR = false;

int g_encode_threads = 0;

static std::mutex g_v4l2_mtx;

struct buffer { void* start; size_t length; };

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

struct buffer* buffersMain = nullptr;
struct buffer* buffersAlt = nullptr;
struct devInfo* devInfoMain = nullptr;
struct devInfo* devInfoAlt = nullptr;

// ------------- Helpers -------------
void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}
int xioctl(int fh, int request, void* arg) {
  int r;
  do { r = ioctl(fh, request, arg); } while (-1 == r && errno == EINTR);
  return r;
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
static std::string trim_copy(const std::string& s) {
  const char* ws = " \t\r\n";
  auto b = s.find_first_not_of(ws);
  if (b == std::string::npos) return {};
  auto e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}
static int parse_video_index_from_path(const char* dev) {
  if (!dev) return -1;
  size_t len = strlen(dev);
  if (len == 0) return -1;
  int end = (int)len - 1, start = end;
  while (start >= 0 && isdigit((unsigned char)dev[start])) start--;
  start++;
  if (start <= end) return atoi(dev + start);
  return -1;
}

// ---------------- Networking ----------------
static int set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return -1;
  return 0;
}
static int set_blocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  flags &= ~O_NONBLOCK;
  if (fcntl(fd, F_SETFL, flags) < 0) return -1;
  return 0;
}
static int setup_server_socket(int port) {
  signal(SIGPIPE, SIG_IGN);
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) { perror("[net] socket"); return -1; }
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (set_nonblocking(fd) < 0) { perror("[net] set_nonblocking(listen)"); close(fd); return -1; }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)port);
  if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("[net] bind"); close(fd); return -1; }
  if (listen(fd, 16) < 0) { perror("[net] listen"); close(fd); return -1; }
  fprintf(stderr, "[net] Listening on port %d\n", port);
  return fd;
}
static bool send_all(int fd, const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
    if (n > 0) { sent += (size_t)n; continue; }
    if (n < 0 && (errno == EINTR)) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    return false;
  }
  return true;
}

// ---------------- V4L2 control helpers ----------------
#ifndef VIDIOC_LOG_STATUS
#define VIDIOC_LOG_STATUS _IO('V', 70)
#endif
static int open_video_node_rw(const char* dev, int dev_index = -1) {
  int fd = open(dev, O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) fprintf(stderr, "[v4l2%d] Cannot open '%s': %d, %s\n", dev_index, dev, errno, strerror(errno));
  return fd;
}
static std::string expand_user_path(const char* p) {
  if (!p || !*p) return std::string();
  std::string s(p);
  if (s[0] == '~') {
    const char* home = getenv("HOME");
    if (home && *home) s = std::string(home) + s.substr(1);
  }
  return s;
}
static bool read_file_binary(const std::string& path, std::vector<uint8_t>& out, int dev_index = -1) {
  out.clear();
  FILE* f = fopen(path.c_str(), "rb");
  if (!f) { fprintf(stderr, "[edid%d] Failed to open: %s (%s)\n", dev_index, path.c_str(), strerror(errno)); return false; }
  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  if (len < 0) { fclose(f); fprintf(stderr, "[edid%d] ftell failed\n", dev_index); return false; }
  fseek(f, 0, SEEK_SET);
  out.resize((size_t)len);
  size_t rd = fread(out.data(), 1, (size_t)len, f);
  fclose(f);
  if (rd != (size_t)len) { fprintf(stderr, "[edid%d] Read size mismatch for %s\n", dev_index, path.c_str()); out.clear(); return false; }
  return true;
}
static void fix_edid_checksums_inplace(std::vector<uint8_t>& edid, int dev_index = -1) {
  if (edid.size() % 128 != 0) {
    fprintf(stderr, "[edid%d] Warning: EDID size (%zu) not multiple of 128\n", dev_index, edid.size());
    return;
  }
  const size_t blocks = edid.size() / 128;
  for (size_t b = 0; b < blocks; ++b) {
    uint8_t sum = 0;
    uint8_t* blk = edid.data() + b * 128;
    for (int i = 0; i < 127; ++i) sum = (uint8_t)(sum + blk[i]);
    blk[127] = (uint8_t)(0x100 - sum);
  }
}
static bool set_device_edid_from_file(const char* dev, const char* edid_path, bool fix_checksums, int dev_index = -1) {
  std::string path = expand_user_path(edid_path);
  std::vector<uint8_t> edid;
  if (!read_file_binary(path, edid, dev_index)) return false;
  if (edid.size() < 128 || (edid.size() % 128) != 0) {
    fprintf(stderr, "[edid%d] Invalid EDID size %zu\n", dev_index, edid.size());
    return false;
  }
  if (fix_checksums) fix_edid_checksums_inplace(edid, dev_index);
  int fd = open_video_node_rw(dev, dev_index); if (fd < 0) return false;
  struct v4l2_edid vedid{};
  vedid.pad = 0; vedid.start_block = 0; vedid.blocks = (uint32_t)(edid.size() / 128); vedid.edid = edid.data();
  int rc = xioctl(fd, VIDIOC_S_EDID, &vedid);
  if (rc < 0) { fprintf(stderr, "[edid%d] VIDIOC_S_EDID failed on %s: %s\n", dev_index, dev, strerror(errno)); close(fd); return false; }
  fprintf(stderr, "[edid%d] Set %u EDID block(s) on %s\n", dev_index, vedid.blocks, dev);
  close(fd); return true;
}
static bool query_and_set_dv_timings(const char* dev, int dev_index = -1) {
  int fd = open_video_node_rw(dev, dev_index); if (fd < 0) return false;
  struct v4l2_dv_timings timings{};
  if (xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &timings) < 0) { fprintf(stderr, "[dvt%d] QUERY failed on %s: %s\n", dev_index, dev, strerror(errno)); close(fd); return false; }
  if (xioctl(fd, VIDIOC_S_DV_TIMINGS, &timings) < 0)   { fprintf(stderr, "[dvt%d] SET failed on %s: %s\n", dev_index, dev, strerror(errno)); close(fd); return false; }
  fprintf(stderr, "[dvt%d] Set DV timings to %ux%u\n", dev_index, timings.bt.width, timings.bt.height);
  close(fd); return true;
}
static void print_current_format(const char* dev, int dev_index = -1) {
  int fd = open_video_node_rw(dev, dev_index); if (fd < 0) return;
  struct v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
    uint32_t fourcc = fmt.fmt.pix.pixelformat;
    char fcc[5] = { (char)(fourcc & 0xFF), (char)((fourcc >> 8) & 0xFF), (char)((fourcc >> 16) & 0xFF), (char)((fourcc >> 24) & 0xFF), 0 };
    fprintf(stderr, "[fmt%d] %s: %ux%u, pix=%s, bytesperline=%u, sizeimage=%u\n",
      dev_index, dev, fmt.fmt.pix.width, fmt.fmt.pix.height, fcc, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  }
  close(fd);
}
static bool set_pixfmt_rgb3(const char* dev, int dev_index = -1) {
  int fd = open_video_node_rw(dev, dev_index); if (fd < 0) return false;
  struct v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) { close(fd); return false; }
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { close(fd); return false; }
  close(fd); return true;
}
static void log_device_status(const char* dev, int dev_index = -1) {
  int fd = open_video_node_rw(dev, dev_index); if (fd < 0) return;
  xioctl(fd, VIDIOC_LOG_STATUS, NULL);
  close(fd);
}
static void run_hdmi_setup_for_device(const char* dev_path, const char* edid_file_path, bool fix_checksums, bool set_rgb3_fmt = true, int dev_index = -1) {
  if (!set_device_edid_from_file(dev_path, edid_file_path, fix_checksums, dev_index)) return;
  usleep(2 * 1000 * 1000);
  query_and_set_dv_timings(dev_path, dev_index);
  usleep(1 * 1000 * 1000);
  print_current_format(dev_path, dev_index);
  usleep(1 * 1000 * 1000);
  if (set_rgb3_fmt) set_pixfmt_rgb3(dev_path, dev_index);
  usleep(1 * 1000 * 1000);
  log_device_status(dev_path, dev_index);
}

// ---------------- V4L2 init/deinit/capture -----------------
int init_dev_stage1(struct devInfo*& devInfos) {
  fprintf(stderr, "[cap%d] Opening %s\n", devInfos->index, devInfos->device);
  struct stat st{};
  if (-1 == stat(devInfos->device, &st)) { fprintf(stderr, "[cap%d] Cannot stat '%s': %s\n", devInfos->index, devInfos->device, strerror(errno)); exit(EXIT_FAILURE); }
  if (!S_ISCHR(st.st_mode)) { fprintf(stderr, "[cap%d] %s is not a device\n", devInfos->index, devInfos->device); exit(EXIT_FAILURE); }
  devInfos->fd = open(devInfos->device, O_RDWR | O_NONBLOCK, 0);
  if (-1 == devInfos->fd) { fprintf(stderr, "[cap%d] Cannot open '%s': %s\n", devInfos->index, devInfos->device, strerror(errno)); exit(EXIT_FAILURE); }
  struct v4l2_capability cap{};
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) errno_exit("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) { fprintf(stderr, "[cap%d] %s not capture device\n", devInfos->index, devInfos->device); exit(EXIT_FAILURE); }
  struct v4l2_cropcap cropcap{}; cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(devInfos->fd, VIDIOC_CROPCAP, &cropcap)) {
    struct v4l2_crop crop{}; crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; crop.c = cropcap.defrect;
    xioctl(devInfos->fd, VIDIOC_S_CROP, &crop); // ignore failure
  }
  struct v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      byteScaler = 3;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "[cap%d] Forcing RGB24\n", devInfos->index);
    } else if (devInfos->force_format == 2) {
      byteScaler = 2;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "[cap%d] Forcing UYVY\n", devInfos->index);
    } else if (devInfos->force_format == 1) {
      byteScaler = 1;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      fprintf(stderr, "[cap%d] Forcing GREY\n", devInfos->index);
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt)) errno_exit("VIDIOC_S_FMT");
  } else {
    if (-1 == xioctl(devInfos->fd, VIDIOC_G_FMT, &fmt)) errno_exit("VIDIOC_G_FMT");
  }
  unsigned int min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min) fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) fmt.fmt.pix.sizeimage = min;

  CLEAR(devInfos->req);
  devInfos->req.count = 4;
  devInfos->req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  devInfos->req.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_REQBUFS, &devInfos->req)) errno_exit("VIDIOC_REQBUFS");
  if (devInfos->req.count < 2) { fprintf(stderr, "[cap%d] Insufficient MMAP buffers\n", devInfos->index); exit(EXIT_FAILURE); }
  return 0;
}
int init_dev_stage2(struct buffer*& buffers, struct devInfo*& devInfos) {
  buffers = (buffer*)calloc(devInfos->req.count, sizeof(*buffers));
  if (!buffers) { fprintf(stderr, "[cap%d] OOM buffers\n", devInfos->index); exit(EXIT_FAILURE); }
  for (devInfos->n_buffers = 0; devInfos->n_buffers < devInfos->req.count; ++devInfos->n_buffers) {
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP; buf.index = devInfos->n_buffers;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYBUF, &buf)) errno_exit("VIDIOC_QUERYBUF");
    buffers[devInfos->n_buffers].length = buf.length;
    buffers[devInfos->n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, devInfos->fd, buf.m.offset);
    if (MAP_FAILED == buffers[devInfos->n_buffers].start) errno_exit("mmap");
  }
  if (devInfos->isTC358743) {
    struct v4l2_dv_timings timings{}; int ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
    if (ret >= 0) {
      devInfos->startingWidth = timings.bt.width;
      devInfos->startingHeight = timings.bt.height;
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;
      ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
      if (ret < 0) { fprintf(stderr, "[cap%d] Failed to set DV timings\n", devInfos->index); return 1; }
      const struct v4l2_bt_timings* bt = &timings.bt;
      double tot_h = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
      double tot_w = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
      devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_w * tot_h));
      if (devInfos->framerate < devInfos->targetFramerate) devInfos->targetFramerate = devInfos->framerate;
      devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
      devInfos->frameDelayMicros = (1000000.0 / devInfos->framerate);
      devInfos->frameDelayMillis = (1000.0 / devInfos->framerate);
      devInfos->targetFrameDelayMicros = devInfos->frameDelayMicros * devInfos->framerateDivisor;
      devInfos->targetFrameDelayMillis = devInfos->frameDelayMillis * devInfos->framerateDivisor;
      devInfos->realAndTargetRatesMatch = fabs(devInfos->frameDelayMicros - devInfos->targetFrameDelayMicros) < 0.001;
    }
  } else {
    fprintf(stderr, "[cap%d] Only TC358743 supported currently\n", devInfos->index);
    exit(1);
  }
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP; buf.index = i;
    xioctl(devInfos->fd, VIDIOC_QBUF, &buf);
  }
  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type);
  devInfos->outputFrame = (unsigned char*)calloc((size_t)devInfos->startingWidth * (size_t)devInfos->startingHeight * (size_t)byteScaler, 1);
  if (!devInfos->outputFrame) { fprintf(stderr, "[cap%d] OOM outputFrame\n", devInfos->index); exit(1); }
  return 0;
}
int get_frame(struct buffer* buffers, struct devInfo* devInfos) {
  fd_set fds;
  struct timeval tv{};
  FD_ZERO(&fds); FD_SET(devInfos->fd, &fds);
  tv.tv_sec = 0; tv.tv_usec = (suseconds_t)(devInfos->frameDelayMicros * 2.0);
  int r = select(devInfos->fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) { if (EINTR == errno) return 0; errno_exit("select"); }
  if (0 == r) {
    fprintf(stderr, "[cap%d] select timeout\n", devInfos->index);
    shouldLoop.store(false);
    return 1;
  }
  struct v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(devInfos->fd, VIDIOC_DQBUF, &buf)) {
    if (errno == EAGAIN) return 0;
    fprintf(stderr, "[cap%d] DQBUF error: %s\n", devInfos->index, strerror(errno));
    shouldLoop.store(false);
    return 1;
  }
  assert(buf.index < devInfos->n_buffers);
  size_t valid = buf.bytesused ? buf.bytesused : buffers[buf.index].length;
  size_t copy_len = std::min<size_t>(valid, (size_t)devInfos->startingSize);
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, copy_len);
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf)) errno_exit("VIDIOC_QBUF");
  return 0;
}
int deinit_bufs(struct buffer*& buffers, struct devInfo*& devInfos) {
  if (!devInfos) return 0;
  if (devInfos->fd >= 0) {
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(devInfos->fd, VIDIOC_STREAMOFF, &t);
  }
  if (buffers) {
    for (unsigned int i = 0; i < devInfos->n_buffers; ++i) {
      if (buffers[i].start && buffers[i].length) munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers); buffers = nullptr;
  }
  if (devInfos->fd >= 0) { close(devInfos->fd); devInfos->fd = -1; }
  return 0;
}
static void free_devinfo(struct devInfo*& d) {
  if (!d) return;
  if (d->outputFrame) { free(d->outputFrame); d->outputFrame = nullptr; }
  if (d->device) { free(d->device); d->device = nullptr; }
  free(d); d = nullptr;
}
int init_vars(struct devInfo*& devInfos, struct buffer*& bufs, const int force_format, const double targetFramerate, const bool isTC358743, const char* dev_name, int index) {
  devInfos = (devInfo*)calloc(1, sizeof(*devInfos));
  devInfos->device = (char*)calloc(strlen(dev_name) + 1, 1);
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
  init_dev_stage2(bufs, devInfos);
  return 0;
}
int reinit_device_inplace(struct devInfo*& devInfos, struct buffer*& bufs) {
  deinit_bufs(bufs, devInfos);
  init_dev_stage1(devInfos);
  init_dev_stage2(bufs, devInfos);
  return 0;
}
void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
  if (isDualInput) deinit_bufs(buffersAlt, devInfoAlt);
  free_devinfo(devInfoMain);
  if (isDualInput) free_devinfo(devInfoAlt);
  for (int fd : client_fds) close(fd);
  client_fds.clear();
  if (listen_fd >= 0) { close(listen_fd); listen_fd = -1; }
}
void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt) {
  fprintf(stderr, "[main] Initializing..\n");
  init_vars(deviMain, bufMain, 3, allDevicesTargetFramerate, true, devNames[0].c_str(), 0);
  if (isDualInput) init_vars(deviAlt, bufAlt, 3, allDevicesTargetFramerate, true, devNames[1].c_str(), 1);
  shouldLoop.store(true);
  numPixels = devInfoMain->startingWidth * devInfoMain->startingHeight;
  usleep(1000);
}

// ---------------- Packet send helpers ----------------
static std::vector<uint8_t> build_devinfo_packet(const devInfo* d) {
  using namespace netpkt;
  const std::string devname = d->device ? std::string(d->device) : std::string();

  DevInfoMeta meta{};
  meta.dev_index = htons((uint16_t)d->index);
  meta.byteScaler = htons((uint16_t)byteScaler);
  meta.width = htonl((uint32_t)d->startingWidth);
  meta.height = htonl((uint32_t)d->startingHeight);
  meta.framerate = htonl((uint32_t)d->framerate);
  meta.target_fps_x1000 = htonl((uint32_t)llround(d->targetFramerate * 1000.0));
  meta.inputIsBGR = g_inputIsBGR ? 1 : 0;
  meta.isTC358743 = d->isTC358743 ? 1 : 0;
  meta.streamCodec = (uint8_t)g_streamCodec;
  meta.reserved = 0;
  meta.frameDelayMicros_x1000 = htonll((uint64_t)llround(d->frameDelayMicros * 1000.0));
  meta.deviceNameLen = htonl((uint32_t)devname.size());

  Header hdr{};
  hdr.magic = htonl(MAGIC);
  hdr.version = htons(PROTO_VER);
  hdr.type = htons(PT_DEVINFO);
  hdr.headerSize = htonl((uint32_t)(sizeof(hdr) + sizeof(meta) + devname.size()));
  hdr.payloadSize = htonll(0);

  std::vector<uint8_t> buf;
  buf.resize(sizeof(hdr) + sizeof(meta) + devname.size());
  memcpy(buf.data(), &hdr, sizeof(hdr));
  memcpy(buf.data() + sizeof(hdr), &meta, sizeof(meta));
  if (!devname.empty()) memcpy(buf.data() + sizeof(hdr) + sizeof(meta), devname.data(), devname.size());
  return buf;
}
static bool send_devinfo_to_fd(int fd, const devInfo* d) {
  auto pkt = build_devinfo_packet(d);
  return send_all(fd, pkt.data(), pkt.size());
}
static bool send_signal_to_fd(int fd, uint8_t state) {
  using namespace netpkt;
  SignalMeta sm{}; sm.state = state;

  Header hdr{};
  hdr.magic = htonl(MAGIC);
  hdr.version = htons(PROTO_VER);
  hdr.type = htons(PT_SIGNAL);
  hdr.headerSize = htonl((uint32_t)(sizeof(hdr) + sizeof(sm)));
  hdr.payloadSize = htonll(0);

  if (!send_all(fd, &hdr, sizeof(hdr))) return false;
  if (!send_all(fd, &sm, sizeof(sm))) return false;
  return true;
}
static void broadcast_signal(uint8_t state) {
  for (size_t i = 0; i < client_fds.size();) {
    int fd = client_fds[i];
    if (!send_signal_to_fd(fd, state)) {
      fprintf(stderr, "[net] Dropping client fd=%d (signal send)\n", fd);
      close(fd);
      client_fds.erase(client_fds.begin() + i);
      continue;
    }
    ++i;
  }
}
static bool send_frame_to_fd(int fd, const devInfo* d, uint64_t frame_id, const uint8_t* payload, size_t payload_len) {
  using namespace netpkt;
  FrameMeta fm{};
  fm.frame_id = htonll(frame_id);
  auto now = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  fm.timestamp_us = htonll((uint64_t)now);
  fm.width = htonl((uint32_t)d->startingWidth);
  fm.height = htonl((uint32_t)d->startingHeight);
  fm.stride = htonl((uint32_t)(d->startingWidth * byteScaler));
  fm.codec = (uint8_t)g_streamCodec; // RAW currently
  fm.is_bgr = g_inputIsBGR ? 1 : 0;
  fm.reserved = 0;

  Header hdr{};
  hdr.magic = htonl(MAGIC);
  hdr.version = htons(PROTO_VER);
  hdr.type = htons(PT_FRAME);
  hdr.headerSize = htonl((uint32_t)(sizeof(hdr) + sizeof(fm)));
  hdr.payloadSize = htonll((uint64_t)payload_len);

  if (!send_all(fd, &hdr, sizeof(hdr))) return false;
  if (!send_all(fd, &fm, sizeof(fm))) return false;
  if (payload_len && !send_all(fd, payload, payload_len)) return false;
  return true;
}
static void accept_new_clients_and_send_devinfo(bool signal_up) {
  using namespace netpkt;
  while (true) {
    sockaddr_in cliaddr{};
    socklen_t len = sizeof(cliaddr);
    int cfd = accept(listen_fd, (sockaddr*)&cliaddr, &len);
    if (cfd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      perror("[net] accept");
      break;
    }
    set_blocking(cfd);
    int one = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    client_fds.push_back(cfd);
    char ip[64]; inet_ntop(AF_INET, &cliaddr.sin_addr, ip, sizeof(ip));
    fprintf(stderr, "[net] Client connected: %s:%d (fd=%d). Total clients: %zu\n", ip, ntohs(cliaddr.sin_port), cfd, client_fds.size());
    if (devInfoMain) {
      if (!send_devinfo_to_fd(cfd, devInfoMain)) {
        fprintf(stderr, "[net] Failed to send DEVINFO to fd=%d; closing\n", cfd);
        close(cfd); client_fds.pop_back(); continue;
      }
      if (!send_signal_to_fd(cfd, signal_up ? SIGNAL_UP : SIGNAL_DOWN)) {
        fprintf(stderr, "[net] Failed to send SIGNAL to fd=%d; closing\n", cfd);
        close(cfd); client_fds.pop_back(); continue;
      }
    }
  }
}
static void broadcast_devinfo_if_any_change(const devInfo* d, int& last_w, int& last_h, int& last_bs) {
  if (!d) return;
  if (d->startingWidth == last_w && d->startingHeight == last_h && byteScaler == last_bs) return;
  auto pkt = build_devinfo_packet(d);
  for (size_t i = 0; i < client_fds.size();) {
    int fd = client_fds[i];
    if (!send_all(fd, pkt.data(), pkt.size())) {
      fprintf(stderr, "[net] Dropping client fd=%d (devinfo send)\n", fd);
      close(fd);
      client_fds.erase(client_fds.begin() + i);
      continue;
    }
    ++i;
  }
  last_w = d->startingWidth; last_h = d->startingHeight; last_bs = byteScaler;
}
static void broadcast_frame_packet(const devInfo* d, uint64_t frame_id, const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < client_fds.size();) {
    int fd = client_fds[i];
    if (!send_frame_to_fd(fd, d, frame_id, buf, len)) {
      fprintf(stderr, "[net] Dropping client fd=%d (frame send)\n", fd);
      close(fd);
      client_fds.erase(client_fds.begin() + i);
      continue;
    }
    ++i;
  }
}

// ---------------- CLI ----------------
void parse_cli_or_die(int argc, const char** argv) {
  double opt_fps = allDevicesTargetFramerate;
  const char* opt_devices = nullptr;
  int opt_port = 0;
  int opt_bgr = 0;

  struct poptOption optionsTable[] = {
    { "fps",     'f', POPT_ARG_DOUBLE, &opt_fps, 0, "Target framerate", "FPS" },
    { "devices", 'd', POPT_ARG_STRING, &opt_devices, 0, "V4L2 devices: /dev/video0[,/dev/video1]", "DEV[,DEV]" },
    { "port",    'p', POPT_ARG_INT, &opt_port, 0, "TCP listen port", "PORT" },
    { "bgr",     0,   POPT_ARG_NONE, &opt_bgr, 0, "Capture ordering is BGR24", nullptr },
    { "help",    'h', POPT_ARG_NONE, nullptr, 'h', "Help", nullptr },
    { nullptr, 0, 0, nullptr, 0, nullptr, nullptr }
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
      std::string token = (pos == std::string::npos) ? list.substr(start) : list.substr(start, pos - start);
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
    while ((arg = poptGetArg(pc)) != nullptr) devNames.emplace_back(arg);
  }
  poptFreeContext(pc);
  if (devNames.size() == 1) isDualInput = false;
  else if (devNames.size() == 2) isDualInput = true;
  else { fprintf(stderr, "[main] Usage: %s --devices=/dev/video0 --port=9001 [--bgr]\n", argv[0]); exit(1); }

  if (opt_port <= 0 || opt_port > 65535) { fprintf(stderr, "[main] Need --port 1..65535\n"); exit(1); }
  allDevicesTargetFramerate = opt_fps;
  listenPort = opt_port;
  g_inputIsBGR = (opt_bgr != 0);
  g_streamCodec = CODEC_RAW; // network payload RAW for now
  fprintf(stderr, "[main] Options: --fps=%.3f --devices=%s%s%s --port=%d --inputIsBGR=%d\n",
    allDevicesTargetFramerate, devNames[0].c_str(), isDualInput ? "," : "", isDualInput ? devNames[1].c_str() : "",
    listenPort, g_inputIsBGR ? 1 : 0);
}

// --------------- Recovery thread ---------------
static std::thread g_recover_thread;
static void recovery_worker() {
  while (programRunning.load()) {
    if (shouldLoop.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    // No-signal: deinit -> HDMI setup -> reinit -> probe
    usleep(500000);
    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      if (devInfoMain) deinit_bufs(buffersMain, devInfoMain);
    }
    const char* edid_path = "/root/v4l2-video-capture-testing-program/customgoodhdmiedid_binary.txt";
    run_hdmi_setup_for_device(devInfoMain ? devInfoMain->device : "/dev/video0", edid_path, true, true, devInfoMain ? devInfoMain->index : 0);
    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      reinit_device_inplace(devInfoMain, buffersMain);
    }
    usleep(500000);
    int probe_rc = 1;
    {
      std::lock_guard<std::mutex> lk(g_v4l2_mtx);
      if (devInfoMain && buffersMain) probe_rc = get_frame(buffersMain, devInfoMain);
    }
    if (probe_rc == 0) {
      fprintf(stderr, "[recovery] Signal restored.\n");
      shouldLoop.store(true);
    } else {
      fprintf(stderr, "[recovery] Still no signal; retrying.\n");
    }
  }
}

// ----------------- main -----------------
int main(int argc, char** argv) {
  parse_cli_or_die(argc, (const char**)argv);
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt);

  listen_fd = setup_server_socket(listenPort);
  if (listen_fd < 0) { cleanup_vars(); return 1; }

  g_recover_thread = std::thread(recovery_worker);

  // Warm-up: grab a few frames
  for (int i = 0; i < 30; i++) {
    accept_new_clients_and_send_devinfo(true);
    std::lock_guard<std::mutex> lk(g_v4l2_mtx);
    if (get_frame(buffersMain, devInfoMain) != 0) break;
  }

  fprintf(stderr, "[main] Starting capture-to-network loop. Network codec=RAW RGB24\n");

  uint64_t frame_id = 0;
  int last_w = devInfoMain->startingWidth, last_h = devInfoMain->startingHeight, last_bs = byteScaler;
  bool last_signal_up = true; // shouldLoop initially true

  while (programRunning.load()) {
    // Primary capture loop
    while (programRunning.load() && shouldLoop.load()) {
      accept_new_clients_and_send_devinfo(true);

      // If signal transitioned up, broadcast it
      if (!last_signal_up) {
        broadcast_signal(netpkt::SIGNAL_UP);
        last_signal_up = true;
      }

      // If resolution/bytescale changed (recovery), push new DEVINFO to clients
      broadcast_devinfo_if_any_change(devInfoMain, last_w, last_h, last_bs);

      int rc = 0;
      {
        std::lock_guard<std::mutex> lk(g_v4l2_mtx);
        rc = get_frame(buffersMain, devInfoMain);
      }
      if (rc != 0) {
        // signal down
        if (last_signal_up) {
          broadcast_signal(netpkt::SIGNAL_DOWN);
          last_signal_up = false;
        }
        break;
      }

      const size_t bytes = frame_bytes(devInfoMain);
      broadcast_frame_packet(devInfoMain, ++frame_id, devInfoMain->outputFrame, bytes);

      if (!devInfoMain->realAndTargetRatesMatch) {
        double us = devInfoMain->targetFrameDelayMicros;
        if (us > 0) usleep((useconds_t)us);
      }
    }

    // No-signal state: throttle and advertise current state to new clients
    while (programRunning.load() && !shouldLoop.load()) {
      accept_new_clients_and_send_devinfo(false);
      double us = frame_delay_us(devInfoMain);
      if (us < 5000.0) us = 5000.0;
      usleep((useconds_t)us);
    }
  }

  programRunning.store(false);
  if (g_recover_thread.joinable()) g_recover_thread.join();
  cleanup_vars();
  return 0;
}
