// v4l2_streamer.cpp
// Build: see command at bottom.
// Notes:
// - Default stream is raw RGB24 frames over TCP.
// - Enable MJPEG software-encoding with --mjpeg (uses libjpeg).
// - If your capture device produces BGR24 ordering, use --bgr to have the server convert to RGB24
//   (so clients can use -pixel_format rgb24). Alternatively, clients can specify -pixel_format bgr24
//   and you can omit --bgr to avoid the extra memcpy.
//
// Quick: server will swap R<->B when --bgr is given. No other changes to capture flow.
//
// Multithread changes:
// - Added an MJPEG encoding thread pool and a bounded job queue to avoid blocking capture/networking.
// - Main thread captures frames and enqueues jobs; encoder threads compress and publish the latest JPEG.
// - Main thread broadcasts the most recently encoded JPEG without blocking encoders.
// - Optional: multi-threaded channel swap for RAW path (useful when --bgr on large frames).
//
// Lazy-send feature:
// - Added --lazy/--no-lazy to skip sending identical frames.
// - We compare the current raw captured frame to the previous raw captured frame.
// - If the frame is unchanged: we skip MJPEG compression and skip network send (saves CPU/bandwidth).
// - Comparison is done against the captured bytes (pre-conversion), so it works for both RAW and MJPEG paths.

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

// Networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <signal.h>

// JPEG (for --mjpeg)
#include <jpeglib.h>

// misc
#include <algorithm>

// threading additions
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>

using namespace std;

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

int byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, numPixels = defaultWidth * defaultHeight;
double allDevicesTargetFramerate = 240;
bool isDualInput = false, done = false;
std::atomic<bool> shouldLoop;
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
int g_jpeg_quality = 80; // 1-100
bool g_inputIsBGR = false; // set by --bgr: capture produces BGR byte order; server will convert to RGB before sending

// Lazy-send toggle (default enabled)
bool g_lazy_send = true;

// New: encoder threading controls
int g_encode_threads = 0;            // 0 => auto
size_t g_encode_queue_max = 0;       // set later relative to threads

// Latest JPEG produced by encoder workers (atomic shared_ptr for lock-free reads)
std::shared_ptr<std::vector<unsigned char>> g_latest_jpeg;
std::atomic<uint64_t> g_latest_jpeg_id{0}; // monotonically increasing id for freshness gating

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
  unsigned char *outputFrame;
  char* device;
};
struct buffer* buffersMain;
struct buffer* buffersAlt;
struct devInfo* devInfoMain;
struct devInfo* devInfoAlt;

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

// helper to trim whitespace from tokens (local to this file)
static inline std::string trim_copy(const std::string& s) {
    const char* ws = " \t\r\n";
    const auto b = s.find_first_not_of(ws);
    if (b == std::string::npos) return std::string();
    const auto e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

// Networking helpers
static int set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return -1;
  return 0;
}
static int setup_server_socket(int port) {
  signal(SIGPIPE, SIG_IGN); // Avoid SIGPIPE on send() to disconnected clients
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

// Simple RGB24->JPEG encoder using libjpeg, output to memory.
// This keeps it straightforward for streaming concatenated JPEGs over TCP.
static bool encode_rgb24_to_jpeg_mem(const unsigned char* rgb,
                                     int width,
                                     int height,
                                     int quality,
                                     std::vector<unsigned char>& out)
{
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
  cinfo.in_color_space = JCS_RGB; // RGB24

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

    // On blocking sockets, EAGAIN/EWOULDBLOCK should not happen (unless timeouts are set).
    // Treat any other error as a disconnect.
    return false; // drop client
  }
  return true;
}

// Add this helper somewhere near other helpers (before accept_new_clients)
static void seed_initial_frame_for_fd(int fd, int width, int height) {
  if (g_streamCodec == CODEC_MJPEG) {
    // Seed with a JPEG at the actual resolution
    std::vector<unsigned char> rgb(width * height * 3, 0); // black RGB24 frame
    std::vector<unsigned char> jpeg;
    if (encode_rgb24_to_jpeg_mem(rgb.data(), width, height, g_jpeg_quality, jpeg) && !jpeg.empty()) {
      send_all_or_drop(fd, jpeg.data(), jpeg.size());
    }
  } else {
    // Seed with a RAW frame at the actual resolution
    size_t frame_len = (size_t)width * (size_t)height * (size_t)byteScaler;
    std::vector<unsigned char> seed(frame_len, 0);
    send_all_or_drop(fd, seed.data(), seed.size());
  }
}

// Keep listening socket non-blocking if you like, but leave clients blocking.
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
    // Note: do NOT set_nonblocking(cfd);
    int one = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    // Optional: keepalive
    setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    client_fds.push_back(cfd);

    // Seed this new client with an initial frame at the actual resolution
    int w = (devInfoMain ? devInfoMain->startingWidth : defaultWidth);
    int h = (devInfoMain ? devInfoMain->startingHeight : defaultHeight);
    for (int i = 0; i < 120; i++) seed_initial_frame_for_fd(cfd, w, h);

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

// parse optional args and collect device names
void parse_cli_or_die(int argc, const char** argv) {
  double opt_fps = allDevicesTargetFramerate; // default remains the global default
  const char* opt_devices = nullptr;          // optional comma-separated list
  int opt_port = 0;
  int opt_mjpeg = 0;
  int opt_raw = 0;
  int opt_q = g_jpeg_quality;
  int opt_bgr = 0;
  int opt_enc_threads = 0; // new
  int opt_lazy = 0;
  int opt_no_lazy = 0;

  struct poptOption optionsTable[] = {
    { "fps",     'f',    POPT_ARG_DOUBLE,   &opt_fps,      0,    "Target framerate for all devices",                        "FPS" },
    { "devices", 'd',    POPT_ARG_STRING,   &opt_devices,  0,    "V4L2 device(s): /dev/video0 or /dev/video0,/dev/video1",  "DEV[,DEV]" },
    { "port",    'p',    POPT_ARG_INT,      &opt_port,     0,    "TCP listen port for streaming frames",                    "PORT" },
    { "mjpeg",   0,      POPT_ARG_NONE,     &opt_mjpeg,    0,    "Encode and stream Motion-JPEG instead of raw RGB24",      nullptr },
    { "raw",     0,      POPT_ARG_NONE,     &opt_raw,      0,    "Force raw-RGB24 stream (default)",                         nullptr },
    { "jpeg-quality", 'q', POPT_ARG_INT,    &opt_q,        0,    "JPEG quality when --mjpeg (1-100, default 80)",           "Q" },
    { "bgr",     0,      POPT_ARG_NONE,     &opt_bgr,      0,    "Capture is BGR24; server will swap to RGB24 before sending", nullptr },
    { "encode-threads",  0, POPT_ARG_INT,   &opt_enc_threads, 0, "MJPEG encoder threads (0=auto)",                          "N" },
    { "lazy",    0,      POPT_ARG_NONE,     &opt_lazy,     0,    "Enable lazy-send (skip identical frames)",                nullptr },
    { "no-lazy", 0,      POPT_ARG_NONE,     &opt_no_lazy,  0,    "Disable lazy-send",                                       nullptr },
    { "help",    'h',    POPT_ARG_NONE,     nullptr,       'h',  "Show help and exit",                                      nullptr },
    { nullptr,   0,      0,                 nullptr,       0,    nullptr,                                                   nullptr }
  };
  poptContext pc = poptGetContext(argv[0], argc, argv, optionsTable, 0);
  int rc;
  while ((rc = poptGetNextOpt(pc)) >= 0) {
    // values are written directly to opt variables
  }
  if (rc < -1) {
    fprintf(stderr, "[main] Error parsing options: %s: %s\n", poptBadOption(pc, POPT_BADOPTION_NOALIAS), poptStrerror(rc));
    poptPrintUsage(pc, stderr, 0);
    exit(1);
  }
  // Collect remaining non-option arguments as device names
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
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -f, --fps=<FPS>            Target framerate for all devices (default: %.3f)\n", allDevicesTargetFramerate);
    fprintf(stderr, "  -d, --devices=<DEV[,DEV]>  Comma-separated devices, e.g. /dev/video0 or /dev/video0,/dev/video1\n");
    fprintf(stderr, "  -p, --port=<PORT>          TCP listen port for streaming frames (required)\n");
    fprintf(stderr, "      --mjpeg                Stream Motion-JPEG (software-encoded)\n");
    fprintf(stderr, "      --raw                  Force raw RGB24 stream (default)\n");
    fprintf(stderr, "  -q, --jpeg-quality=<Q>     JPEG quality when --mjpeg (1-100, default 80)\n");
    fprintf(stderr, "      --bgr                  Capture appears to be BGR24; server will convert to RGB24 before sending\n");
    fprintf(stderr, "      --encode-threads=<N>   MJPEG encoder threads (0=auto, default auto)\n");
    fprintf(stderr, "      --lazy                 Enable lazy-send (skip identical frames, default ON)\n");
    fprintf(stderr, "      --no-lazy              Disable lazy-send\n");
    exit(1);
  }
  if (opt_port <= 0 || opt_port > 65535) {
    fprintf(stderr, "[main] Error: valid --port=<1-65535> is required\n");
    exit(1);
  }

  // Apply parsed options to globals
  allDevicesTargetFramerate = opt_fps;
  listenPort = opt_port;

  if (opt_q < 1) opt_q = 1;
  if (opt_q > 100) opt_q = 100;
  g_jpeg_quality = opt_q;

  g_inputIsBGR = (opt_bgr != 0);

  // Resolve stream mode (explicit --raw wins if both specified)
  if (opt_mjpeg && !opt_raw) g_streamCodec = CODEC_MJPEG;
  else g_streamCodec = CODEC_RAW;

  g_encode_threads = opt_enc_threads;

  // Lazy default is ON; explicit flags override
  if (opt_no_lazy) g_lazy_send = false;
  else if (opt_lazy) g_lazy_send = true;

  fprintf(stderr, "[main] Parsed options: --fps=%.3f, --devices=%s%s%s, --port=%d, --stream=%s%s, --bgr=%d, --encode-threads=%d, --lazy=%d\n",
          allDevicesTargetFramerate,
          devNames[0].c_str(),
          isDualInput ? "," : "",
          isDualInput ? devNames[1].c_str() : "",
          listenPort,
          (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG"),
          (g_streamCodec == CODEC_MJPEG ? (std::string(" (q=") + std::to_string(g_jpeg_quality) + ")").c_str() : ""),
          g_inputIsBGR ? 1 : 0,
          g_encode_threads,
          g_lazy_send ? 1 : 0);
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

// Swap R/B in-place for RGB24/BGR24 conversion (single-thread)
static inline void swap_rb_inplace(unsigned char* buf, size_t pixels) {
  // Each pixel is 3 bytes: [R,G,B] or [B,G,R]
  for (size_t i = 0; i < pixels; ++i) {
    unsigned char* p = buf + i * 3;
    unsigned char t = p[0];
    p[0] = p[2];
    p[2] = t;
  }
}

// Multi-threaded in-place R<->B swap for large frames (keeps things simple)
static inline void swap_rb_inplace_mt(unsigned char* buf, size_t pixels, int threads) {
  if (threads <= 1 || pixels < 512 * 512) { // tiny frames don't benefit; just a quick heuristic
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

int init_dev_stage1(struct buffer*& buffers, struct devInfo*& devInfos) {
  fprintf(stderr, "\n[cap%d] Starting V4L2 capture testing program with the following V4L2 device: %s\n", devInfos->index, devInfos->device);
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
  fprintf(stderr, "[cap%d] Opened V4L2 device: %s\n", devInfos->index, devInfos->device);
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;
  if (-1 == xioctl(devInfos->fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "[cap%d] %s is no V4L2 device\n", devInfos->index, devInfos->device);
      exit(EXIT_FAILURE);
    }
    else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "[cap%d] %s is no video capture device\n", devInfos->index, devInfos->device);
    exit(EXIT_FAILURE);
  }
  // Select video input, video standard and tune here.
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (0 == xioctl(devInfos->fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        // Cropping not supported.
        break;
      default:
        // Errors ignored.
        break;
      }
    }
  } else {
    // Errors ignored.
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "[cap%d] Forcing format for %s to: %d\n", devInfos->index, devInfos->device, devInfos->force_format);
  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      // We request RGB24 from device. If device actually provides BGR ordering, use --bgr to swap.
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 2) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    } else if (devInfos->force_format == 1) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE; // V4L2_FIELD_INTERLACED;
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
    // Note VIDIOC_S_FMT may change width and height.
  } else {
    // Preserve original settings as set by v4l2-ctl for example
    if (-1 == xioctl(devInfos->fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }
  // Buggy driver paranoia.
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
  // TODO: Implement a more proper way to handle settings in this area regarding if we really need DV timings to be set
  if (devInfos->isTC358743) {
    struct v4l2_dv_timings timings;
    v4l2_std_id std;
    int ret;
    memset(&timings, 0, sizeof timings);
    ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
    if (ret >= 0) {
      fprintf(stderr, "[cap%d] QUERY_DV_TIMINGS for %s: %ux%ux%d pixclk %llu\n", devInfos->index, devInfos->device, timings.bt.width, timings.bt.height, byteScaler, timings.bt.pixelclock);
      devInfos->startingWidth = timings.bt.width;
      devInfos->startingHeight = timings.bt.height;
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;
      // Can read DV timings, so set them.
      ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
      if (ret < 0) {
        fprintf(stderr, "[cap%d] Failed to set DV timings\n", devInfos->index);
        return 1;
      } else {
        double tot_height, tot_width;
        const struct v4l2_bt_timings* bt = &timings.bt;
        tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
        tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
        devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));
        if (devInfos->framerate < devInfos->targetFramerate) devInfos->targetFramerate = devInfos->framerate;
        devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
        devInfos->frameDelayMicros = (1000000.0 / devInfos->framerate);
        devInfos->frameDelayMillis = (1000.0 / devInfos->framerate);
        devInfos->targetFrameDelayMicros = (1000000.0 / devInfos->framerate) * devInfos->framerateDivisor;
        devInfos->targetFrameDelayMillis = (1000.0 / devInfos->framerate) * devInfos->framerateDivisor;
        int rawInputThroughput = (float)((float)(devInfos->framerate * devInfos->startingSize) / 125000.0F); // Mb/s based on input framerate
        int rawOutputThroughput = ((float)((float)(devInfos->framerate * devInfos->startingSize) / 125000.0F)) / devInfos->framerateDivisor; // Mb/s based on output framerate
        devInfos->realAndTargetRatesMatch = doubles_equal(devInfos->frameDelayMicros, devInfos->targetFrameDelayMicros);
        fprintf(stderr, "[cap%d] device_name: %s, startingWidth: %d, startingHeight: %d, byteScaler: %d, startingSize: %d, framerate(actual): %u, framerateDivisor: %f, targetFramerate: %f, frameDelayMicros: %f, frameDelayMillis: %f, targetFrameDelayMicros: %f, targetFrameDelayMillis: %f, realAndTargetRatesMatch: %d\n",
          devInfos->index, devInfos->device, devInfos->startingWidth, devInfos->startingHeight, byteScaler, devInfos->startingSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, devInfos->frameDelayMicros, devInfos->frameDelayMillis, devInfos->targetFrameDelayMicros, devInfos->targetFrameDelayMillis, devInfos->realAndTargetRatesMatch);
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, rawInputThroughput: ~%dMb/~%dMiB/~%dMB/sec, rawOutputThroughput: ~%dMb/~%dMiB/~%dMB/sec\n", devInfos->index, devInfos->device, devInfos->isTC358743, rawInputThroughput, (int)((double)rawInputThroughput / 8.389), rawInputThroughput / 8, rawOutputThroughput, (int)((double)rawOutputThroughput / 8.389), rawOutputThroughput / 8);
      }
    } else {
      memset(&std, 0, sizeof std);
      ret = ioctl(devInfos->fd, VIDIOC_QUERYSTD, &std);
      if (ret >= 0) {
        // Can read standard, so set it.
        ret = xioctl(devInfos->fd, VIDIOC_S_STD, &std);
        if (ret < 0) {
          fprintf(stderr, "[cap%d] Failed to set standard\n", devInfos->index);
          return 1;
        } else {
          // SD video - assume 50Hz / 25fps
          devInfos->framerate = 25;
        }
      }
    }
  } else {
    fprintf(stderr, "[cap%d] Fatal: Only the TC358743 is supported for now. Support for general camera inputs (such as: %s) will need to be added in the future..\nExiting now.\n", devInfos->index, devInfos->device);
    exit(1);
  }
  unsigned int i;
  for (i = 0; i < devInfos->n_buffers; ++i) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf)) {
      //fprintf(stderr, "[cap%d] Fatal: V4L2 device (%s) -1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf) in: init_dev_stage2()\n", devInfos->index, devInfos->device);
      //errno_exit("VIDIOC_QBUF");
    }
  }
  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type)) {
    //fprintf(stderr, "[cap%d] Fatal: V4L2 device (%s) -1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type) in: init_dev_stage2()\n", devInfos->index, devInfos->device);
    //errno_exit("VIDIOC_STREAMON");
  }
  fprintf(stderr, "[cap%d] Initialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  return 0;
}

int get_frame(struct buffer* buffers, struct devInfo* devInfos) {
  fd_set fds;
  struct timeval tv;
  int r;
  FD_ZERO(&fds);
  FD_SET(devInfos->fd, &fds);
  // Timeout period to wait for device to respond
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
    fprintf(stderr, "[cap%d] select timeout\n", devInfos->index);
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
      // fall through
    default:
      fprintf(stderr, "%s error %d, %s\n", "VIDIOC_DQBUF", errno, strerror(errno));
      shouldLoop.store(false);
      return 1;
    }
  }
  assert(buf.index < devInfos->n_buffers);
  // Use bytesused if provided, fallback to mapped buffer length.
  size_t valid = buf.bytesused ? buf.bytesused : buffers[buf.index].length;
  // Clamp to allocated outputFrame size to be safe.
  size_t copy_len = std::min<size_t>(valid, (size_t)devInfos->startingSize);
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, copy_len); // copy frame data to frame buffer
  if (-1 == xioctl(devInfos->fd, VIDIOC_QBUF, &buf))
    errno_exit("VIDIOC_QBUF");
  return 0;
}

int deinit_bufs(struct buffer*& buffers, struct devInfo*& devInfos) {
  for (unsigned int i = 0; i < devInfos->n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
      errno_exit("munmap");
  free(buffers);
  fprintf(stderr, "[cap%d] Uninitialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  if (-1 == close(devInfos->fd))
    errno_exit("close");
  devInfos->fd = -1;
  fprintf(stderr, "[cap%d] Closed V4L2 device: %s\n", devInfos->index, devInfos->device);
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
  devInfos->device = (char*)calloc(strlen(dev_name)+1, sizeof(char));
  strcpy(devInfos->device, dev_name);
  devInfos->frame_number = 0;
  devInfos->framerate = 30;
  devInfos->framerateDivisor = 1;
  devInfos->startingWidth = defaultWidth;
  devInfos->startingHeight = defaultHeight;
  devInfos->startingSize = (devInfos->startingWidth * devInfos->startingHeight * byteScaler);
  devInfos->force_format = force_format;
  devInfos->targetFramerate = targetFramerate;
  devInfos->fd = -1;
  devInfos->isTC358743 = isTC358743;
  devInfos->index = index;

  // Stage1 uses the passed 'bufs' reference name; call correctly for each device
  init_dev_stage1(bufs, devInfos);
  bufs = (buffer*)calloc(devInfos->req.count, sizeof(*bufs));
  init_dev_stage2(bufs, devInfos);
  devInfos->outputFrame = (unsigned char*)calloc((devInfos->startingWidth * devInfos->startingHeight * byteScaler), sizeof(unsigned char)); // allocate memory for frame buffer
  did_memory_allocate_correctly(devInfos);
  return 0;
}

void cleanup_vars() {
  deinit_bufs(buffersMain, devInfoMain);
  if (isDualInput) deinit_bufs(buffersAlt, devInfoAlt);

  // Close network sockets
  for (int fd : client_fds) {
    close(fd);
  }
  client_fds.clear();
  if (listen_fd >= 0) {
    close(listen_fd);
    listen_fd = -1;
  }
}

void configure_main(struct devInfo*& deviMain, struct buffer*& bufMain, struct devInfo*& deviAlt, struct buffer*& bufAlt) {
  fprintf(stderr, "[main] Initializing..\n");
  // allocate memory for structs
  init_vars(deviMain, bufMain, 3, allDevicesTargetFramerate, true, true, devNames[0].c_str(), 0);
  if (isDualInput) init_vars(deviAlt, bufAlt, 3, allDevicesTargetFramerate, true, true, devNames[1].c_str(), 1);
  shouldLoop.store(true);
  numPixels = devInfoMain->startingWidth * devInfoMain->startingHeight;
  usleep(1000);
  fprintf(stderr, "\n");
}

// ----------------- MJPEG Encoder Thread Pool -----------------

struct EncodeJob {
  std::vector<unsigned char> frame; // captured bytes (RGB24 or BGR24 depending on --bgr)
  int width = 0;
  int height = 0;
  uint64_t id = 0;
};

// simple bounded MPMCish queue guarded by mutex/condvar
static std::mutex s_enc_mtx;
static std::condition_variable s_enc_cv;
static std::deque<EncodeJob> s_enc_queue;
static std::vector<std::thread> s_enc_workers;
static std::atomic<bool> s_enc_stop{false};
static std::atomic<uint64_t> s_frame_id_counter{0};

static void encoder_worker_loop() {
  // thread-local temp RGB buffer to avoid reallocs on every job
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

    // If capture is BGR, convert to RGB into tls_rgb before encoding
    const unsigned char* src_ptr = job.frame.data();
    const size_t pixels = (size_t)job.width * (size_t)job.height;
    if (g_inputIsBGR) {
      tls_rgb.resize(job.frame.size());
      // Fast swap while copying (single-thread in worker; avoids oversubscription)
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
      continue; // skip publishing if encode failed
    }

    // Publish only if this job is newer than last published (keeps stream "fresh")
    uint64_t prev = g_latest_jpeg_id.load(std::memory_order_acquire);
    while (job.id > prev) {
      if (g_latest_jpeg_id.compare_exchange_weak(prev, job.id, std::memory_order_acq_rel)) {
        // Update the latest JPEG pointer atomically (readers use atomic_load)
        std::atomic_store_explicit(&g_latest_jpeg, std::make_shared<std::vector<unsigned char>>(std::move(out)), std::memory_order_release);
        break;
      }
      // if CAS failed, 'prev' is reloaded; loop will retry if still older
    }
    // If job.id <= prev, a newer frame is already published; discard this one
  }
}

static void start_encoder_pool_if_needed(int width, int height) {
  if (g_streamCodec != CODEC_MJPEG) return;
  if (!g_encode_threads || g_encode_threads < 0) {
    // keep one core free for capture/network; at least 1
    unsigned hc = std::thread::hardware_concurrency();
    g_encode_threads = (hc > 1) ? std::max(1u, hc - 1) : 1u;
  }
  if (g_encode_threads > 32) g_encode_threads = 32; // sanity cap
  g_encode_queue_max = std::max<size_t>(g_encode_threads * 2, 2); // small bounded queue to cap latency/memory
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
  // clear queue
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    s_enc_queue.clear();
  }
  // reset latest (leave it available until shutdown complete)
}

// Enqueue a frame for encoding (drops oldest if queue is full to keep latency down)
static void enqueue_encode_job(const unsigned char* frameData, size_t bytes, int width, int height) {
  EncodeJob job;
  job.frame.assign(frameData, frameData + bytes);
  job.width = width;
  job.height = height;
  job.id = s_frame_id_counter.fetch_add(1, std::memory_order_relaxed) + 1;

  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    if (s_enc_queue.size() >= g_encode_queue_max) {
      // Drop oldest to maintain bounded latency
      s_enc_queue.pop_front();
    }
    s_enc_queue.emplace_back(std::move(job));
  }
  s_enc_cv.notify_one();
}

// ----------------- main -----------------

int main(const int argc, char** argv) {
  // Parse options and devices first
  parse_cli_or_die(argc, (const char**)argv);

  // Create a window to display the results
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt);

  // Setup TCP server
  listen_fd = setup_server_socket(listenPort);
  if (listen_fd < 0) {
    fprintf(stderr, "[main] Failed to create TCP listening socket on port %d\n", listenPort);
    cleanup_vars();
    return 1;
  }

  // Start encoder pool if MJPEG mode (after device init so we know dimensions)
  if (g_streamCodec == CODEC_MJPEG) {
    start_encoder_pool_if_needed(devInfoMain->startingWidth, devInfoMain->startingHeight);
  }

  MicroStopwatch sw;
  usleep(1000);

  // Capture and ignore some frames for about 1/2 seconds due to strange issue with first frame(s)
  for (int i = 0; i < 60; i++) {
    accept_new_clients(); // allow clients to connect early
    if (isDualInput) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_alt = std::async(std::launch::async, get_frame, buffersAlt, devInfoAlt);
      background_task_cap_main.wait();
      background_task_cap_alt.wait();
    } else {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
    }
  }

  fprintf(stderr, "\n[main] Starting main loop now (stream=%s%s, inputIsBGR=%d, lazy=%d)\n",
          (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG"),
          (g_streamCodec == CODEC_MJPEG ? (std::string(", q=") + std::to_string(g_jpeg_quality)).c_str() : ""),
          g_inputIsBGR ? 1 : 0,
          g_lazy_send ? 1 : 0);

  // Buffer to hold last raw frame for lazy send comparison (we compare pre-conversion bytes)
  std::vector<unsigned char> prev_raw_frame;
  bool have_prev_frame = false;

  while (true) {
  while (shouldLoop) {
    accept_new_clients(); // non-blocking accept of new clients

    if (!devInfoMain->realAndTargetRatesMatch) sw.start();
    background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
    background_task_cap_main.wait();

    const size_t frame_bytes = (size_t)devInfoMain->startingSize;

    bool frame_changed = true;
    // Lazy-send: compare against previous captured frame (pre-conversion)
    if (g_lazy_send && have_prev_frame && prev_raw_frame.size() == frame_bytes) {
      // Note: assumes device delivers full-size frames (RGB24). If bytesused were smaller, you'd compare only valid bytes.
      frame_changed = (std::memcmp(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes) != 0);
    }

    if (!have_prev_frame) {
      // init prev buffer on first loop
      prev_raw_frame.resize(frame_bytes);
      std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes);
      have_prev_frame = true;
      frame_changed = true; // first frame should be considered "changed"
    } else if (frame_changed) {
      // refresh previous frame snapshot so next compare is accurate
      std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes);
    }

    // Lazy-send: compare against previous captured frame (pre-conversion)
    // If not changed, skip both encoding and sending to save CPU/bandwidth
    if (g_lazy_send && !frame_changed) {
      if (!devInfoMain->realAndTargetRatesMatch) {
        double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
        if (us > 0) usleep((useconds_t)us);
      }
      continue;
    }

    // Decide what to do based on stream mode
    if (g_streamCodec == CODEC_MJPEG) {
      // Enqueue current frame for background encoding (we already verified it's changed)
      enqueue_encode_job(devInfoMain->outputFrame,
                         frame_bytes,
                         devInfoMain->startingWidth,
                         devInfoMain->startingHeight);

      // Try to broadcast the freshest encoded JPEG (non-blocking). It's okay if encoder hasn't caught up yet.
      auto sp = std::atomic_load_explicit(&g_latest_jpeg, std::memory_order_acquire);
      if (sp && !sp->empty()) {
        broadcast_frame(sp->data(), sp->size());
      }
    } else {
      // RAW RGB24 stream: if capture is BGR and user requested server convert (--bgr),
      // do fast in-place swap (multi-threaded on large frames). Only do it when we're sending.
      if (g_inputIsBGR) {
        size_t pixels = (size_t)devInfoMain->startingWidth * (size_t)devInfoMain->startingHeight;
        swap_rb_inplace_mt(devInfoMain->outputFrame, pixels, std::max(1, g_encode_threads)); // reuse enc thread count as hint
      }
      broadcast_frame(devInfoMain->outputFrame, frame_bytes);
    }

    if (!devInfoMain->realAndTargetRatesMatch) {
      double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
      if (us > 0) usleep((useconds_t)us);
    }
  }
  usleep(2500000);
  shouldLoop.store(true);
  deinit_bufs(buffersMain, devInfoMain);
  if (isDualInput) deinit_bufs(buffersAlt, devInfoAlt);
  usleep(2500000);
  /*init_dev_stage1(buffersMain, devInfoMain);
  init_dev_stage2(buffersMain, devInfoMain);*/
  init_vars(devInfoMain, buffersMain, 3, allDevicesTargetFramerate, true, true, devNames[0].c_str(), 0);
  }

  // allow clients to drain
  usleep(1000000);
  stop_encoder_pool();
  cleanup_vars();
  return 0;
}

// Build (example):
// g++ -O3 -std=gnu++17 v4l2_streamer.cpp -o v4l2_streamer -lv4l2 -lpopt -ljpeg -lpthread
