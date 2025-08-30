// v4l2_streamer.cpp
// Notes:
// - Default stream is raw RGB24 frames over TCP.
// - Enable MJPEG software-encoding with --mjpeg (uses libjpeg).
// - If your capture device produces BGR24 ordering, use --bgr to have the server convert to RGB24
//   (so clients can use -pixel_format rgb24). Alternatively, clients can specify -pixel_format bgr24
//   and you can omit --bgr to avoid the extra memcpy.
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
#include <cstdint>
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
#include <random>
// threading additions
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>

// CRT filter
#include "crt_filter.h"

using namespace std;

#define V4L_ALLFORMATS  3
#define V4L_RAWFORMATS  1
#define V4L_COMPFORMATS 2
#define CLEAR(x) memset(&(x), 0, sizeof(x))

int byteScaler = 3, defaultWidth = 1920, defaultHeight = 1080, numPixels = defaultWidth * defaultHeight;
double allDevicesTargetFramerate = 240;
bool isDualInput = false;
std::atomic<bool> shouldLoop{false};

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

// Lazy-send toggle (default enabled)
bool g_lazy_send = true;
// How many identical frames must be seen before we consider the frame "stale" and render the CRT filter.
int g_lazy_threshold = 1;

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

struct buffer* buffersMain = nullptr;
struct buffer* buffersAlt = nullptr;
struct devInfo* devInfoMain = nullptr;
struct devInfo* devInfoAlt = nullptr;

static std::thread s_restart_thread;
static std::mutex s_restart_mtx;
static std::condition_variable s_restart_cv;
static std::atomic<bool> s_restart_requested{false};
static std::atomic<bool> s_restarting{false};
static std::atomic<bool> done_flag{false};

// Track current streaming resolution even when devices are torn down
static std::atomic<int> g_cur_width{defaultWidth};
static std::atomic<int> g_cur_height{defaultHeight};

// CRT params (portable initializer)
static CRTParams make_default_crt_params() {
  CRTParams p{};
  p.flicker_60hz = 0.904f;
  p.flicker_noise = 0.093f;
  p.scanline_strength = 0.25f; // 0..1
  p.mask_strength = 0.83f;     // 0..1
  p.grain_strength = 0.025f;   // 0..1
  p.h_warp_amp = 0.33f;        // pixels
  p.h_warp_freq_y = 0.03f;     // per-line
  p.h_warp_freq_t = 0.8f;      // per-second
  p.v_shake_amp = 1.2f;        // lines
  p.wobble_line_noise = 0.33f; // extra wobble noise per line (pixels)
  p.block_rows = 32;           // multithread chunk size (rows)
  return p;
}
static CRTParams params = make_default_crt_params();
static size_t threads_hint = std::thread::hardware_concurrency();
static int fps = 60;

// No-signal visual settings
static bool   g_noise_enabled = true;   // true => use noise; false => fall back to solid grey
static double g_noise_sigma   = 1.5;    // Gaussian blur sigma in pixels (0 = no blur)

// Helpers
void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

int xioctl(int fh, int request, void* arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while ((r == -1) && (errno == EINTR));
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

// --- Noise and Gaussian blur helpers ---

static inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Make a 1D Gaussian kernel (normalized). Sigma 0 => {1}.
static std::vector<float> make_gaussian_kernel_1d(float sigma) {
  if (sigma <= 0.0f) return std::vector<float>{1.0f};
  int radius = std::max(1, (int)std::ceil(3.0f * sigma)); // 3*sigma truncation
  std::vector<float> k((size_t)radius * 2 + 1);
  const float s2 = 2.0f * sigma * sigma;
  float sum = 0.0f;
  for (int i = -radius; i <= radius; ++i) {
    float w = std::exp(-(i * i) / s2);
    k[(size_t)(i + radius)] = w;
    sum += w;
  }
  for (float& v : k) v /= sum;
  return k;
}

// Separable Gaussian blur for RGB24. src and dst are size w*h*3.
// Uses two passes (horizontal + vertical) and clamps borders.
static void gaussian_blur_rgb24(const uint8_t* src, uint8_t* dst, int w, int h, float sigma) {
  const size_t row_stride = (size_t)w * 3;
  const size_t pix_stride = 3;
  const size_t total_bytes = (size_t)w * (size_t)h * 3;

  if (sigma <= 0.0f) {
    // No blur: direct copy
    std::memcpy(dst, src, total_bytes);
    return;
  }

  // Cache the kernel per-thread for the given sigma to avoid rebuilding every frame
  thread_local float tl_last_sigma = -1.0f;
  thread_local std::vector<float> tl_kernel;
  if (tl_last_sigma != sigma) {
    tl_kernel = make_gaussian_kernel_1d(sigma);
    tl_last_sigma = sigma;
  }
  const std::vector<float>& k = tl_kernel;
  const int radius = (int)k.size() / 2;

  // Intermediate buffer (horizontal pass)
  std::vector<uint8_t> tmp(total_bytes);

  // Horizontal pass
  for (int y = 0; y < h; ++y) {
    const uint8_t* row_in  = src + (size_t)y * row_stride;
    uint8_t*       row_out = tmp.data() + (size_t)y * row_stride;

    for (int x = 0; x < w; ++x) {
      float accR = 0, accG = 0, accB = 0;
      for (int dx = -radius; dx <= radius; ++dx) {
        int sx = clampi(x + dx, 0, w - 1);
        const uint8_t* p = row_in + (size_t)sx * pix_stride;
        float kv = k[(size_t)(dx + radius)];
        accR += kv * p[0];
        accG += kv * p[1];
        accB += kv * p[2];
      }
      uint8_t* q = row_out + (size_t)x * pix_stride;
      q[0] = (uint8_t)clampi((int)std::lround(accR), 0, 255);
      q[1] = (uint8_t)clampi((int)std::lround(accG), 0, 255);
      q[2] = (uint8_t)clampi((int)std::lround(accB), 0, 255);
    }
  }

  // Vertical pass
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      float accR = 0, accG = 0, accB = 0;
      for (int dy = -radius; dy <= radius; ++dy) {
        int sy = clampi(y + dy, 0, h - 1);
        const uint8_t* p = tmp.data() + (((size_t)sy * (size_t)w + (size_t)x) * pix_stride);
        float kv = k[(size_t)(dy + radius)];
        accR += kv * p[0];
        accG += kv * p[1];
        accB += kv * p[2];
      }
      uint8_t* q = dst + (((size_t)y * (size_t)w + (size_t)x) * pix_stride);
      q[0] = (uint8_t)clampi((int)std::lround(accR), 0, 255);
      q[1] = (uint8_t)clampi((int)std::lround(accG), 0, 255);
      q[2] = (uint8_t)clampi((int)std::lround(accB), 0, 255);
    }
  }
}

// Fill buffer with RGB24 noise (uniform 0..255 each channel)
static void fill_rgb24_noise(std::vector<uint8_t>& buf, int w, int h, std::mt19937& rng) {
  buf.resize((size_t)w * (size_t)h * 3);
  uint8_t* p = buf.data();
  size_t n = buf.size();
  for (size_t i = 0; i < n; i += 4) {
    uint32_t r = rng(); // 4 random bytes
    if (i + 0 < n) p[i + 0] = (uint8_t)( r        & 0xFF);
    if (i + 1 < n) p[i + 1] = (uint8_t)((r >> 8)  & 0xFF);
    if (i + 2 < n) p[i + 2] = (uint8_t)((r >> 16) & 0xFF);
    if (i + 3 < n) p[i + 3] = (uint8_t)((r >> 24) & 0xFF);
  }
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
    return false; // drop client
  }
  return true;
}

// Placeholder (grey) frame cache
struct PlaceholderCache {
  int w = 0, h = 0;
  std::vector<unsigned char> raw;  // RGB24 grey
  std::vector<unsigned char> jpeg; // encoded grey JPEG

  void rebuild_if_needed(int width, int height) {
    if (w == width && h == height && !raw.empty() && (g_streamCodec != CODEC_MJPEG || !jpeg.empty())) return;
    w = width;
    h = height;
    raw.assign((size_t)w * (size_t)h * (size_t)byteScaler, 127); // mid-grey RGB24
    if (g_streamCodec == CODEC_MJPEG) {
      jpeg.clear();
      encode_rgb24_to_jpeg_mem(raw.data(), w, h, g_jpeg_quality, jpeg);
    } else {
      jpeg.clear();
    }
  }
};

static PlaceholderCache g_placeholder;

// Add this helper somewhere near other helpers (before accept_new_clients)
static void seed_initial_frame_for_fd(int fd, int width, int height) {
  g_placeholder.rebuild_if_needed(width, height);
  if (g_streamCodec == CODEC_MJPEG) {
    if (!g_placeholder.jpeg.empty()) {
      send_all_or_drop(fd, g_placeholder.jpeg.data(), g_placeholder.jpeg.size());
    }
  } else {
    if (!g_placeholder.raw.empty()) {
      send_all_or_drop(fd, g_placeholder.raw.data(), g_placeholder.raw.size());
    }
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
    setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    client_fds.push_back(cfd);

    // Seed this new client with initial frames at the actual resolution
    int w = (devInfoMain ? devInfoMain->startingWidth : g_cur_width.load());
    int h = (devInfoMain ? devInfoMain->startingHeight : g_cur_height.load());
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
  int opt_lazy_thresh = 1; // new: number of identical frames to consider stale

  struct poptOption optionsTable[] = {
    { "fps",     'f',    POPT_ARG_DOUBLE,   &opt_fps,      0,    "Target framerate for all devices",                        "FPS" },
    { "devices", 'd',    POPT_ARG_STRING,   &opt_devices,  0,    "V4L2 device(s): /dev/video0 or /dev/video0,/dev/video1",  "DEV[,DEV]" },
    { "port",    'p',    POPT_ARG_INT,      &opt_port,     0,    "TCP listen port for streaming frames",                    "PORT" },
    { "mjpeg",   0,      POPT_ARG_NONE,     &opt_mjpeg,    0,    "Encode and stream Motion-JPEG instead of raw RGB24",      nullptr },
    { "raw",     0,      POPT_ARG_NONE,     &opt_raw,      0,    "Force raw-RGB24 stream (default)",                         nullptr },
    { "jpeg-quality", 'q', POPT_ARG_INT,    &opt_q,        0,    "JPEG quality when --mjpeg (1-100, default 80)",           "Q" },
    { "bgr",     0,      POPT_ARG_NONE,     &opt_bgr,      0,    "Capture is BGR24; server will convert to RGB24 before sending", nullptr },
    { "encode-threads",  0, POPT_ARG_INT,   &opt_enc_threads, 0, "MJPEG encoder threads (0=auto)",                          "N" },
    { "lazy",    0,      POPT_ARG_NONE,     &opt_lazy,     0,    "Enable lazy-send (skip identical frames)",                nullptr },
    { "no-lazy", 0,      POPT_ARG_NONE,     &opt_no_lazy,  0,    "Disable lazy-send",                                       nullptr },
    { "lazy-threshold", 0, POPT_ARG_INT,    &opt_lazy_thresh, 0, "Number of consecutive identical frames required to mark stale (default 1)", "N" },
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
    fprintf(stderr, "      --lazy-threshold       Number of consecutive identical frames required to mark stale (default 1)\n");
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

  // apply lazy threshold
  if (opt_lazy_thresh < 1) opt_lazy_thresh = 1;
  g_lazy_threshold = opt_lazy_thresh;

  std::string stream_name = (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG");
  std::string stream_extra = (g_streamCodec == CODEC_MJPEG ? (" (q=" + std::to_string(g_jpeg_quality) + ")") : "");
  fprintf(stderr, "[main] Parsed options: --fps=%.3f, --devices=%s%s%s, --port=%d, --stream=%s%s, --bgr=%d, --encode-threads=%d, --lazy=%d, --lazy-threshold=%d\n",
          allDevicesTargetFramerate,
          devNames[0].c_str(),
          isDualInput ? "," : "",
          isDualInput ? devNames[1].c_str() : "",
          listenPort,
          stream_name.c_str(),
          stream_extra.c_str(),
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

// Swap R/B in-place for RGB24/BGR24 conversion (single-thread)
static inline void swap_rb_inplace(unsigned char* buf, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    unsigned char* p = buf + i * 3;
    unsigned char t = p[0];
    p[0] = p[2];
    p[2] = t;
  }
}

// Multi-threaded in-place R<->B swap for large frames
static inline void swap_rb_inplace_mt(unsigned char* buf, size_t pixels, int threads) {
  if (threads <= 1 || pixels < 512ull * 512ull) {
    swap_rb_inplace(buf, pixels);
    return;
  }
  unsigned hc = std::thread::hardware_concurrency();
  threads = std::min<int>(threads, hc ? (int)hc : 2);
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
    crop.c = cropcap.defrect; // reset to default
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_CROP, &crop)) {
      // ignore
    }
  }

  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fprintf(stderr, "[cap%d] Forcing format for %s to: %d\n", devInfos->index, devInfos->device, devInfos->force_format);

  if (devInfos->force_format) {
    if (devInfos->force_format == 3) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
    } else if (devInfos->force_format == 2) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
    } else if (devInfos->force_format == 1) {
      byteScaler = devInfos->force_format;
      fmt.fmt.pix.width = devInfos->startingWidth;
      fmt.fmt.pix.height = devInfos->startingHeight;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }
    if (-1 == xioctl(devInfos->fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");
  } else {
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

  if (devInfos->isTC358743) {
    struct v4l2_dv_timings timings;
    v4l2_std_id std;
    int ret;
    memset(&timings, 0, sizeof timings);
    ret = xioctl(devInfos->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
    if (ret >= 0) {
      fprintf(stderr, "[cap%d] QUERY_DV_TIMINGS for %s: %ux%ux%d pixclk %llu\n",
              devInfos->index, devInfos->device, timings.bt.width, timings.bt.height, byteScaler, timings.bt.pixelclock);
      devInfos->startingWidth = timings.bt.width;
      devInfos->startingHeight = timings.bt.height;
      devInfos->startingSize = devInfos->startingWidth * devInfos->startingHeight * byteScaler;

      ret = xioctl(devInfos->fd, VIDIOC_S_DV_TIMINGS, &timings);
      if (ret < 0) {
        fprintf(stderr, "[cap%d] Failed to set DV timings\n", devInfos->index);
        return 1;
      } else {
        double tot_height, tot_width;
        const struct v4l2_bt_timings* bt = &timings.bt;
        tot_height = bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch
                   + bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
        tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
        devInfos->framerate = (unsigned int)((double)bt->pixelclock / (tot_width * tot_height));

        if (devInfos->framerate < devInfos->targetFramerate) devInfos->targetFramerate = devInfos->framerate;

        devInfos->framerateDivisor = (devInfos->framerate / devInfos->targetFramerate);
        devInfos->frameDelayMicros = (1000000.0 / devInfos->framerate);
        devInfos->frameDelayMillis = (1000.0 / devInfos->framerate);
        devInfos->targetFrameDelayMicros = (1000000.0 / devInfos->framerate) * devInfos->framerateDivisor;
        devInfos->targetFrameDelayMillis = (1000.0 / devInfos->framerate) * devInfos->framerateDivisor;

        int rawInputThroughput = (int)((double)(devInfos->framerate * devInfos->startingSize) / 125000.0); // Mb/s
        int rawOutputThroughput = (int)(((double)(devInfos->framerate * devInfos->startingSize) / 125000.0) / devInfos->framerateDivisor); // Mb/s

        devInfos->realAndTargetRatesMatch = doubles_equal(devInfos->frameDelayMicros, devInfos->targetFrameDelayMicros);
        fprintf(stderr, "[cap%d] device_name: %s, startingWidth: %d, startingHeight: %d, byteScaler: %d, startingSize: %d, framerate(actual): %u, framerateDivisor: %f, targetFramerate: %f, frameDelayMicros: %f, frameDelayMillis: %f, targetFrameDelayMicros: %f, targetFrameDelayMillis: %f, realAndTargetRatesMatch: %d\n",
          devInfos->index, devInfos->device, devInfos->startingWidth, devInfos->startingHeight, byteScaler, devInfos->startingSize, devInfos->framerate, devInfos->framerateDivisor, devInfos->targetFramerate, devInfos->frameDelayMicros, devInfos->frameDelayMillis, devInfos->targetFrameDelayMicros, devInfos->targetFrameDelayMillis, devInfos->realAndTargetRatesMatch);
        fprintf(stderr, "[cap%d] device_name: %s, isTC358743: %d, rawInputThroughput: ~%dMb/~%dMiB/~%dMB/sec, rawOutputThroughput: ~%dMb/~%dMiB/~%dMB/sec\n",
          devInfos->index, devInfos->device, devInfos->isTC358743,
          rawInputThroughput, (int)((double)rawInputThroughput / 8.389), rawInputThroughput / 8,
          rawOutputThroughput, (int)((double)rawOutputThroughput / 8.389), rawOutputThroughput / 8);
      }
    } else {
      memset(&std, 0, sizeof std);
      ret = ioctl(devInfos->fd, VIDIOC_QUERYSTD, &std);
      if (ret >= 0) {
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
      // ignore
    }
  }

  devInfos->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(devInfos->fd, VIDIOC_STREAMON, &devInfos->type)) {
    // ignore
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
    shouldLoop.store(false, std::memory_order_release);
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
        shouldLoop.store(false, std::memory_order_release);
        return 1;
    }
  }

  assert(buf.index < devInfos->n_buffers);

  size_t valid = buf.bytesused ? buf.bytesused : buffers[buf.index].length;
  size_t copy_len = std::min<size_t>(valid, (size_t)devInfos->startingSize);
  std::memcpy(devInfos->outputFrame, (unsigned char*)buffers[buf.index].start, copy_len); // copy frame data to frame buffer

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
  for (unsigned int i = 0; devInfos && i < devInfos->n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
      errno_exit("munmap");

  if (buffers) {
    free(buffers);
    buffers = nullptr;
  }

  fprintf(stderr, "[cap%d] Uninitialized V4L2 device: %s\n", devInfos->index, devInfos->device);
  if (devInfos->fd >= 0 && -1 == close(devInfos->fd)) errno_exit("close");
  if (devInfos) devInfos->fd = -1;

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
  devInfos->device = (char*)calloc(strlen(dev_name) + 1, sizeof(char));
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

  g_cur_width.store(devInfos->startingWidth, std::memory_order_release);
  g_cur_height.store(devInfos->startingHeight, std::memory_order_release);

  // flip back to active capture
  shouldLoop.store(true, std::memory_order_release);

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
  fprintf(stderr, "[main] Initializing..\n");
  init_vars(deviMain, bufMain, 3, allDevicesTargetFramerate, true, true, devNames[0].c_str(), 0);
  if (isDualInput) init_vars(deviAlt, bufAlt, 3, allDevicesTargetFramerate, true, true, devNames[1].c_str(), 1);

  numPixels = devInfoMain->startingWidth * devInfoMain->startingHeight;
  usleep(1000);
  fprintf(stderr, "\n");
}

static void restart_worker() {
  while (!done_flag.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> lk(s_restart_mtx);
    s_restart_cv.wait(lk, []{ return s_restart_requested.load(std::memory_order_acquire) || done_flag.load(std::memory_order_acquire); });
    if (done_flag.load(std::memory_order_acquire)) break;

    s_restart_requested.store(false, std::memory_order_release);
    s_restarting.store(true, std::memory_order_release);
    lk.unlock();

    // Tear down devices (do NOT touch sockets)
    deinit_bufs(buffersMain, devInfoMain);
    if (isDualInput) deinit_bufs(buffersAlt, devInfoAlt);
    free_devinfo(devInfoMain);
    if (isDualInput) free_devinfo(devInfoAlt);

    // backoff
    usleep(2500000);

    // Re-init devices
    init_vars(devInfoMain, buffersMain, 3, allDevicesTargetFramerate, true, true, devNames[0].c_str(), 0);
    if (isDualInput) {
      init_vars(devInfoAlt, buffersAlt, 3, allDevicesTargetFramerate, true, true, devNames[1].c_str(), 1);
    }

    s_restarting.store(false, std::memory_order_release);
  }
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
  thread_local std::vector<unsigned char> tls_rgb;
  while (true) {
    EncodeJob job;
    {
      std::unique_lock<std::mutex> lk(s_enc_mtx);
      s_enc_cv.wait(lk, []{ return s_enc_stop.load(std::memory_order_acquire) || !s_enc_queue.empty(); });
      if (s_enc_stop.load(std::memory_order_acquire) && s_enc_queue.empty()) return;
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
  if (g_encode_threads > 32) g_encode_threads = 32;
  g_encode_queue_max = std::max<size_t>(g_encode_threads * 2, 2);

  fprintf(stderr, "[enc] Starting encoder pool: threads=%d, queue_max=%zu, frame=%dx%d, quality=%d, bgr=%d\n",
          g_encode_threads, g_encode_queue_max, width, height, g_jpeg_quality, g_inputIsBGR ? 1 : 0);

  s_enc_stop.store(false, std::memory_order_release);
  s_enc_workers.reserve((size_t)g_encode_threads);
  for (int i = 0; i < g_encode_threads; ++i) {
    s_enc_workers.emplace_back(encoder_worker_loop);
  }
}

static void stop_encoder_pool() {
  if (s_enc_workers.empty()) return;
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    s_enc_stop.store(true, std::memory_order_release);
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

// ----------------- main -----------------
int main(const int argc, char** argv) {
  if (!threads_hint) threads_hint = 4;
  // Parse options and devices first
  parse_cli_or_die(argc, (const char**)argv);
  // Initialize capture
  configure_main(devInfoMain, buffersMain, devInfoAlt, buffersAlt);
  // CRT filter
  CRTFilter filter(devInfoMain->startingWidth, devInfoMain->startingHeight, params, threads_hint, fps);
  // Restart thread
  s_restart_thread = std::thread(restart_worker);
  // Setup TCP server
  listen_fd = setup_server_socket(listenPort);
  if (listen_fd < 0) {
    fprintf(stderr, "[main] Failed to create TCP listening socket on port %d\n", listenPort);
    cleanup_vars();
    done_flag.store(true, std::memory_order_release);
    s_restart_cv.notify_all();
    if (s_restart_thread.joinable()) s_restart_thread.join();
    return 1;
  }
  // Start encoder pool if MJPEG mode
  if (g_streamCodec == CODEC_MJPEG) {
    start_encoder_pool_if_needed(devInfoMain->startingWidth, devInfoMain->startingHeight);
  }
  MicroStopwatch sw;
  usleep(1000);
  // Warm-up: capture and ignore some frames
  for (int i = 0; i < 2; i++) {
    accept_new_clients();
    if (isDualInput) {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_alt  = std::async(std::launch::async, get_frame, buffersAlt,  devInfoAlt);
      background_task_cap_main.wait();
      background_task_cap_alt.wait();
    } else {
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
    }
  }
  std::string stream_name = (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG");
  std::string stream_extra = (g_streamCodec == CODEC_MJPEG ? (", q=" + std::to_string(g_jpeg_quality)) : "");
  fprintf(stderr, "\n[main] Starting main loop now (stream=%s%s, inputIsBGR=%d, lazy=%d)\n", stream_name.c_str(), stream_extra.c_str(), g_inputIsBGR ? 1 : 0, g_lazy_send ? 1 : 0);
  // Lazy-send state
  std::vector<unsigned char> prev_raw_frame;
  bool have_prev_frame = false;
  int consecutive_same_count = 0;
  // Continuous main loop: always run, either capturing or streaming grey
  while (!done_flag.load(std::memory_order_acquire)) {
    sw.start();
    accept_new_clients();
    const bool active = shouldLoop.load(std::memory_order_acquire);
    // Keep placeholder in sync with the current or last-known dimensions
    int cur_w = g_cur_width.load(std::memory_order_acquire);
    int cur_h = g_cur_height.load(std::memory_order_acquire);
    g_placeholder.rebuild_if_needed(cur_w, cur_h);
    if (active) {
      // Capture path
      if (!devInfoMain->realAndTargetRatesMatch) sw.start();
      // Fetch a frame
      background_task_cap_main = std::async(std::launch::async, get_frame, buffersMain, devInfoMain);
      background_task_cap_main.wait();
      // If shouldLoop was flipped false by get_frame (error/timeout), fall through to placeholder branch
      if (!shouldLoop.load(std::memory_order_acquire)) {
        // Request a restart once
        {
          std::lock_guard<std::mutex> lk(s_restart_mtx);
          if (!s_restart_requested.load(std::memory_order_acquire)) {
            s_restart_requested.store(true, std::memory_order_release);
          }
        }
        s_restart_cv.notify_one();
      } else {
        // Process/send live frame
        const size_t frame_bytes = (size_t)devInfoMain->startingSize;
        bool frame_changed = true;
        bool is_same = false;
        if (g_lazy_send && have_prev_frame && prev_raw_frame.size() == frame_bytes) {
          is_same = (std::memcmp(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes) == 0);
        }
        if (!have_prev_frame) {
          prev_raw_frame.resize(frame_bytes);
          std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes);
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
            if (prev_raw_frame.size() == frame_bytes) {
              std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes);
            } else {
              prev_raw_frame.resize(frame_bytes);
              std::memcpy(prev_raw_frame.data(), devInfoMain->outputFrame, frame_bytes);
            }
          }
        }
        if (g_lazy_send && !frame_changed) {
          if (consecutive_same_count < g_lazy_threshold) {
            if (!devInfoMain->realAndTargetRatesMatch) {
              double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
              if (us > 0) usleep((useconds_t)us);
            }
          } else {
            // Stale threshold reached: render CRT filter and send
            const int w = devInfoMain->startingWidth;
            const int h = devInfoMain->startingHeight;
            const uint8_t* src_rgb = reinterpret_cast<const uint8_t*>(devInfoMain->outputFrame);
            std::vector<uint8_t> tmp_rgb;
            if (g_inputIsBGR) {
              tmp_rgb.resize(frame_bytes);
              const unsigned char* s = devInfoMain->outputFrame;
              unsigned char* d = tmp_rgb.data();
              const size_t pixels = (size_t)w * (size_t)h;
              for (size_t i = 0; i < pixels; ++i) {
                d[0] = s[2];
                d[1] = s[1];
                d[2] = s[0];
                s += 3; d += 3;
              }
              src_rgb = tmp_rgb.data();
            }
            std::vector<uint8_t> filtered_rgb;
            filtered_rgb.reserve(frame_bytes);
            filter.apply(src_rgb, filtered_rgb); // outputs RGB24
            if (g_streamCodec == CODEC_MJPEG) {
              std::vector<unsigned char> jpeg;
              if (encode_rgb24_to_jpeg_mem(filtered_rgb.data(), w, h, g_jpeg_quality, jpeg) && !jpeg.empty()) {
                broadcast_frame(jpeg.data(), jpeg.size());
              }
            } else {
              broadcast_frame(filtered_rgb.data(), filtered_rgb.size());
            }
            if (!devInfoMain->realAndTargetRatesMatch) {
              double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
              if (us > 0) usleep((useconds_t)us);
            }
          }
        } else {
          // Live changed frame
          if (g_streamCodec == CODEC_MJPEG) {
            enqueue_encode_job(devInfoMain->outputFrame, frame_bytes, devInfoMain->startingWidth, devInfoMain->startingHeight);
            auto sp = std::atomic_load_explicit(&g_latest_jpeg, std::memory_order_acquire);
            if (sp && !sp->empty()) {
              broadcast_frame(sp->data(), sp->size());
            }
          } else {
            if (g_inputIsBGR) {
              size_t pixels = (size_t)devInfoMain->startingWidth * (size_t)devInfoMain->startingHeight;
              swap_rb_inplace_mt(devInfoMain->outputFrame, pixels, std::max(1, g_encode_threads));
            }
            broadcast_frame(devInfoMain->outputFrame, frame_bytes);
          }
          if (!devInfoMain->realAndTargetRatesMatch) {
            double us = devInfoMain->targetFrameDelayMicros - sw.elapsedMicros();
            if (us > 0) usleep((useconds_t)us);
          }
        }
      }
    } else {
      // No-signal path: continuously stream placeholder frames (noise + optional Gaussian blur)
      const int w = g_cur_width.load(std::memory_order_acquire);
      const int h = g_cur_height.load(std::memory_order_acquire);
      // Trigger restart if not already doing so
      if (!s_restarting.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(s_restart_mtx);
        if (!s_restart_requested.load(std::memory_order_acquire)) {
          s_restart_requested.store(true, std::memory_order_release);
          s_restart_cv.notify_one();
        }
      }
      // Build and send one frame (either noise+blur or solid grey)
      if (g_noise_enabled) {
        // Thread-local RNG to avoid contention and cost of reseeding
        thread_local std::mt19937 rng(std::random_device{}());
        // Reusable buffers
        static thread_local std::vector<uint8_t> noise;
        static thread_local std::vector<uint8_t> blurred;
        fill_rgb24_noise(noise, w, h, rng);
        blurred.resize((size_t)w * (size_t)h * 3);
        // Apply Gaussian blur with sigma = g_noise_sigma
        gaussian_blur_rgb24(noise.data(), blurred.data(), w, h, (float)g_noise_sigma);
        if (g_streamCodec == CODEC_MJPEG) {
          std::vector<unsigned char> jpeg;
          if (encode_rgb24_to_jpeg_mem(blurred.data(), w, h, g_jpeg_quality, jpeg) && !jpeg.empty()) {
            broadcast_frame(jpeg.data(), jpeg.size());
          }
        } else {
          broadcast_frame(blurred.data(), blurred.size());
        }
      } else {
        // Fallback: solid grey placeholder (existing behavior)
        g_placeholder.rebuild_if_needed(w, h);
        if (g_streamCodec == CODEC_MJPEG) {
          if (!g_placeholder.jpeg.empty()) broadcast_frame(g_placeholder.jpeg.data(), g_placeholder.jpeg.size());
        } else {
          if (!g_placeholder.raw.empty()) broadcast_frame(g_placeholder.raw.data(), g_placeholder.raw.size());
        }
      }
      // Pace frames at target framerate
      double target_fps = std::max(1.0, allDevicesTargetFramerate);
      double frame_us = 1000000.0 / target_fps;
      double us = frame_us - sw.elapsedMicros();
      if (us > 0) usleep((useconds_t)us);
      // reset lazy state so when capture resumes we don't misclassify
      have_prev_frame = false;
      consecutive_same_count = 0;
    }
  }
  // allow clients to drain
  usleep(1000000);
  stop_encoder_pool();
  cleanup_vars();
  done_flag.store(true, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lk(s_restart_mtx);
    s_restart_requested.store(true, std::memory_order_release);
  }
  s_restart_cv.notify_all();
  if (s_restart_thread.joinable()) s_restart_thread.join();
  return 0;
}
