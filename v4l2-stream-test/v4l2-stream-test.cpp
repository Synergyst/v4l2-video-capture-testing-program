// v4l2-stream-test.cpp
// Client-side compute with CUDA vignette:
// - Connects to capture server, receives DEVINFO/FRAME/SIGNAL packets.
// - Performs BGR->RGB swap on RAW frames (client-side responsibility).
// - Implements lazy-send and CRT fallback on stale frames.
// - Applies CUDA-based vignette to frames (normal and CRT/no-signal paths).
// - Re-broadcasts locally as RAW or MJPEG via a TCP server.

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
#include <sys/types.h>
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
#include <cctype>

// Keep CRT and PNG loader
#include "crt_filter_cuda.h"
#include "png_loader.h"

// CUDA vignette
#include "vignette_cuda.h"

using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

// ---------------- Common Packet Protocol (must match server) ----------------
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
  uint32_t magic;
  uint16_t version;
  uint16_t type;
  uint32_t headerSize;
  uint64_t payloadSize;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DevInfoMeta {
  uint16_t dev_index;
  uint16_t byteScaler;
  uint32_t width;
  uint32_t height;
  uint32_t framerate;
  uint32_t target_fps_x1000;
  uint8_t  inputIsBGR;
  uint8_t  isTC358743;
  uint8_t  streamCodec;
  uint8_t  reserved;
  uint64_t frameDelayMicros_x1000;
  uint32_t deviceNameLen;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct FrameMeta {
  uint64_t frame_id;
  uint64_t timestamp_us;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint8_t  codec;
  uint8_t  is_bgr;
  uint16_t reserved;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SignalMeta {
  uint8_t state;
  uint8_t reserved[7];
};
#pragma pack(pop)

} // namespace netpkt

// ---------------- Local streaming server (re-broadcast) ----------------
static int listenPort = 0;
static int listen_fd = -1;
static std::vector<int> client_fds;

enum StreamCodec { CODEC_RAW = 0, CODEC_MJPEG = 1 };
static StreamCodec g_streamCodec = CODEC_RAW; // outgoing to local clients
static int g_jpeg_quality = 80;
static bool g_lazy_send = true;
static int g_lazy_threshold = 1;
static int g_encode_threads = 0;

static std::atomic<bool> programRunning{true};

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
  fprintf(stderr, "[net] Re-broadcast listening on port %d\n", port);
  return fd;
}
static bool send_all(int fd, const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
    if (n > 0) { sent += (size_t)n; continue; }
    if (n < 0 && (errno == EINTR)) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
    return false;
  }
  return true;
}
static void accept_new_clients(int width, int height, int byteScaler) {
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
    fprintf(stderr, "[net] Client connected: %s:%d (fd=%d). Total: %zu\n", ip, ntohs(cliaddr.sin_port), cfd, client_fds.size());
    size_t frame_len = (size_t)width * (size_t)height * (size_t)byteScaler;
    std::vector<unsigned char> seed(frame_len, 0);
    send_all(cfd, seed.data(), seed.size());
  }
}
static void broadcast_frame_local(const unsigned char* data, size_t len) {
  for (size_t i = 0; i < client_fds.size();) {
    int fd = client_fds[i];
    if (!send_all(fd, data, len)) {
      fprintf(stderr, "[net] Dropping client fd=%d\n", fd);
      close(fd);
      client_fds.erase(client_fds.begin() + i);
      continue;
    }
    ++i;
  }
}

// --------------- JPEG helpers ---------------
static bool encode_rgb24_to_jpeg_mem(const unsigned char* rgb, int width, int height, int quality, std::vector<unsigned char>& out) {
  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  unsigned char* jpegBuf = nullptr;
  unsigned long jpegSize = 0;
  jpeg_mem_dest(&cinfo, &jpegBuf, &jpegSize);
  cinfo.image_width = width; cinfo.image_height = height;
  cinfo.input_components = 3; cinfo.in_color_space = JCS_RGB;
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

// --------------- Downstream processing state ---------------
struct DevState {
  int width = 0;
  int height = 0;
  int byteScaler = 3;
  int framerate = 60;
  double targetFps = 60.0;
  double frameDelayMicros = 16666.0;
  bool isTC358743 = true;
  bool inputIsBGR = false; // from upstream meta
  StreamCodec netCodec = CODEC_RAW;
  std::string deviceName;
};

static inline size_t frame_bytes(const DevState& s) {
  return (size_t)s.width * (size_t)s.height * (size_t)s.byteScaler;
}
static inline void swap_rb_inplace_mt(unsigned char* buf, size_t pixels, int threads) {
  if (threads <= 1 || pixels < 512ull * 512ull) {
    for (size_t i = 0; i < pixels; ++i) {
      unsigned char* p = buf + i * 3;
      unsigned char t = p[0]; p[0] = p[2]; p[2] = t;
    }
    return;
  }
  threads = std::min<int>(threads, std::thread::hardware_concurrency() ? (int)std::thread::hardware_concurrency() : 2);
  size_t chunk = (pixels + threads - 1) / threads;
  std::vector<std::thread> ths;
  ths.reserve(threads);
  for (int t = 0; t < threads; ++t) {
    size_t startPix = (size_t)t * chunk; if (startPix >= pixels) break;
    size_t endPix = std::min(pixels, startPix + chunk);
    ths.emplace_back([buf, startPix, endPix]() {
      unsigned char* p = buf + startPix * 3;
      for (size_t i = startPix; i < endPix; ++i) {
        unsigned char tmp = p[0]; p[0] = p[2]; p[2] = tmp; p += 3;
      }
    });
  }
  for (auto& th : ths) th.join();
}
static inline bool frames_likely_equal_fast(const uint8_t* a, const uint8_t* b, int w, int h) {
  const int stride = w * 3;
  const int samples = 8;
  for (int i = 0; i < samples; ++i) {
    int y = (h * (i + 1)) / (samples + 1);
    const uint8_t* ra = a + y * stride;
    const uint8_t* rb = b + y * stride;
    if (memcmp(ra, rb, std::min(stride, 256)) != 0) return false;
  }
  return true;
}

// CRT helpers
struct CRTContext {
  CRTParams params{};
  std::unique_ptr<CRTFilter> filter;
  size_t threads = 0;
  int fps = 60;
  int w = 0, h = 0;
};
static void crtctx_init_defaults(CRTContext& ctx) {
  ctx.params.flicker_60hz = 0.394f;
  ctx.params.flicker_noise = 0.1393f;
  ctx.params.scanline_strength = 0.925f;
  ctx.params.mask_strength = 0.83f;
  ctx.params.grain_strength = 0.0125f;
  ctx.params.h_warp_amp = 0.33f;
  ctx.params.h_warp_freq_y = 0.03f;
  ctx.params.h_warp_freq_t = 0.8f;
  ctx.params.v_shake_amp = 1.2f;
  ctx.params.wobble_line_noise = 0.36f;
  ctx.params.block_rows = 32;
  size_t th = std::thread::hardware_concurrency();
  ctx.threads = (th == 0 ? 4 : th);
  ctx.fps = 60;
}
static void ensure_crt_filter(CRTContext& ctx, int w, int h) {
  if (!ctx.filter || ctx.w != w || ctx.h != h) {
    ctx.w = w; ctx.h = h;
    ctx.filter = std::make_unique<CRTFilter>(w, h, ctx.params, ctx.threads, ctx.fps);
  }
}

// --------------- Encoder thread pool (for local MJPEG output) ---------------
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
static std::shared_ptr<std::vector<unsigned char>> g_latest_jpeg;
static std::atomic<uint64_t> g_latest_jpeg_id{0};
static std::atomic<uint64_t> s_frame_id_counter{0};

static void encoder_worker_loop() {
  while (true) {
    EncodeJob job;
    {
      std::unique_lock<std::mutex> lk(s_enc_mtx);
      s_enc_cv.wait(lk, []{ return s_enc_stop.load() || !s_enc_queue.empty(); });
      if (s_enc_stop.load() && s_enc_queue.empty()) return;
      job = std::move(s_enc_queue.front());
      s_enc_queue.pop_front();
    }
    std::vector<unsigned char> out;
    if (!encode_rgb24_to_jpeg_mem(job.frame.data(), job.width, job.height, g_jpeg_quality, out)) continue;
    uint64_t prev = g_latest_jpeg_id.load(std::memory_order_acquire);
    while (job.id > prev) {
      if (g_latest_jpeg_id.compare_exchange_weak(prev, job.id, std::memory_order_acq_rel)) {
        std::atomic_store_explicit(&g_latest_jpeg, std::make_shared<std::vector<unsigned char>>(std::move(out)), std::memory_order_release);
        break;
      }
    }
  }
}
static void start_encoder_pool_if_needed() {
  if (g_streamCodec != CODEC_MJPEG) return;
  if (!g_encode_threads || g_encode_threads < 0) {
    unsigned hc = std::thread::hardware_concurrency();
    g_encode_threads = (hc > 1) ? std::max(1u, hc - 1) : 1u;
  }
  if (!s_enc_workers.empty()) return;
  s_enc_stop.store(false);
  s_enc_workers.reserve((size_t)g_encode_threads);
  for (int i = 0; i < g_encode_threads; ++i) s_enc_workers.emplace_back(encoder_worker_loop);
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
}
static void enqueue_encode_job(const unsigned char* frameData, size_t bytes, int width, int height) {
  EncodeJob job;
  job.frame.assign(frameData, frameData + bytes);
  job.width = width;
  job.height = height;
  job.id = s_frame_id_counter.fetch_add(1, std::memory_order_relaxed) + 1;
  {
    std::lock_guard<std::mutex> lk(s_enc_mtx);
    if (s_enc_queue.size() >= (size_t)std::max(2, g_encode_threads * 2)) s_enc_queue.pop_front();
    s_enc_queue.emplace_back(std::move(job));
  }
  s_enc_cv.notify_one();
}

// --------------- Upstream connection (receiver) ---------------
static std::string upstreamHost = "192.168.168.175";
static int upstreamPort = 1337;

static int connect_to_server(const std::string& host, int port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) { perror("[upstream] socket"); return -1; }
  sockaddr_in addr{};
  addr.sin_family = AF_INET; addr.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
    fprintf(stderr, "[upstream] inet_pton failed for %s\n", host.c_str());
    close(fd); return -1;
  }
  if (connect(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("[upstream] connect");
    close(fd); return -1;
  }
  set_blocking(fd);
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
  fprintf(stderr, "[upstream] Connected to %s:%d\n", host.c_str(), port);
  return fd;
}
static bool read_exact(int fd, void* buf, size_t len) {
  uint8_t* p = (uint8_t*)buf;
  size_t got = 0;
  while (got < len) {
    ssize_t n = recv(fd, p + got, len - got, 0);
    if (n > 0) { got += (size_t)n; continue; }
    if (n == 0) return false;
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
    return false;
  }
  return true;
}

// No-signal handling on the client
static std::atomic<bool> upstreamSignalUp{true};
static std::atomic<bool> nosignal_thread_running{false};
static std::thread nosignal_thread;
static PNGImage png_ctx; // int width,height; std::vector<uint8_t> rgb;

static void load_or_fallback_png(const DevState& s) {
  if (png_ctx.rgb.empty()) {
    if (!load_png_rgb24("/root/v4l2-stream-test/nosignal.png", png_ctx, false) ||
        png_ctx.rgb.empty()) {
      png_ctx.width = s.width > 0 ? s.width : 1920;
      png_ctx.height = s.height > 0 ? s.height : 1080;
      png_ctx.rgb.assign((size_t)png_ctx.width * (size_t)png_ctx.height * 3ull, 0);
    }
  }
}

// CUDA vignette config and lock (serialize GPU usage across threads)
static std::mutex g_vignette_cuda_mtx;
static bool g_enable_vignette = true;
static VignetteParamsCUDA g_vig {
  .center_x = 0.5f,
  .center_y = 0.5f,
  .inner_radius = 0.073f,
  .outer_radius = 0.098f,
  .strength = 1.85f,
  .axis_scale_x = 0.1957f,
  .axis_scale_y = 0.0471f,
  .angle_rad = 0.0f
};
static void apply_vignette_cuda_locked(uint8_t* rgb, int w, int h) {
  if (!g_enable_vignette) return;
  std::lock_guard<std::mutex> lk(g_vignette_cuda_mtx);
  (void)apply_vignette_cuda(rgb, w, h, &g_vig);
}

static void nosignal_worker(DevState stateCopy) {
  nosignal_thread_running.store(true);
  load_or_fallback_png(stateCopy);
  CRTContext crt; crtctx_init_defaults(crt);
  ensure_crt_filter(crt, stateCopy.width, stateCopy.height);

  std::vector<uint8_t> working;
  const bool match = (png_ctx.width == stateCopy.width && png_ctx.height == stateCopy.height);
  if (!match) {
    working.assign((size_t)stateCopy.width * (size_t)stateCopy.height * 3ull, 0);
  }

  const double us_delay = std::max(1000.0, stateCopy.frameDelayMicros);
  while (programRunning.load() && !upstreamSignalUp.load()) {
    accept_new_clients(stateCopy.width, stateCopy.height, 3);

    const uint8_t* src = match ? png_ctx.rgb.data() : working.data();
    std::vector<uint8_t> filtered;
    filtered.reserve((size_t)stateCopy.width * (size_t)stateCopy.height * 3ull);
    ensure_crt_filter(crt, stateCopy.width, stateCopy.height);
    crt.filter->apply(src, filtered);
    // Apply CUDA vignette
    apply_vignette_cuda_locked(filtered.data(), stateCopy.width, stateCopy.height);

    if (g_streamCodec == CODEC_MJPEG) {
      std::vector<unsigned char> jpeg;
      if (encode_rgb24_to_jpeg_mem(filtered.data(), stateCopy.width, stateCopy.height, g_jpeg_quality, jpeg) && !jpeg.empty()) {
        broadcast_frame_local(jpeg.data(), jpeg.size());
      }
    } else {
      broadcast_frame_local(filtered.data(), filtered.size());
    }

    usleep((useconds_t)us_delay);
  }
  nosignal_thread_running.store(false);
}
static void start_nosignal_if_needed(const DevState& s) {
  if (!nosignal_thread_running.load() && s.width > 0 && s.height > 0) {
    upstreamSignalUp.store(false);
    DevState copy = s;
    nosignal_thread = std::thread(nosignal_worker, copy);
  }
}
static void stop_nosignal_if_running() {
  if (nosignal_thread_running.load()) {
    upstreamSignalUp.store(true);
    if (nosignal_thread.joinable()) nosignal_thread.join();
  }
}

static bool recv_and_handle_packet(int upstream_fd, DevState& state, std::vector<uint8_t>& current_frame_rgb) {
  using namespace netpkt;
  Header hdr{};
  if (!read_exact(upstream_fd, &hdr, sizeof(hdr))) return false;
  if (ntohl(hdr.magic) != MAGIC || ntohs(hdr.version) != PROTO_VER) {
    fprintf(stderr, "[upstream] Bad header magic/version\n");
    return false;
  }
  uint16_t type = ntohs(hdr.type);
  uint32_t headerSize = ntohl(hdr.headerSize);
  uint64_t payloadSize = ntohll(hdr.payloadSize);
  const size_t base = sizeof(Header);
  if (headerSize < base) { fprintf(stderr, "[upstream] headerSize too small\n"); return false; }
  size_t metaBytes = headerSize - base;

  std::vector<uint8_t> meta;
  meta.resize(metaBytes);
  if (metaBytes && !read_exact(upstream_fd, meta.data(), metaBytes)) return false;

  if (type == PT_DEVINFO) {
    if (metaBytes < sizeof(DevInfoMeta)) { fprintf(stderr, "[upstream] DEVINFO meta too small\n"); return false; }
    DevInfoMeta m{};
    memcpy(&m, meta.data(), sizeof(m));
    uint32_t devNameLen = ntohl(m.deviceNameLen);
    if (metaBytes < sizeof(DevInfoMeta) + devNameLen) { fprintf(stderr, "[upstream] DEVINFO devname truncated\n"); return false; }
    state.width = (int)ntohl(m.width);
    state.height = (int)ntohl(m.height);
    state.byteScaler = (int)ntohs(m.byteScaler);
    state.framerate = (int)ntohl(m.framerate);
    state.targetFps = (double)ntohl(m.target_fps_x1000) / 1000.0;
    state.frameDelayMicros = (double)ntohll(m.frameDelayMicros_x1000) / 1000.0;
    state.isTC358743 = (m.isTC358743 != 0);
    state.inputIsBGR = (m.inputIsBGR != 0);
    state.netCodec = (StreamCodec)m.streamCodec;
    state.deviceName.assign((const char*)meta.data() + sizeof(DevInfoMeta), devNameLen);

    fprintf(stderr, "[upstream] DEVINFO: %dx%d scaler=%d fps=%d target=%.3f BGR=%d dev=%s\n",
      state.width, state.height, state.byteScaler, state.framerate, state.targetFps, state.inputIsBGR ? 1 : 0, state.deviceName.c_str());
    current_frame_rgb.resize((size_t)state.width * (size_t)state.height * 3ull);
    return true;
  } else if (type == PT_SIGNAL) {
    if (metaBytes < sizeof(SignalMeta)) { fprintf(stderr, "[upstream] SIGNAL meta too small\n"); return false; }
    SignalMeta sm{}; memcpy(&sm, meta.data(), sizeof(sm));
    if (sm.state == SIGNAL_DOWN) {
      fprintf(stderr, "[upstream] SIGNAL_DOWN received\n");
      upstreamSignalUp.store(false);
    } else if (sm.state == SIGNAL_UP) {
      fprintf(stderr, "[upstream] SIGNAL_UP received\n");
      upstreamSignalUp.store(true);
    }
    return true;
  } else if (type == PT_FRAME) {
    if (metaBytes < sizeof(FrameMeta)) { fprintf(stderr, "[upstream] FRAME meta too small\n"); return false; }
    FrameMeta fm{}; memcpy(&fm, meta.data(), sizeof(fm));
    uint32_t w = ntohl(fm.width);
    uint32_t h = ntohl(fm.height);
    uint8_t codec = fm.codec;
    uint8_t is_bgr = fm.is_bgr;

    std::vector<uint8_t> payload;
    payload.resize(payloadSize);
    if (payloadSize && !read_exact(upstream_fd, payload.data(), payloadSize)) return false;

    if (codec == CODEC_RAW) {
      if ((int)w != state.width || (int)h != state.height) {
        state.width = (int)w; state.height = (int)h;
        current_frame_rgb.resize((size_t)w * (size_t)h * 3ull);
      }
      const size_t need = (size_t)w * (size_t)h * 3ull;
      if (payload.size() >= need) {
        memcpy(current_frame_rgb.data(), payload.data(), need);
      } else {
        memset(current_frame_rgb.data(), 0, need);
        memcpy(current_frame_rgb.data(), payload.data(), payload.size());
      }
      // Client-side BGR->RGB swap
      if (is_bgr) {
        size_t pixels = (size_t)w * (size_t)h;
        swap_rb_inplace_mt(current_frame_rgb.data(), pixels, std::max(1, g_encode_threads));
      }
    } else {
      // Future: MJPEG inbound -> decode to RGB24 here
    }
    return true;
  } else {
    if (payloadSize) {
      std::vector<uint8_t> sink(payloadSize);
      if (!read_exact(upstream_fd, sink.data(), payloadSize)) return false;
    }
    return true;
  }
}

// ---------------- CLI ----------------
static bool g_flag_no_vignette = false;
static float opt_vig_center_x = 0.5f;
static float opt_vig_center_y = 0.5f;
static float opt_vig_inner = 0.073f;
static float opt_vig_outer = 0.098f;
static float opt_vig_strength = 1.85f;
static float opt_vig_axis_x = 0.1957f;
static float opt_vig_axis_y = 0.0471f;
static float opt_vig_angle_deg = 0.0f;

void parse_cli_or_die(int argc, const char** argv) {
  int opt_port = 0;     // local re-broadcast port
  int opt_mjpeg = 0;
  int opt_raw = 0;
  int opt_q = g_jpeg_quality;
  int opt_lazy = 0;
  int opt_no_lazy = 0;
  int opt_lazy_thresh = 1;
  const char* opt_upstream = nullptr; // host:port

  struct poptOption optionsTable[] = {
    { "listen",  'p', POPT_ARG_INT, &opt_port, 0, "Local TCP listen port for downstream clients", "PORT" },
    { "mjpeg",   0,   POPT_ARG_NONE,&opt_mjpeg,0, "Re-broadcast MJPEG", nullptr },
    { "raw",     0,   POPT_ARG_NONE,&opt_raw,  0, "Re-broadcast RAW RGB24 (default)", nullptr },
    { "jpeg-quality", 'q', POPT_ARG_INT, &opt_q, 0, "JPEG quality (1-100)", "Q" },
    { "lazy",    0,   POPT_ARG_NONE,&opt_lazy, 0, "Enable lazy-send downstream", nullptr },
    { "no-lazy", 0,   POPT_ARG_NONE,&opt_no_lazy,0,"Disable lazy-send", nullptr },
    { "lazy-threshold", 0, POPT_ARG_INT, &opt_lazy_thresh, 0, "Consecutive identical frames before CRT path", "N" },
    { "upstream", 'u', POPT_ARG_STRING, &opt_upstream, 0, "Upstream host:port (capture server)", "HOST:PORT" },
    { "encode-threads", 0, POPT_ARG_INT, &g_encode_threads, 0, "MJPEG encoder threads (0=auto)", "N" },

    // CUDA vignette controls
    { "no-vignette", 0, POPT_ARG_NONE, &g_flag_no_vignette, 0, "Disable CUDA vignette", nullptr },
    { "vig-center-x", 0, POPT_ARG_FLOAT, &opt_vig_center_x, 0, "Vignette center X (0..1)", "F" },
    { "vig-center-y", 0, POPT_ARG_FLOAT, &opt_vig_center_y, 0, "Vignette center Y (0..1)", "F" },
    { "vig-inner",    0, POPT_ARG_FLOAT, &opt_vig_inner,    0, "Inner radius (0..1)", "F" },
    { "vig-outer",    0, POPT_ARG_FLOAT, &opt_vig_outer,    0, "Outer radius (0..1)", "F" },
    { "vig-strength", 0, POPT_ARG_FLOAT, &opt_vig_strength, 0, "Strength (0..2)", "F" },
    { "vig-axis-x",   0, POPT_ARG_FLOAT, &opt_vig_axis_x,   0, "Axis scale X", "F" },
    { "vig-axis-y",   0, POPT_ARG_FLOAT, &opt_vig_axis_y,   0, "Axis scale Y", "F" },
    { "vig-angle-deg",0, POPT_ARG_FLOAT, &opt_vig_angle_deg,0, "Angle in degrees", "F" },

    { "help",    'h', POPT_ARG_NONE, nullptr, 'h', "Help", nullptr },
    { nullptr, 0, 0, nullptr, 0, nullptr, nullptr }
  };
  poptContext pc = poptGetContext(argv[0], argc, argv, optionsTable, 0);
  int rc;
  while ((rc = poptGetNextOpt(pc)) >= 0) {}
  if (rc < -1) {
    fprintf(stderr, "[main] Options error: %s: %s\n", poptBadOption(pc, POPT_BADOPTION_NOALIAS), poptStrerror(rc));
    poptPrintUsage(pc, stderr, 0);
    exit(1);
  }
  poptFreeContext(pc);

  if (opt_port <= 0 || opt_port > 65535) { fprintf(stderr, "[main] --listen PORT (1..65535) required\n"); exit(1); }
  listenPort = opt_port;

  if (opt_q < 1) opt_q = 1; if (opt_q > 100) opt_q = 100;
  g_jpeg_quality = opt_q;
  if (opt_no_lazy) g_lazy_send = false; else if (opt_lazy) g_lazy_send = true;
  if (opt_lazy_thresh < 1) opt_lazy_thresh = 1;
  g_lazy_threshold = opt_lazy_thresh;
  g_streamCodec = (opt_mjpeg && !opt_raw) ? CODEC_MJPEG : CODEC_RAW;

  if (opt_upstream && *opt_upstream) {
    std::string s(opt_upstream);
    auto pos = s.find(':');
    if (pos == std::string::npos) { fprintf(stderr, "[main] --upstream must be HOST:PORT\n"); exit(1); }
    upstreamHost = s.substr(0, pos);
    upstreamPort = atoi(s.substr(pos + 1).c_str());
  }

  g_enable_vignette = (g_flag_no_vignette == 0);
  g_vig.center_x = opt_vig_center_x;
  g_vig.center_y = opt_vig_center_y;
  g_vig.inner_radius = opt_vig_inner;
  g_vig.outer_radius = opt_vig_outer;
  g_vig.strength = opt_vig_strength;
  g_vig.axis_scale_x = opt_vig_axis_x;
  g_vig.axis_scale_y = opt_vig_axis_y;
  g_vig.angle_rad = (float)(opt_vig_angle_deg * M_PI / 180.0);

  fprintf(stderr, "[main] Upstream=%s:%d, local listen=%d, rebroadcast=%s%s, lazy=%d thr=%d, q=%d, vignette=%s\n",
          upstreamHost.c_str(), upstreamPort, listenPort,
          (g_streamCodec == CODEC_RAW ? "RAW" : "MJPEG"),
          (g_streamCodec == CODEC_MJPEG ? (std::string(" (q=") + std::to_string(g_jpeg_quality) + ")").c_str() : ""),
          g_lazy_send ? 1 : 0, g_lazy_threshold, g_jpeg_quality,
          g_enable_vignette ? "on" : "off");
}

// ----------------- main -----------------
int main(int argc, char** argv) {
  parse_cli_or_die(argc, (const char**)argv);

  int upstream_fd = connect_to_server(upstreamHost, upstreamPort);
  if (upstream_fd < 0) return 1;

  listen_fd = setup_server_socket(listenPort);
  if (listen_fd < 0) {
    close(upstream_fd);
    return 1;
  }

  start_encoder_pool_if_needed();

  DevState state{};
  CRTContext crt; crtctx_init_defaults(crt);

  std::vector<uint8_t> frame_rgb;     // current RGB24 frame from upstream (after BGR swap if needed)
  std::vector<uint8_t> prev_raw_frame;
  bool have_prev = false;
  int consecutive_same = 0;
  uint64_t last_sent_jpeg_id = 0;

  while (programRunning.load()) {
    if (!recv_and_handle_packet(upstream_fd, state, frame_rgb)) {
      fprintf(stderr, "[upstream] Disconnected. Exiting.\n");
      break;
    }

    // Start/stop no-signal worker as signaled by upstream
    if (!upstreamSignalUp.load()) {
      start_nosignal_if_needed(state);
    } else {
      stop_nosignal_if_running();
    }

    // Accept local clients (always)
    accept_new_clients(state.width, state.height, 3);

    // Process new frames when present and signal is up
    if (state.width > 0 && state.height > 0 && !frame_rgb.empty() && upstreamSignalUp.load()) {
      const size_t frame_bytes_cnt = (size_t)state.width * (size_t)state.height * 3ull;

      // Apply CUDA vignette first on the fresh frame
      //apply_vignette_cuda_locked(frame_rgb.data(), state.width, state.height);

      bool frame_changed = true;
      bool is_same = false;
      if (g_lazy_send && have_prev && prev_raw_frame.size() == frame_bytes_cnt) {
        if (frames_likely_equal_fast(prev_raw_frame.data(), frame_rgb.data(), state.width, state.height)) {
          is_same = (std::memcmp(prev_raw_frame.data(), frame_rgb.data(), frame_bytes_cnt) == 0);
        } else {
          is_same = false;
        }
      }
      if (!have_prev) {
        prev_raw_frame.resize(frame_bytes_cnt);
        memcpy(prev_raw_frame.data(), frame_rgb.data(), frame_bytes_cnt);
        have_prev = true;
        frame_changed = true;
        consecutive_same = 0;
      } else {
        if (is_same) {
          frame_changed = false;
          consecutive_same++;
        } else {
          frame_changed = true;
          consecutive_same = 0;
          if (prev_raw_frame.size() != frame_bytes_cnt) prev_raw_frame.resize(frame_bytes_cnt);
          memcpy(prev_raw_frame.data(), frame_rgb.data(), frame_bytes_cnt);
        }
      }

      if (g_streamCodec == CODEC_MJPEG) {
        if (g_lazy_send && !frame_changed && consecutive_same >= g_lazy_threshold) {
          ensure_crt_filter(crt, state.width, state.height);
          std::vector<uint8_t> filtered_rgb;
          filtered_rgb.reserve(frame_rgb.size());
          crt.filter->apply(frame_rgb.data(), filtered_rgb);
          // CUDA vignette on CRT result too
          apply_vignette_cuda_locked(filtered_rgb.data(), state.width, state.height);
          enqueue_encode_job(filtered_rgb.data(), filtered_rgb.size(), state.width, state.height);
        } else if (frame_changed) {
          enqueue_encode_job(frame_rgb.data(), frame_rgb.size(), state.width, state.height);
        }
        auto sp = std::atomic_load_explicit(&g_latest_jpeg, std::memory_order_acquire);
        uint64_t cur_id = g_latest_jpeg_id.load(std::memory_order_acquire);
        if (sp && !sp->empty() && cur_id != last_sent_jpeg_id) {
          broadcast_frame_local(sp->data(), sp->size());
          last_sent_jpeg_id = cur_id;
        }
      } else {
        if (g_lazy_send && !frame_changed && consecutive_same >= g_lazy_threshold) {
          ensure_crt_filter(crt, state.width, state.height);
          std::vector<uint8_t> filtered_rgb;
          filtered_rgb.reserve(frame_rgb.size());
          crt.filter->apply(frame_rgb.data(), filtered_rgb);
          // CUDA vignette on CRT result
          apply_vignette_cuda_locked(filtered_rgb.data(), state.width, state.height);
          broadcast_frame_local(filtered_rgb.data(), filtered_rgb.size());
        } else if (frame_changed) {
          broadcast_frame_local(frame_rgb.data(), frame_rgb.size());
        }
      }
    }
  }

  programRunning.store(false);
  stop_encoder_pool();
  for (int fd : client_fds) close(fd);
  client_fds.clear();
  if (listen_fd >= 0) close(listen_fd);
  if (upstream_fd >= 0) close(upstream_fd);
  return 0;
}
