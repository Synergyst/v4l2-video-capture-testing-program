// crt_filter.cu
#include "crt_filter_cuda.h"
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <random>

// ---------------- CUDA helpers ----------------
static inline bool check_cuda(const char* msg, cudaError_t err) {
  if (err != cudaSuccess) {
    fprintf(stderr, "[cuda][CRT] %s: %s\n", msg, cudaGetErrorString(err));
    return false;
  }
  return true;
}

// Persistent device buffers reused across frames.
// Note: this is module-scoped (shared if you ever make multiple CRTFilter instances).
static uint8_t* g_d_src = nullptr;
static uint8_t* g_d_dst = nullptr;
static size_t   g_d_capacity = 0; // bytes
static int      g_last_w = 0;
static int      g_last_h = 0;

// ---------------- Per-frame constants passed to kernel ----------------
struct CRTKernelParams {
  // image info
  int   width;
  int   height;
  // per-frame constants
  float t;           // seconds
  float flicker;     // overall brightness modulator
  int   v_shift_pix; // vertical shift in lines (rounded)
  int   vphase;      // 0/1 (scanline alternation phase)
  int   frame_idx;   // frame counter (for hashing)
  // fixed params from CRTParams
  float scanline_strength;  // 0..1
  float mask_strength;      // 0..1 (already used to build tri gains)
  float grain_strength;     // amplitude multiplier
  float h_warp_amp;         // pixels
  float h_warp_freq_y;      // per-line
  float h_warp_freq_t;      // per-second
  float wobble_line_noise;  // pixels
  float phaseH;             // random phase for horizontal wobble
  // triad shadow-mask multipliers (only the mask part; base_gain is per-line)
  float triR[3];
  float triG[3];
  float triB[3];
};

// Device-side helpers
__device__ inline uint8_t clamp_u8_dev(int v) {
  return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}
// hash like your host-side version: [-1,1]
__device__ inline float hash31_dev(int x, int y, int t) {
  // uint32_t h = (uint32_t)(x * 374761393u + y * 668265263u) ^ (uint32_t)(t * 362437u);
  unsigned int h = (unsigned int)(x * 374761393u + y * 668265263u) ^ (unsigned int)(t * 362437u);
  h = (h ^ (h >> 13)) * 1274126177u;
  h ^= (h >> 16);
  // map to [-1, 1]
  float f = (float)(h & 0x7FFFFFu) / (float)0x3FFFFFu; // ~[0,2)
  return f * 2.0f - 1.0f;
}

__global__ void crt_kernel(const uint8_t* __restrict__ src,
                           uint8_t* __restrict__ dst,
                           CRTKernelParams kp)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int total = kp.width * kp.height;
  if (idx >= total) return;

  const int x = idx % kp.width;
  const int y = idx / kp.width;

  // Vertical source line with jitter/bounce (already rounded on host)
  int y_src = y - kp.v_shift_pix;
  if (y_src < 0)           y_src = 0;
  if (y_src >= kp.height)  y_src = kp.height - 1;

  // Horizontal wobble (constant across line) + per-line noise
  // wobble_base = h_warp_amp * sin(2*pi*(h_warp_freq_y*y + h_warp_freq_t*t) + phaseH)
  const float TWO_PI = 6.2831853071795864769f;
  float wobble_base = kp.h_warp_amp *
                      __sinf(TWO_PI * (kp.h_warp_freq_y * (float)y + kp.h_warp_freq_t * kp.t) + kp.phaseH);
  float wobble_noise = kp.wobble_line_noise * hash31_dev(0, y, kp.frame_idx);
  int wob_i = (int)nearbyintf(wobble_base + wobble_noise);

  // Scanline darkening (alternating lines)
  float scan_gain = (((y + kp.vphase) & 1) ? (1.0f - kp.scanline_strength) : 1.0f);
  float base_gain = kp.flicker * scan_gain;

  // Shadow-mask triad selection
  int tri = x % 3;
  float mr = base_gain * kp.triR[tri];
  float mg = base_gain * kp.triG[tri];
  float mb = base_gain * kp.triB[tri];

  // Source pixel with horizontal wobble, clamped
  int xs = x + wob_i;
  if (xs < 0)         xs = 0;
  if (xs >= kp.width) xs = kp.width - 1;

  int src_ofs = (y_src * kp.width + xs) * 3;
  int dst_ofs = idx * 3;

  // Grain per pixel
  float grain = kp.grain_strength * hash31_dev(x, y, kp.frame_idx);

  int r = (int)lrintf(src[src_ofs + 0] * mr + 127.5f * grain);
  int g = (int)lrintf(src[src_ofs + 1] * mg + 127.5f * grain);
  int b = (int)lrintf(src[src_ofs + 2] * mb + 127.5f * grain);

  dst[dst_ofs + 0] = clamp_u8_dev(r);
  dst[dst_ofs + 1] = clamp_u8_dev(g);
  dst[dst_ofs + 2] = clamp_u8_dev(b);
}

// ---------------- CRTFilter (host) ----------------
CRTFilter::CRTFilter(int width, int height, const CRTParams& p, size_t threads, int fps)
  : w_(width), h_(height), p_(p), pool_(threads), fps_((fps > 0) ? fps : 30) {
  std::random_device rd;
  rng_.seed(rd());
  const float TWO_PI = 6.2831853071795864769f;
  phase60_ = frand_(rng_, 0.0f, TWO_PI);
  phaseH_  = frand_(rng_, 0.0f, TWO_PI);
  phaseV_  = frand_(rng_, 0.0f, TWO_PI);
  // Create CUDA context in this thread
  (void)check_cuda("cudaFree(0) init", cudaFree(0));
}

CRTFilter::FrameConsts CRTFilter::prepare_frame_(int frame_idx, int fps) {
  const float TWO_PI = 6.2831853071795864769f;
  const float t = (fps > 0) ? (float)frame_idx / (float)fps : (float)frame_idx * (1.0f / 30.0f);

  // Per-frame flicker
  float flicker = 1.0f
    + p_.flicker_60hz * std::sin(TWO_PI * 60.0f * t + phase60_)
    + frand_(rng_, -p_.flicker_noise, p_.flicker_noise);

  // Occasional vertical sync "bounce"
  if (v_bounce_frames_ == 0 && frand_(rng_, 0.0f, 1.0f) < 0.015f) {
    v_bounce_frames_ = 50 + (int)frand_(rng_, 0.0f, 40.0f);
    v_bounce_amp_    = frand_(rng_, 1.0f, 3.0f);
  }

  float v_bounce = 0.0f;
  if (v_bounce_frames_ > 0) {
    float decay = (float)v_bounce_frames_ / 90.0f;
    v_bounce = v_bounce_amp_ * std::sin(TWO_PI * 2.0f * t + phaseV_) * decay;
    v_bounce_frames_--;
  }

  // Base vertical jitter
  float v_jitter = p_.v_shake_amp * std::sin(TWO_PI * 0.9f * t + phaseV_);
  float v_shift = v_bounce + v_jitter; // in lines

  FrameConsts fc;
  fc.t = t;
  fc.flicker = flicker;
  fc.v_shift = v_shift;
  fc.vphase = frame_idx & 1;
  return fc;
}

// Auto-advancing apply
void CRTFilter::apply(const uint8_t* src, std::vector<uint8_t>& dst) {
  this->apply(src, static_cast<int>(frame_idx_), fps_, dst);
  ++frame_idx_;
}

void CRTFilter::apply(const uint8_t* src, int frame_idx, int fps, std::vector<uint8_t>& dst) {
  if (!src || w_ <= 0 || h_ <= 0) {
    dst.clear();
    return;
  }
  dst.resize((size_t)w_ * (size_t)h_ * 3u);

  // Per-frame constants (host)
  const auto fc = prepare_frame_(frame_idx, fps);

  // Prepare kernel params
  CRTKernelParams kp{};
  kp.width  = w_;
  kp.height = h_;
  kp.t = fc.t;
  kp.flicker = fc.flicker;
  kp.v_shift_pix = (int)lrintf(fc.v_shift);
  kp.vphase = fc.vphase;
  kp.frame_idx = frame_idx;

  kp.scanline_strength = std::clamp(p_.scanline_strength, 0.0f, 1.0f);
  kp.mask_strength     = std::clamp(p_.mask_strength, 0.0f, 1.0f);
  kp.grain_strength    = std::max(0.0f, p_.grain_strength);
  kp.h_warp_amp        = p_.h_warp_amp;
  kp.h_warp_freq_y     = p_.h_warp_freq_y;
  kp.h_warp_freq_t     = p_.h_warp_freq_t;
  kp.wobble_line_noise = p_.wobble_line_noise;
  kp.phaseH            = phaseH_;

  // Triad mask gains (mask only; base_gain multiplies in kernel)
  const float ms = kp.mask_strength;
  const float triad[3][3] = {
      {1.00f, 0.80f, 0.75f}, // x%3 == 0
      {0.75f, 1.00f, 0.80f}, // x%3 == 1
      {0.80f, 0.75f, 1.00f}, // x%3 == 2
  };
  for (int i = 0; i < 3; ++i) {
    kp.triR[i] = (1.0f - ms) + ms * triad[i][0];
    kp.triG[i] = (1.0f - ms) + ms * triad[i][1];
    kp.triB[i] = (1.0f - ms) + ms * triad[i][2];
  }

  // Ensure device buffers are allocated
  const size_t bytes = (size_t)w_ * (size_t)h_ * 3ull;
  if (g_d_capacity < bytes || w_ != g_last_w || h_ != g_last_h) {
    if (g_d_src) cudaFree(g_d_src);
    if (g_d_dst) cudaFree(g_d_dst);
    g_d_src = g_d_dst = nullptr;
    g_d_capacity = 0;
    if (!check_cuda("cudaMalloc d_src", cudaMalloc((void**)&g_d_src, bytes))) return;
    if (!check_cuda("cudaMalloc d_dst", cudaMalloc((void**)&g_d_dst, bytes))) { cudaFree(g_d_src); g_d_src = nullptr; return; }
    g_d_capacity = bytes;
    g_last_w = w_;
    g_last_h = h_;
  }

  // H2D
  if (!check_cuda("cudaMemcpy H2D(src)", cudaMemcpy(g_d_src, src, bytes, cudaMemcpyHostToDevice))) return;

  // Launch kernel
  int threads = 256;
  int blocks = (int)((bytes / 3 + threads - 1) / threads); // one thread per pixel
  crt_kernel<<<blocks, threads>>>(g_d_src, g_d_dst, kp);
  if (!check_cuda("kernel launch", cudaGetLastError())) return;
  if (!check_cuda("cudaDeviceSynchronize", cudaDeviceSynchronize())) return;

  // D2H
  if (!check_cuda("cudaMemcpy D2H(dst)", cudaMemcpy(dst.data(), g_d_dst, bytes, cudaMemcpyDeviceToHost))) return;
}
