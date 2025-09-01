#include "vignette_filter.h"
#include <algorithm>
#include <cmath>
#include <thread>

static inline float smoothstep01(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

VignetteFilter::VignetteFilter(int width, int height, const VignetteParams& params, std::size_t threads)
  : w_(width),
    h_(height),
    threads_(threads ? threads
                     : std::max(1u, std::thread::hardware_concurrency()
                                    ? std::thread::hardware_concurrency()
                                    : 1u)),
    params_(params) {
  recomputeCached_();
}

void VignetteFilter::resize(int width, int height) {
  if (width == w_ && height == h_) return;
  w_ = width; h_ = height;
  recomputeCached_();
}

void VignetteFilter::setParams(const VignetteParams& params) {
  params_ = params;
  recomputeCached_();
}

void VignetteFilter::recomputeCached_() {
  constexpr float kPi = 3.14159265358979323846f;
  const float angle_rad = params_.angle_deg * (kPi / 180.0f);
  ca_ = std::cos(angle_rad);
  sa_ = std::sin(angle_rad);

  cx_ = params_.center_x * static_cast<float>(w_);
  cy_ = params_.center_y * static_cast<float>(h_);

  const float min_dim = static_cast<float>(std::min(w_, h_));
  hx_ = 0.5f * min_dim * std::max(1e-6f, params_.axis_scale_x);
  hy_ = 0.5f * min_dim * std::max(1e-6f, params_.axis_scale_y);

  doGamma_ = params_.gamma_correct && params_.gamma > 0.0f;

  if (doGamma_) {
    color_lin_[0] = std::pow(std::clamp(params_.color[0], 0.0f, 1.0f), params_.gamma);
    color_lin_[1] = std::pow(std::clamp(params_.color[1], 0.0f, 1.0f), params_.gamma);
    color_lin_[2] = std::pow(std::clamp(params_.color[2], 0.0f, 1.0f), params_.gamma);
  } else {
    color_lin_[0] = std::clamp(params_.color[0], 0.0f, 1.0f);
    color_lin_[1] = std::clamp(params_.color[1], 0.0f, 1.0f);
    color_lin_[2] = std::clamp(params_.color[2], 0.0f, 1.0f);
  }
}

inline float VignetteFilter::toLinear_(uint8_t u8) const {
  float v = u8 / 255.0f;
  return doGamma_ ? std::pow(v, params_.gamma) : v;
}

inline uint8_t VignetteFilter::toDisplay_(float lin) const {
  float v = doGamma_ ? std::pow(std::max(0.0f, lin), 1.0f / params_.gamma) : std::max(0.0f, lin);
  if (params_.clamp_output) v = std::clamp(v, 0.0f, 1.0f);
  return static_cast<uint8_t>(std::lround(v * 255.0f));
}

// Pointer-based core
void VignetteFilter::apply(const uint8_t* src_rgb24,
                           uint8_t* dst_rgb24,
                           std::size_t strideBytes,
                           uint8_t* optionalMaskOut) const {
  if (!src_rgb24 || !dst_rgb24 || w_ <= 0 || h_ <= 0) return;

  const std::size_t stride = (strideBytes == 0) ? static_cast<std::size_t>(w_) * 3u : strideBytes;

  // If src==dst, copy source to a temporary buffer to allow in-place effect.
  const uint8_t* src = src_rgb24;
  std::vector<uint8_t> tmp;
  if (src_rgb24 == dst_rgb24) {
    tmp.assign(static_cast<std::size_t>(h_) * stride, 0);
    std::copy_n(src_rgb24, tmp.size(), tmp.data());
    src = tmp.data();
  }

  const float r0 = std::max(0.0f, params_.inner_radius);
  const float r1 = std::max(r0 + 1e-6f, params_.outer_radius);

  auto process_rows = [&](int y0, int y1) {
    for (int y = y0; y < y1; ++y) {
      const uint8_t* srow = src + static_cast<std::size_t>(y) * stride;
      uint8_t* drow = dst_rgb24 + static_cast<std::size_t>(y) * stride;
      uint8_t* maskRow = optionalMaskOut ? (optionalMaskOut + static_cast<std::size_t>(y) * static_cast<std::size_t>(w_)) : nullptr;

      const float dy = (static_cast<float>(y) + 0.5f) - cy_;

      for (int x = 0; x < w_; ++x) {
        const float dx = (static_cast<float>(x) + 0.5f) - cx_;
        const float xr = dx * ca_ + dy * sa_;
        const float yr = -dx * sa_ + dy * ca_;

        const float xn = xr / hx_;
        const float yn = yr / hy_;
        const float r = std::sqrt(xn * xn + yn * yn);

        float t = (r - r0) / (r1 - r0);
        float m = smoothstep01(t);
        if (params_.invert) m = 1.0f - m;
        m = std::clamp(m * params_.strength, 0.0f, 1.0f);

        if (maskRow) {
          maskRow[x] = static_cast<uint8_t>(std::lround(m * 255.0f));
        }

        const uint8_t* sp = srow + static_cast<std::size_t>(x) * 3u;
        uint8_t* dp = drow + static_cast<std::size_t>(x) * 3u;

        float inR = toLinear_(sp[0]);
        float inG = toLinear_(sp[1]);
        float inB = toLinear_(sp[2]);

        float outR, outG, outB;
        if (params_.mode == VignetteBlendMode::Multiply) {
          const float multR = (1.0f - m) + m * color_lin_[0];
          const float multG = (1.0f - m) + m * color_lin_[1];
          const float multB = (1.0f - m) + m * color_lin_[2];
          outR = inR * multR;
          outG = inG * multG;
          outB = inB * multB;
        } else {
          outR = inR * (1.0f - m) + color_lin_[0] * m;
          outG = inG * (1.0f - m) + color_lin_[1] * m;
          outB = inB * (1.0f - m) + color_lin_[2] * m;
        }

        dp[0] = toDisplay_(outR);
        dp[1] = toDisplay_(outG);
        dp[2] = toDisplay_(outB);
      }
    }
  };

  if (threads_ <= 1 || h_ < 64) {
    process_rows(0, h_);
  } else {
    const int T = static_cast<int>(threads_);
    std::vector<std::thread> th;
    th.reserve(T);
    int rows_per = (h_ + T - 1) / T;
    for (int ti = 0; ti < T; ++ti) {
      int y0 = ti * rows_per; if (y0 >= h_) break;
      int y1 = std::min(h_, y0 + rows_per);
      th.emplace_back(process_rows, y0, y1);
    }
    for (auto& t : th) t.join();
  }
}

// Vector overload forwards to pointer-based core
void VignetteFilter::apply(const uint8_t* src_rgb24,
                           std::vector<uint8_t>& dst_rgb24,
                           std::vector<uint8_t>* optionalMaskOut) const {
  if (!src_rgb24 || w_ <= 0 || h_ <= 0) {
    dst_rgb24.clear();
    if (optionalMaskOut) optionalMaskOut->clear();
    return;
  }
  const std::size_t stride = static_cast<std::size_t>(w_) * 3u;
  dst_rgb24.resize(static_cast<std::size_t>(h_) * stride);

  uint8_t* maskPtr = nullptr;
  std::vector<uint8_t> dummyMask;
  if (optionalMaskOut) {
    optionalMaskOut->assign(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0);
    maskPtr = optionalMaskOut->data();
  }
  apply(src_rgb24, dst_rgb24.data(), /*strideBytes=*/0, maskPtr);
}

// In-place convenience
void VignetteFilter::applyInPlace(uint8_t* buf_rgb24,
                                  std::size_t strideBytes,
                                  uint8_t* optionalMaskOut) const {
  apply(buf_rgb24, buf_rgb24, strideBytes, optionalMaskOut);
}
