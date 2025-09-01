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

// Signed distance to an axis-aligned rounded rectangle (Inigo Quilez)
// p: point in pixels relative to box center
// b: half extents (half width/height) in pixels
// r: corner radius in pixels
static inline float sdf_rounded_box(float px, float py, float bx, float by, float r) {
  const float ax = std::fabs(px);
  const float ay = std::fabs(py);
  const float qx = ax - bx + r;
  const float qy = ay - by + r;
  const float mx = std::max(qx, 0.0f);
  const float my = std::max(qy, 0.0f);
  const float outside = std::hypot(mx, my);
  const float inside  = std::min(std::max(qx, qy), 0.0f);
  return outside + inside - r; // <0 inside shape, 0 at edge, >0 outside
}

// vignette_filter.cpp (replace the body of VignetteFilter::applyRoundedBox pointer overload)
void VignetteFilter::applyRoundedBox(const uint8_t* src_rgb24,
                                     uint8_t* dst_rgb24,
                                     const VignetteBoxParams& box,
                                     std::size_t strideBytes,
                                     uint8_t* optionalMaskOut) const
{
  if (!src_rgb24 || !dst_rgb24 || w_ <= 0 || h_ <= 0) return;

  const std::size_t stride = (strideBytes == 0) ? static_cast<std::size_t>(w_) * 3u : strideBytes;

  // Normalize and clamp box
  float x0n = std::clamp(box.x0, 0.0f, 1.0f);
  float y0n = std::clamp(box.y0, 0.0f, 1.0f);
  float x1n = std::clamp(box.x1, 0.0f, 1.0f);
  float y1n = std::clamp(box.y1, 0.0f, 1.0f);
  if (x1n < x0n) std::swap(x0n, x1n);
  if (y1n < y0n) std::swap(y0n, y1n);

  const float x0 = x0n * w_;
  const float y0 = y0n * h_;
  const float x1 = x1n * w_;
  const float y1 = y1n * h_;
  const float bw = std::max(1.0f, x1 - x0);
  const float bh = std::max(1.0f, y1 - y0);

  const float cx = 0.5f * (x0 + x1);
  const float cy = 0.5f * (y0 + y1);
  const float bx = 0.5f * bw;
  const float by = 0.5f * bh;

  const float minBoxDim = std::max(1.0f, std::min(bw, bh));
  const float radius_px = std::clamp(box.corner_radius_norm, 0.0f, 0.5f) * minBoxDim;

  // Feather: <=1 => normalized to minBoxDim; >1 => pixels
  const float feather_px = (box.feather <= 1.0f)
                             ? std::max(0.0f, box.feather) * minBoxDim
                             : std::max(0.0f, box.feather);
  const float invFeather = (feather_px > 0.0f) ? (1.0f / feather_px) : 0.0f;

  // Gamma/color (rounded-box params)
  const bool doGamma = box.gamma_correct && box.gamma > 0.0f;
  auto toLinear = [&](uint8_t u8) -> float {
    float v = u8 * (1.0f / 255.0f);
    return doGamma ? std::pow(v, box.gamma) : v;
  };
  auto toDisplay = [&](float lin) -> uint8_t {
    float v = doGamma ? std::pow(std::max(0.0f, lin), 1.0f / box.gamma) : std::max(0.0f, lin);
    if (box.clamp_output) v = std::clamp(v, 0.0f, 1.0f);
    // Faster than lround in hot path:
    return static_cast<uint8_t>(v * 255.0f + 0.5f);
  };
  float color_lin[3] = { box.color[0], box.color[1], box.color[2] };
  if (doGamma) {
    color_lin[0] = std::pow(std::clamp(color_lin[0], 0.0f, 1.0f), box.gamma);
    color_lin[1] = std::pow(std::clamp(color_lin[1], 0.0f, 1.0f), box.gamma);
    color_lin[2] = std::pow(std::clamp(color_lin[2], 0.0f, 1.0f), box.gamma);
  } else {
    color_lin[0] = std::clamp(color_lin[0], 0.0f, 1.0f);
    color_lin[1] = std::clamp(color_lin[1], 0.0f, 1.0f);
    color_lin[2] = std::clamp(color_lin[2], 0.0f, 1.0f);
  }

  // Row worker
  auto work_rows = [&](int y0r, int y1r) {
    for (int y = y0r; y < y1r; ++y) {
      const uint8_t* srow = src_rgb24 + static_cast<std::size_t>(y) * stride;
      uint8_t* drow = dst_rgb24 + static_cast<std::size_t>(y) * stride;
      uint8_t* mrow = optionalMaskOut ? (optionalMaskOut + static_cast<std::size_t>(y) * static_cast<std::size_t>(w_)) : nullptr;

      // Precompute y-relative coords
      const float py = (static_cast<float>(y) + 0.5f) - cy;
      const float ay = std::fabs(py);
      // Quick outside-rect saturation: far beyond box + feather => m=1 for invert=false
      const bool fully_outside_y = (ay >= by + feather_px);

      for (int x = 0; x < w_; ++x) {
        const uint8_t* sp = srow + static_cast<std::size_t>(x) * 3u;
        uint8_t* dp = drow + static_cast<std::size_t>(x) * 3u;

        // If we are fully outside on Y and also far outside on X, skip SDF in favor of saturated mask.
        float m;
        if (!box.invert && fully_outside_y) {
          const float px = (static_cast<float>(x) + 0.5f) - cx;
          const float ax = std::fabs(px);
          if (ax >= bx + feather_px) {
            m = box.strength; // full effect
          } else {
            // Need exact m near the X border: compute SDF once
            const float qx = std::max(ax - bx + radius_px, 0.0f);
            const float qy = std::max(ay - by + radius_px, 0.0f);
            const float outside = std::sqrt(qx*qx + qy*qy);
            const float inside  = std::min(std::max(ax - bx + radius_px, ay - by + radius_px), 0.0f);
            float d = outside + inside - radius_px; // signed distance
            if (feather_px <= 0.0f) {
              m = (d >= 0.0f) ? 1.0f : 0.0f;
            } else {
              m = smoothstep01((d + feather_px) * invFeather);
            }
            m = std::clamp(m * box.strength, 0.0f, 1.0f);
          }
        } else {
          // General path
          const float px = (static_cast<float>(x) + 0.5f) - cx;
          const float ax = std::fabs(px);

          // Fast SDF to rounded rect (inline, avoiding std::hypot)
          const float qx = std::max(ax - bx + radius_px, 0.0f);
          const float qy = std::max(ay - by + radius_px, 0.0f);
          const float outside = std::sqrt(qx*qx + qy*qy);
          const float inside  = std::min(std::max(ax - bx + radius_px, ay - by + radius_px), 0.0f);
          const float d = outside + inside - radius_px;

          if (!box.invert) {
            if (feather_px <= 0.0f) {
              m = (d >= 0.0f) ? 1.0f : 0.0f;
            } else {
              m = smoothstep01((d + feather_px) * invFeather);
            }
          } else {
            if (feather_px <= 0.0f) {
              m = (d > 0.0f) ? 1.0f : 0.0f;
            } else {
              m = smoothstep01(d * invFeather);
            }
          }
          m = std::clamp(m * box.strength, 0.0f, 1.0f);
        }

        if (mrow) mrow[x] = static_cast<uint8_t>(m * 255.0f + 0.5f);

        // Blend (gamma-aware)
        float inR = toLinear(sp[0]);
        float inG = toLinear(sp[1]);
        float inB = toLinear(sp[2]);

        float outR, outG, outB;
        if (box.mode == VignetteBlendMode::Multiply) {
          const float multR = (1.0f - m) + m * color_lin[0];
          const float multG = (1.0f - m) + m * color_lin[1];
          const float multB = (1.0f - m) + m * color_lin[2];
          outR = inR * multR;
          outG = inG * multG;
          outB = inB * multB;
        } else {
          outR = inR * (1.0f - m) + color_lin[0] * m;
          outG = inG * (1.0f - m) + color_lin[1] * m;
          outB = inB * (1.0f - m) + color_lin[2] * m;
        }

        dp[0] = toDisplay(outR);
        dp[1] = toDisplay(outG);
        dp[2] = toDisplay(outB);
      }
    }
  };

  if (threads_ <= 1 || h_ < 64) {
    work_rows(0, h_);
  } else {
    const int T = static_cast<int>(threads_);
    std::vector<std::thread> th;
    th.reserve(T);
    int rows_per = (h_ + T - 1) / T;
    for (int ti = 0; ti < T; ++ti) {
      int y0r = ti * rows_per;
      if (y0r >= h_) break;
      int y1r = std::min(h_, y0r + rows_per);
      th.emplace_back(work_rows, y0r, y1r);
    }
    for (auto& t : th) t.join();
  }
}

// vignette_filter.cpp (add the in-place rounded-box wrapper)
void VignetteFilter::applyRoundedBoxInPlace(uint8_t* buf_rgb24,
                                            const VignetteBoxParams& box,
                                            std::size_t strideBytes,
                                            uint8_t* optionalMaskOut) const {
  applyRoundedBox(buf_rgb24, buf_rgb24, box, strideBytes, optionalMaskOut);
}

void VignetteFilter::applyRoundedBox(const uint8_t* src_rgb24,
                                     std::vector<uint8_t>& dst_rgb24,
                                     const VignetteBoxParams& box,
                                     std::vector<uint8_t>* optionalMaskOut) const
{
  if (!src_rgb24 || w_ <= 0 || h_ <= 0) {
    dst_rgb24.clear();
    if (optionalMaskOut) optionalMaskOut->clear();
    return;
  }
  const std::size_t stride = static_cast<std::size_t>(w_) * 3u;
  dst_rgb24.resize(static_cast<std::size_t>(h_) * stride);

  uint8_t* maskPtr = nullptr;
  if (optionalMaskOut) {
    optionalMaskOut->assign(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0);
    maskPtr = optionalMaskOut->data();
  }

  applyRoundedBox(src_rgb24, dst_rgb24.data(), box, /*strideBytes=*/0, maskPtr);
}
