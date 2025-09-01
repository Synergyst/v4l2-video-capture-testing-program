#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <cmath>
#include <algorithm>
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  #include <arm_neon.h>
  #define GB_NEON 1
#else
  #define GB_NEON 0
#endif

// Build a normalized 1D Gaussian kernel (fixed-point Q8, sum==256). Returns radius.
static inline int gb_build_kernel_q8(float sigma, std::vector<uint16_t>& wq8) {
  if (sigma <= 0.f) { wq8.clear(); wq8.push_back(256); return 0; }
  // radius ~ ceil(3*sigma) gives ~99.7% coverage; cap to keep performance sane
  int radius = std::min(48, std::max(1, (int)std::ceil(3.f * sigma)));
  const int taps = 2 * radius + 1;
  wq8.resize((size_t)taps);
  const float inv2s2 = 1.f / (2.f * sigma * sigma);
  // build float weights
  std::vector<float> wf((size_t)taps);
  float sumf = 0.f;
  for (int i = -radius; i <= radius; ++i) {
    float w = std::exp(-(float)(i * i) * inv2s2);
    wf[(size_t)(i + radius)] = w;
    sumf += w;
  }
  // normalize to sum==256
  float scale = (sumf > 0.f) ? (256.f / sumf) : 0.f;
  int s = 0;
  for (int i = 0; i < taps; ++i) {
    int v = (int)std::lround(wf[(size_t)i] * scale);
    v = std::clamp(v, 0, 255); // we keep Q8 in 0..255 for each tap
    wq8[(size_t)i] = (uint16_t)v;
    s += v;
  }
  // fix rounding drift to exactly 256
  int diff = 256 - s;
  if (diff != 0) {
    int mid = radius;
    int adj = std::clamp((int)wq8[(size_t)mid] + diff, 0, 255);
    wq8[(size_t)mid] = (uint16_t)adj;
  }
  return radius;
}

// Scalar horizontal pass (clamp edges), Q8 kernel
static inline void gb_hpass_scalar(const uint8_t* src, uint8_t* dst,
                                   int w, int h, size_t stride,
                                   const uint16_t* k, int radius)
{
  const int taps = 2 * radius + 1;
  for (int y = 0; y < h; ++y) {
    const uint8_t* srow = src + (size_t)y * stride;
    uint8_t* drow = dst + (size_t)y * stride;
    for (int x = 0; x < w; ++x) {
      int accR = 0, accG = 0, accB = 0;
      for (int t = -radius; t <= radius; ++t) {
        int xi = x + t;
        if (xi < 0) xi = 0;
        if (xi >= w) xi = w - 1;
        const uint8_t* p = srow + (size_t)xi * 3u;
        int wq = k[(size_t)(t + radius)];
        accR += p[0] * wq;
        accG += p[1] * wq;
        accB += p[2] * wq;
      }
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = (uint8_t)((accR + 128) >> 8);
      q[1] = (uint8_t)((accG + 128) >> 8);
      q[2] = (uint8_t)((accB + 128) >> 8);
    }
  }
}

// Scalar vertical pass (clamp edges), Q8 kernel
static inline void gb_vpass_scalar(const uint8_t* src, uint8_t* dst,
                                   int w, int h, size_t stride,
                                   const uint16_t* k, int radius)
{
  const int taps = 2 * radius + 1;
  for (int y = 0; y < h; ++y) {
    uint8_t* drow = dst + (size_t)y * stride;
    for (int x = 0; x < w; ++x) {
      int accR = 0, accG = 0, accB = 0;
      for (int t = -radius; t <= radius; ++t) {
        int yi = y + t;
        if (yi < 0) yi = 0;
        if (yi >= h) yi = h - 1;
        const uint8_t* p = src + (size_t)yi * stride + (size_t)x * 3u;
        int wq = k[(size_t)(t + radius)];
        accR += p[0] * wq;
        accG += p[1] * wq;
        accB += p[2] * wq;
      }
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = (uint8_t)((accR + 128) >> 8);
      q[1] = (uint8_t)((accG + 128) >> 8);
      q[2] = (uint8_t)((accB + 128) >> 8);
    }
  }
}

#if GB_NEON
// Vectorized horizontal pass for interior region [radius .. w - radius - 16] step 16
static inline void gb_hpass_neon_row(const uint8_t* srow, uint8_t* drow,
                                     int w, size_t stride,
                                     const uint16_t* k, int radius)
{
  const int x_interior_start = radius;
  const int x_interior_end = w - radius - 16; // inclusive start; we loop while x <= end
  if (x_interior_end >= x_interior_start) {
    for (int x = x_interior_start; x <= x_interior_end; x += 16) {
      // accumulators, 16 lanes per channel (two 8-lane halves)
      uint16x8_t accR_lo = vdupq_n_u16(0), accR_hi = vdupq_n_u16(0);
      uint16x8_t accG_lo = vdupq_n_u16(0), accG_hi = vdupq_n_u16(0);
      uint16x8_t accB_lo = vdupq_n_u16(0), accB_hi = vdupq_n_u16(0);

      for (int t = -radius; t <= radius; ++t) {
        const uint8_t* p = srow + (size_t)(x + t) * 3u;
        uint8x16x3_t v = vld3q_u8(p); // deinterleave 16 RGB pixels
        uint16x8_t wv = vdupq_n_u16((uint16_t)k[(size_t)(t + radius)]);

        // R
        uint16x8_t r_lo = vmovl_u8(vget_low_u8(v.val[0]));
        uint16x8_t r_hi = vmovl_u8(vget_high_u8(v.val[0]));
        accR_lo = vmlaq_u16(accR_lo, r_lo, wv);
        accR_hi = vmlaq_u16(accR_hi, r_hi, wv);
        // G
        uint16x8_t g_lo = vmovl_u8(vget_low_u8(v.val[1]));
        uint16x8_t g_hi = vmovl_u8(vget_high_u8(v.val[1]));
        accG_lo = vmlaq_u16(accG_lo, g_lo, wv);
        accG_hi = vmlaq_u16(accG_hi, g_hi, wv);
        // B
        uint16x8_t b_lo = vmovl_u8(vget_low_u8(v.val[2]));
        uint16x8_t b_hi = vmovl_u8(vget_high_u8(v.val[2]));
        accB_lo = vmlaq_u16(accB_lo, b_lo, wv);
        accB_hi = vmlaq_u16(accB_hi, b_hi, wv);
      }

      // Normalize (>>8) and pack back to u8
      uint16x8_t r_lo_s = vshrq_n_u16(accR_lo, 8);
      uint16x8_t r_hi_s = vshrq_n_u16(accR_hi, 8);
      uint16x8_t g_lo_s = vshrq_n_u16(accG_lo, 8);
      uint16x8_t g_hi_s = vshrq_n_u16(accG_hi, 8);
      uint16x8_t b_lo_s = vshrq_n_u16(accB_lo, 8);
      uint16x8_t b_hi_s = vshrq_n_u16(accB_hi, 8);

      uint8x8_t r8_lo = vqmovn_u16(r_lo_s);
      uint8x8_t r8_hi = vqmovn_u16(r_hi_s);
      uint8x8_t g8_lo = vqmovn_u16(g_lo_s);
      uint8x8_t g8_hi = vqmovn_u16(g_hi_s);
      uint8x8_t b8_lo = vqmovn_u16(b_lo_s);
      uint8x8_t b8_hi = vqmovn_u16(b_hi_s);

      uint8x16_t r8 = vcombine_u8(r8_lo, r8_hi);
      uint8x16_t g8 = vcombine_u8(g8_lo, g8_hi);
      uint8x16_t b8 = vcombine_u8(b8_lo, b8_hi);

      uint8x16x3_t outv;
      outv.val[0] = r8;
      outv.val[1] = g8;
      outv.val[2] = b8;

      vst3q_u8(drow + (size_t)x * 3u, outv);
    }
  }

  // Left border
  if (radius > 0) {
    for (int x = 0; x < std::min(radius, w); ++x) {
      int accR = 0, accG = 0, accB = 0;
      for (int t = -radius; t <= radius; ++t) {
        int xi = x + t;
        if (xi < 0) xi = 0;
        if (xi >= w) xi = w - 1;
        const uint8_t* p = srow + (size_t)xi * 3u;
        int wq = k[(size_t)(t + radius)];
        accR += p[0] * wq; accG += p[1] * wq; accB += p[2] * wq;
      }
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = (uint8_t)((accR + 128) >> 8);
      q[1] = (uint8_t)((accG + 128) >> 8);
      q[2] = (uint8_t)((accB + 128) >> 8);
    }
  }

  // Right border and any tail
  int x_tail_start = std::max(radius, w - radius - 16 + 1);
  if (x_tail_start < radius) x_tail_start = radius;
  for (int x = x_tail_start; x < w; ++x) {
    int accR = 0, accG = 0, accB = 0;
    for (int t = -radius; t <= radius; ++t) {
      int xi = x + t;
      if (xi < 0) xi = 0;
      if (xi >= w) xi = w - 1;
      const uint8_t* p = srow + (size_t)xi * 3u;
      int wq = k[(size_t)(t + radius)];
      accR += p[0] * wq; accG += p[1] * wq; accB += p[2] * wq;
    }
    uint8_t* q = drow + (size_t)x * 3u;
    q[0] = (uint8_t)((accR + 128) >> 8);
    q[1] = (uint8_t)((accG + 128) >> 8);
    q[2] = (uint8_t)((accB + 128) >> 8);
  }
}

// Vectorized vertical pass for interior rows (y = radius..h-radius-1), processes x in blocks of 16
static inline void gb_vpass_neon_blocky(const uint8_t* src, uint8_t* dst,
                                        int w, int h, size_t stride,
                                        const uint16_t* k, int radius)
{
  const int y_interior_start = radius;
  const int y_interior_end = h - radius - 1;
  if (y_interior_end < y_interior_start) return;

  // Top border rows (scalar)
  if (radius > 0) {
    gb_vpass_scalar(src, dst, w, radius, stride, k, radius);
  }

  // Interior rows with NEON across x
  for (int y = y_interior_start; y <= y_interior_end; ++y) {
    uint8_t* drow = dst + (size_t)y * stride;

    int x = 0;
    // Vectorized blocks of 16
    for (; x + 16 <= w; x += 16) {
      uint16x8_t accR_lo = vdupq_n_u16(0), accR_hi = vdupq_n_u16(0);
      uint16x8_t accG_lo = vdupq_n_u16(0), accG_hi = vdupq_n_u16(0);
      uint16x8_t accB_lo = vdupq_n_u16(0), accB_hi = vdupq_n_u16(0);

      for (int t = -radius; t <= radius; ++t) {
        const uint8_t* srow = src + (size_t)(y + t) * stride + (size_t)x * 3u;
        uint8x16x3_t v = vld3q_u8(srow);
        uint16x8_t wv = vdupq_n_u16((uint16_t)k[(size_t)(t + radius)]);
        // R
        uint16x8_t r_lo = vmovl_u8(vget_low_u8(v.val[0]));
        uint16x8_t r_hi = vmovl_u8(vget_high_u8(v.val[0]));
        accR_lo = vmlaq_u16(accR_lo, r_lo, wv);
        accR_hi = vmlaq_u16(accR_hi, r_hi, wv);
        // G
        uint16x8_t g_lo = vmovl_u8(vget_low_u8(v.val[1]));
        uint16x8_t g_hi = vmovl_u8(vget_high_u8(v.val[1]));
        accG_lo = vmlaq_u16(accG_lo, g_lo, wv);
        accG_hi = vmlaq_u16(accG_hi, g_hi, wv);
        // B
        uint16x8_t b_lo = vmovl_u8(vget_low_u8(v.val[2]));
        uint16x8_t b_hi = vmovl_u8(vget_high_u8(v.val[2]));
        accB_lo = vmlaq_u16(accB_lo, b_lo, wv);
        accB_hi = vmlaq_u16(accB_hi, b_hi, wv);
      }

      uint16x8_t r_lo_s = vshrq_n_u16(accR_lo, 8);
      uint16x8_t r_hi_s = vshrq_n_u16(accR_hi, 8);
      uint16x8_t g_lo_s = vshrq_n_u16(accG_lo, 8);
      uint16x8_t g_hi_s = vshrq_n_u16(accG_hi, 8);
      uint16x8_t b_lo_s = vshrq_n_u16(accB_lo, 8);
      uint16x8_t b_hi_s = vshrq_n_u16(accB_hi, 8);

      uint8x8_t r8_lo = vqmovn_u16(r_lo_s);
      uint8x8_t r8_hi = vqmovn_u16(r_hi_s);
      uint8x8_t g8_lo = vqmovn_u16(g_lo_s);
      uint8x8_t g8_hi = vqmovn_u16(g_hi_s);
      uint8x8_t b8_lo = vqmovn_u16(b_lo_s);
      uint8x8_t b8_hi = vqmovn_u16(b_hi_s);

      uint8x16_t r8 = vcombine_u8(r8_lo, r8_hi);
      uint8x16_t g8 = vcombine_u8(g8_lo, g8_hi);
      uint8x16_t b8 = vcombine_u8(b8_lo, b8_hi);

      uint8x16x3_t outv;
      outv.val[0] = r8;
      outv.val[1] = g8;
      outv.val[2] = b8;

      vst3q_u8(drow + (size_t)x * 3u, outv);
    }

    // Tail columns (scalar)
    for (; x < w; ++x) {
      int accR = 0, accG = 0, accB = 0;
      for (int t = -radius; t <= radius; ++t) {
        int yi = y + t; // interior so no clamp needed here, but keep safe
        if (yi < 0) yi = 0;
        if (yi >= h) yi = h - 1;
        const uint8_t* p = src + (size_t)yi * stride + (size_t)x * 3u;
        int wq = k[(size_t)(t + radius)];
        accR += p[0] * wq; accG += p[1] * wq; accB += p[2] * wq;
      }
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = (uint8_t)((accR + 128) >> 8);
      q[1] = (uint8_t)((accG + 128) >> 8);
      q[2] = (uint8_t)((accB + 128) >> 8);
    }
  }

  // Bottom border rows (scalar)
  if (radius > 0) {
    gb_vpass_scalar(src + (size_t)(h - radius) * stride,
                    dst + (size_t)(h - radius) * stride,
                    w, radius, stride, k, radius);
  }
}
#endif // GB_NEON

// Public API: out-of-place
static inline void applyGaussianBlurRGB24_neon(const uint8_t* src,
                                          uint8_t* dst,
                                          int w, int h,
                                          float sigma,
                                          size_t strideBytes = 0)
{
  if (!src || !dst || w <= 0 || h <= 0) return;

  std::vector<uint16_t> kq8;
  int radius = gb_build_kernel_q8(sigma, kq8);
  const uint16_t* k = kq8.data();

  const size_t stride = (strideBytes ? strideBytes : (size_t)w * 3u);

  if (radius == 0) {
    // copy
    if (src != dst) {
      for (int y = 0; y < h; ++y) {
        std::copy_n(src + (size_t)y * stride, (size_t)w * 3u, dst + (size_t)y * stride);
      }
    }
    return;
  }

  // temp buffer for separable passes
  std::vector<uint8_t> tmp((size_t)h * stride);

  // H pass
#if GB_NEON
  for (int y = 0; y < h; ++y) {
    const uint8_t* srow = src + (size_t)y * stride;
    uint8_t* drow = tmp.data() + (size_t)y * stride;
    if (w >= 16 + 2 * radius) {
      gb_hpass_neon_row(srow, drow, w, stride, k, radius);
    } else {
      // too narrow for vectorized interior
      gb_hpass_scalar(srow, drow, w, 1, (size_t)w * 3u, k, radius);
    }
  }
#else
  gb_hpass_scalar(src, tmp.data(), w, h, stride, k, radius);
#endif

  // V pass
#if GB_NEON
  if (h >= 1 + 2 * radius && w >= 1) {
    gb_vpass_neon_blocky(tmp.data(), dst, w, h, stride, k, radius);
  } else {
    gb_vpass_scalar(tmp.data(), dst, w, h, stride, k, radius);
  }
#else
  gb_vpass_scalar(tmp.data(), dst, w, h, stride, k, radius);
#endif
}

// Public API: in-place
static inline void applyGaussianBlurRGB24_neon_inplace(uint8_t* buf,
                                                  int w, int h,
                                                  float sigma,
                                                  size_t strideBytes = 0)
{
  applyGaussianBlurRGB24_neon(buf, buf, w, h, sigma, strideBytes);
}
