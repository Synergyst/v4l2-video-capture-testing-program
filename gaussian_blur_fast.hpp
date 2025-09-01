// gaussian_blur_fast.hpp or in your .cpp
#include <cstdint>
#include <cstddef>
#include <vector>
#include <cmath>
#include <algorithm>

#ifndef GBF_OMP
#  ifdef _OPENMP
#    include <omp.h>
#    define GBF_OMP 1
#  else
#    define GBF_OMP 0
#  endif
#endif

// Compute 3 box radii that approximate a Gaussian with stddev sigma.
// Based on "Fast Almost-Gaussian Filtering" (Huang et al) / Ivan Kutskir’s derivation.
static inline void gbf_compute_radii_from_sigma(float sigma, int radii[3]) {
  if (sigma <= 0.f) { radii[0]=radii[1]=radii[2]=0; return; }
  const int n = 3;
  const float wIdeal = std::sqrt((12.0f * sigma * sigma / n) + 1.0f);
  int wl = (int)std::floor(wIdeal);
  wl -= ((wl & 1) == 0);           // make odd
  const int wu = wl + 2;
  const float mFloat = (12.0f * sigma * sigma - n*wl*wl - 4*n*wl - 3*n) / (-4.0f*wl - 4.0f);
  int m = (int)std::round(mFloat);
  // First m boxes use wu, the rest use wl
  int w0 = (0 < m) ? wu : wl;
  int w1 = (1 < m) ? wu : wl;
  int w2 = (2 < m) ? wu : wl;
  radii[0] = (w0 - 1) / 2;
  radii[1] = (w1 - 1) / 2;
  radii[2] = (w2 - 1) / 2;
}

// Horizontal box blur, edge-clamped, RGB24, running sum.
// src and dst may alias (handled by external orchestration); this pass assumes they are different.
static inline void gbf_box_blur_h(const uint8_t* src, uint8_t* dst,
                                  int width, int height, int radius, size_t stride)
{
  if (radius <= 0) { // copy
#if GBF_OMP
#pragma omp parallel for schedule(static)
#endif
    for (int y=0; y<height; ++y) {
      const uint8_t* srow = src + (size_t)y * stride;
      uint8_t* drow = dst + (size_t)y * stride;
      std::copy_n(srow, (size_t)width * 3u, drow);
    }
    return;
  }
  const int w = 2*radius + 1;
  const float inv = 1.0f / (float)w;

#if GBF_OMP
#pragma omp parallel for schedule(static)
#endif
  for (int y=0; y<height; ++y) {
    const uint8_t* srow = src + (size_t)y * stride;
    uint8_t* drow = dst + (size_t)y * stride;

    // Initial window: replicate edge on the left
    int sumR = (int)srow[0] * (radius + 1);
    int sumG = (int)srow[1] * (radius + 1);
    int sumB = (int)srow[2] * (radius + 1);
    for (int i=1; i<=radius; ++i) {
      const uint8_t* p = srow + (size_t)std::min(i, width-1) * 3u;
      sumR += p[0]; sumG += p[1]; sumB += p[2];
    }

    for (int x=0; x<width; ++x) {
      const int i1 = std::min(x + radius + 1, width - 1);
      const int i2 = std::max(x - radius,       0);
      const uint8_t* p1 = srow + (size_t)i1 * 3u;
      const uint8_t* p2 = srow + (size_t)i2 * 3u;
      // Write current average
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = (uint8_t)std::lround(sumR * inv);
      q[1] = (uint8_t)std::lround(sumG * inv);
      q[2] = (uint8_t)std::lround(sumB * inv);
      // Slide window
      sumR += (int)p1[0] - (int)p2[0];
      sumG += (int)p1[1] - (int)p2[1];
      sumB += (int)p1[2] - (int)p2[2];
    }
  }
}

// Vertical box blur, edge-clamped, RGB24, running sum.
static inline void gbf_box_blur_v(const uint8_t* src, uint8_t* dst,
                                  int width, int height, int radius, size_t stride)
{
  if (radius <= 0) {
#if GBF_OMP
#pragma omp parallel for schedule(static)
#endif
    for (int y=0; y<height; ++y) {
      const uint8_t* srow = src + (size_t)y * stride;
      uint8_t* drow = dst + (size_t)y * stride;
      std::copy_n(srow, (size_t)width * 3u, drow);
    }
    return;
  }
  const int w = 2*radius + 1;
  const float inv = 1.0f / (float)w;

#if GBF_OMP
#pragma omp parallel for schedule(static)
#endif
  for (int x=0; x<width; ++x) {
    // initial window at y=0 replicates top row
    const uint8_t* p0 = src + 0 * stride + (size_t)x * 3u;
    int sumR = (int)p0[0] * (radius + 1);
    int sumG = (int)p0[1] * (radius + 1);
    int sumB = (int)p0[2] * (radius + 1);
    for (int i=1; i<=radius; ++i) {
      const uint8_t* p = src + (size_t)std::min(i, height-1) * stride + (size_t)x * 3u;
      sumR += p[0]; sumG += p[1]; sumB += p[2];
    }

    for (int y=0; y<height; ++y) {
      const int j1 = std::min(y + radius + 1, height - 1);
      const int j2 = std::max(y - radius,       0);
      const uint8_t* p1 = src + (size_t)j1 * stride + (size_t)x * 3u;
      const uint8_t* p2 = src + (size_t)j2 * stride + (size_t)x * 3u;

      uint8_t* q = dst + (size_t)y * stride + (size_t)x * 3u;
      q[0] = (uint8_t)std::lround(sumR * inv);
      q[1] = (uint8_t)std::lround(sumG * inv);
      q[2] = (uint8_t)std::lround(sumB * inv);

      sumR += (int)p1[0] - (int)p2[0];
      sumG += (int)p1[1] - (int)p2[1];
      sumB += (int)p1[2] - (int)p2[2];
    }
  }
}

// Core: apply 3 box blurs to approximate Gaussian.
// src and dst can be the same pointer (in-place supported).
static inline void applyGaussianBlurRGB24_fast(const uint8_t* src,
                                          uint8_t* dst,
                                          int width, int height,
                                          float sigma,
                                          size_t strideBytes = 0)
{
  if (!src || !dst || width <= 0 || height <= 0 || sigma <= 0.f) {
    if (src && dst && src != dst) {
      const size_t stride = (strideBytes ? strideBytes : (size_t)width * 3u);
      for (int y=0; y<height; ++y)
        std::copy_n(src + (size_t)y * stride, (size_t)width * 3u, dst + (size_t)y * stride);
    }
    return;
  }

  const size_t stride = (strideBytes ? strideBytes : (size_t)width * 3u);

  // If in-place (src==dst), use an internal working buffer for ping-pong.
  std::vector<uint8_t> workA, workB;
  const uint8_t* curSrc = src;
  uint8_t* curDst = dst;

  if (src == dst) {
    workA.assign((size_t)height * stride, 0);
    std::copy_n(src, workA.size(), workA.data());
    curSrc = workA.data();
  }

  workB.assign((size_t)height * stride, 0);

  int radii[3]; gbf_compute_radii_from_sigma(sigma, radii);

  // For each box: H then V
  for (int k=0; k<3; ++k) {
    const int r = radii[k];
    // curSrc -> workB (H)
    gbf_box_blur_h(curSrc, workB.data(), width, height, r, stride);
    // workB -> curDst (V)
    gbf_box_blur_v(workB.data(), curDst, width, height, r, stride);

    // Next pass reads from curDst
    curSrc = curDst;

    // If curDst == dst but src==dst and more passes remain, flip to workA as destination to avoid extra copy
    if (k != 2) {
      if (curDst == dst) {
        // ping-pong to workA
        curDst = (src == dst) ? workA.data() : curDst; // if not in-place, keep writing to dst
      }
      // If not in-place, keep using (curSrc=dst) and (curDst=dst) is fine since we already updated curSrc
    }
  }

  // If final output ended up in workA (in-place case), copy back to dst
  if (curSrc != dst) {
    // curSrc holds the latest result
    for (int y=0; y<height; ++y) {
      const uint8_t* srow = curSrc + (size_t)y * stride;
      uint8_t* drow = dst + (size_t)y * stride;
      std::copy_n(srow, (size_t)width * 3u, drow);
    }
  }
}

// Convenience overloads
static inline void applyGaussianBlurRGB24_fast_inplace(uint8_t* buf,
                                                  int width, int height,
                                                  float sigma,
                                                  size_t strideBytes = 0) {
  applyGaussianBlurRGB24_fast(buf, buf, width, height, sigma, strideBytes);
}

static inline void applyGaussianBlurRGB24_fast_vec(const uint8_t* src,
                                              int width, int height,
                                              float sigma,
                                              std::vector<uint8_t>& dstOut) {
  const size_t stride = (size_t)width * 3u;
  dstOut.resize((size_t)height * stride);
  applyGaussianBlurRGB24_fast(src, dstOut.data(), width, height, sigma, /*strideBytes=*/0);
}
