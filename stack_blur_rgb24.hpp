// stack_blur_rgb24.hpp (or place into your .cpp)
// Compile with: -O3 -ffast-math -funroll-loops
// Optional: -fopenmp (OpenMP) and on ARM: -mfpu=neon -ftree-vectorize -flax-vector-conversions

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <vector>

#ifndef SBR_OMP
#  ifdef _OPENMP
#    include <omp.h>
#    define SBR_OMP 1
#  else
#    define SBR_OMP 0
#  endif
#endif

// Clamp helper
static inline int sbr_clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Horizontal pass of Stack Blur (RGB24). In/out are different buffers.
static inline void sbr_pass_h(const uint8_t* src, uint8_t* dst,
                              int width, int height, int radius, size_t stride)
{
  const int div = radius * 2 + 1;
  // Division table to avoid divides in inner loop
  std::vector<uint8_t> dv(256 * div);
  for (int i = 0; i < (int)dv.size(); ++i) dv[i] = (uint8_t)(i / div);

#if SBR_OMP
#pragma omp parallel for schedule(static)
#endif
  for (int y = 0; y < height; ++y) {
    const uint8_t* srow = src + (size_t)y * stride;
    uint8_t* drow = dst + (size_t)y * stride;

    // Stack holds last (2r+1) rgb triplets in the window
    std::vector<int> stack((size_t)div * 3, 0);

    int sumR = 0, sumG = 0, sumB = 0;
    int sumInR = 0, sumInG = 0, sumInB = 0;
    int sumOutR = 0, sumOutG = 0, sumOutB = 0;

    // Prime the window centered at x=0 (edge clamped)
    for (int i = -radius; i <= radius; ++i) {
      const int xi = sbr_clampi(i, 0, width - 1);
      const uint8_t* p = srow + (size_t)xi * 3u;
      const int r = p[0], g = p[1], b = p[2];
      const int w = radius + 1 - std::abs(i);

      stack[(i + radius) * 3 + 0] = r;
      stack[(i + radius) * 3 + 1] = g;
      stack[(i + radius) * 3 + 2] = b;

      sumR += r * w; sumG += g * w; sumB += b * w;

      if (i > 0) {
        sumInR += r; sumInG += g; sumInB += b;
      } else {
        sumOutR += r; sumOutG += g; sumOutB += b;
      }
    }

    int sp = radius; // stack pointer

    for (int x = 0; x < width; ++x) {
      // Write averaged pixel
      uint8_t* q = drow + (size_t)x * 3u;
      q[0] = dv[sumR]; q[1] = dv[sumG]; q[2] = dv[sumB];

      // Slide: remove outgoing
      sumR -= sumOutR; sumG -= sumOutG; sumB -= sumOutB;

      // Pop from stack
      const int stackStart = (sp - radius + div) % div;
      int* stk = &stack[(size_t)stackStart * 3];

      sumOutR -= stk[0]; sumOutG -= stk[1]; sumOutB -= stk[2];

      // Next sample (incoming)
      const int xp = sbr_clampi(x + radius + 1, 0, width - 1);
      const uint8_t* p = srow + (size_t)xp * 3u;
      const int rin = p[0], gin = p[1], bin = p[2];

      stk[0] = rin; stk[1] = gin; stk[2] = bin;

      sumInR += rin; sumInG += gin; sumInB += bin;
      sumR += sumInR; sumG += sumInG; sumB += sumInB;

      // Advance stack pointer
      sp = (sp + 1) % div;
      stk = &stack[(size_t)sp * 3];

      sumOutR += stk[0]; sumOutG += stk[1]; sumOutB += stk[2];
      sumInR  -= stk[0]; sumInG  -= stk[1]; sumInB  -= stk[2];
    }
  }
}

// Vertical pass of Stack Blur (RGB24). In/out are different buffers.
static inline void sbr_pass_v(const uint8_t* src, uint8_t* dst,
                              int width, int height, int radius, size_t stride)
{
  const int div = radius * 2 + 1;
  std::vector<uint8_t> dv(256 * div);
  for (int i = 0; i < (int)dv.size(); ++i) dv[i] = (uint8_t)(i / div);

#if SBR_OMP
#pragma omp parallel for schedule(static)
#endif
  for (int x = 0; x < width; ++x) {
    std::vector<int> stack((size_t)div * 3, 0);

    int sumR = 0, sumG = 0, sumB = 0;
    int sumInR = 0, sumInG = 0, sumInB = 0;
    int sumOutR = 0, sumOutG = 0, sumOutB = 0;

    // Prime the window centered at y=0 (edge clamped)
    for (int i = -radius; i <= radius; ++i) {
      const int yi = sbr_clampi(i, 0, height - 1);
      const uint8_t* p = src + (size_t)yi * stride + (size_t)x * 3u;
      const int r = p[0], g = p[1], b = p[2];
      const int w = radius + 1 - std::abs(i);

      stack[(i + radius) * 3 + 0] = r;
      stack[(i + radius) * 3 + 1] = g;
      stack[(i + radius) * 3 + 2] = b;

      sumR += r * w; sumG += g * w; sumB += b * w;

      if (i > 0) {
        sumInR += r; sumInG += g; sumInB += b;
      } else {
        sumOutR += r; sumOutG += g; sumOutB += b;
      }
    }

    int sp = radius;

    for (int y = 0; y < height; ++y) {
      uint8_t* q = dst + (size_t)y * stride + (size_t)x * 3u;
      q[0] = dv[sumR]; q[1] = dv[sumG]; q[2] = dv[sumB];

      sumR -= sumOutR; sumG -= sumOutG; sumB -= sumOutB;

      const int stackStart = (sp - radius + div) % div;
      int* stk = &stack[(size_t)stackStart * 3];

      sumOutR -= stk[0]; sumOutG -= stk[1]; sumOutB -= stk[2];

      const int yp = sbr_clampi(y + radius + 1, 0, height - 1);
      const uint8_t* p = src + (size_t)yp * stride + (size_t)x * 3u;
      const int rin = p[0], gin = p[1], bin = p[2];

      stk[0] = rin; stk[1] = gin; stk[2] = bin;

      sumInR += rin; sumInG += gin; sumInB += bin;
      sumR += sumInR; sumG += sumInG; sumB += sumInB;

      sp = (sp + 1) % div;
      stk = &stack[(size_t)sp * 3];

      sumOutR += stk[0]; sumOutG += stk[1]; sumOutB += stk[2];
      sumInR  -= stk[0]; sumInG  -= stk[1]; sumInB  -= stk[2];
    }
  }
}

// Out-of-place: src -> dst
static inline void stackBlurRGB24(const uint8_t* src,
                                  uint8_t* dst,
                                  int width, int height,
                                  int radius,
                                  size_t strideBytes = 0)
{
  if (!src || !dst || width <= 0 || height <= 0 || radius <= 0) {
    // copy if needed
    if (src && dst && src != dst) {
      const size_t stride = (strideBytes ? strideBytes : (size_t)width * 3u);
      for (int y=0; y<height; ++y)
        std::copy_n(src + (size_t)y * stride, (size_t)width * 3u, dst + (size_t)y * stride);
    }
    return;
  }
  const size_t stride = (strideBytes ? strideBytes : (size_t)width * 3u);

  std::vector<uint8_t> tmp((size_t)height * stride);
  sbr_pass_h(src, tmp.data(), width, height, radius, stride);
  sbr_pass_v(tmp.data(), dst, width, height, radius, stride);
}

// In-place convenience
static inline void stackBlurRGB24_inplace(uint8_t* buf,
                                          int width, int height,
                                          int radius,
                                          size_t strideBytes = 0)
{
  if (!buf || width <= 0 || height <= 0 || radius <= 0) return;
  const size_t stride = (strideBytes ? strideBytes : (size_t)width * 3u);
  std::vector<uint8_t> tmp((size_t)height * stride);
  // buf -> tmp -> buf
  sbr_pass_h(buf, tmp.data(), width, height, radius, stride);
  sbr_pass_v(tmp.data(), buf, width, height, radius, stride);
}

// Vector helper
static inline void stackBlurRGB24_vec(const uint8_t* src,
                                      int width, int height,
                                      int radius,
                                      std::vector<uint8_t>& dstOut)
{
  const size_t stride = (size_t)width * 3u;
  dstOut.resize((size_t)height * stride);
  stackBlurRGB24(src, dstOut.data(), width, height, radius, /*strideBytes=*/0);
}
