// vignette_cuda.cu
#include "vignette_cuda.h"
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>

// Simple CUDA error helper
static inline bool check_cuda(const char* msg, cudaError_t err) {
  if (err != cudaSuccess) {
    fprintf(stderr, "[cuda][vignette] %s: %s\n", msg, cudaGetErrorString(err));
    return false;
  }
  return true;
}

// Reused device buffer to avoid reallocations each frame
static uint8_t* g_d_rgb = nullptr;
static size_t   g_d_capacity = 0; // bytes
static int      g_last_w = 0;
static int      g_last_h = 0;

__global__ void vignette_kernel(uint8_t* d_rgb, int width, int height, VignetteParamsCUDA p) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int total = width * height;
  if (idx >= total) return;

  int x = idx % width;
  int y = idx / width;

  // Normalize coords to [-0.5..0.5]-ish range using image dims
  float cx = p.center_x * (float)width;
  float cy = p.center_y * (float)height;
  float nx = (x - cx) / (float)width;
  float ny = (y - cy) / (float)height;

  // Rotate by angle (compute sin/cos once per thread; cheap vs memory traffic)
  float s = sinf(p.angle_rad);
  float c = cosf(p.angle_rad);
  float rx = c * nx - s * ny;
  float ry = s * nx + c * ny;

  // Elliptical scaling
  rx *= p.axis_scale_x;
  ry *= p.axis_scale_y;

  // Radial distance
  float r = sqrtf(rx * rx + ry * ry);

  // Map radius to [0..1] falloff coordinate t
  float inner = p.inner_radius;
  float outer = p.outer_radius;
  float denom = fmaxf(outer - inner, 1e-6f);
  float t = (r - inner) / denom;
  t = fminf(fmaxf(t, 0.0f), 1.0f);

  // Smoothstep falloff and apply strength
  float w = t * t * (3.f - 2.f * t);
  float gain = 1.f - p.strength * w;
  if (gain < 0.f) gain = 0.f;

  int base = idx * 3;
  float r0 = d_rgb[base + 0] * gain;
  float g0 = d_rgb[base + 1] * gain;
  float b0 = d_rgb[base + 2] * gain;

  // Clamp and write back
  d_rgb[base + 0] = (uint8_t)fminf(fmaxf(r0, 0.f), 255.f);
  d_rgb[base + 1] = (uint8_t)fminf(fmaxf(g0, 0.f), 255.f);
  d_rgb[base + 2] = (uint8_t)fminf(fmaxf(b0, 0.f), 255.f);
}

extern "C" bool apply_vignette_cuda(uint8_t* host_rgb, int width, int height, const VignetteParamsCUDA* params) {
  if (!host_rgb || !params || width <= 0 || height <= 0) return false;

  // Ensure CUDA context
  if (!check_cuda("cudaFree(0) init", cudaFree(0))) return false;

  size_t bytes = (size_t)width * (size_t)height * 3ull;
  if (g_d_capacity < bytes || width != g_last_w || height != g_last_h) {
    if (g_d_rgb) cudaFree(g_d_rgb);
    g_d_rgb = nullptr;
    g_d_capacity = 0;
    if (!check_cuda("cudaMalloc d_rgb", cudaMalloc((void**)&g_d_rgb, bytes))) return false;
    g_d_capacity = bytes;
    g_last_w = width;
    g_last_h = height;
  }

  // H2D
  if (!check_cuda("cudaMemcpy H2D", cudaMemcpy(g_d_rgb, host_rgb, bytes, cudaMemcpyHostToDevice))) return false;

  // Launch kernel
  int threads = 256;
  int pixels  = width * height;
  int blocks  = (pixels + threads - 1) / threads;
  vignette_kernel<<<blocks, threads>>>(g_d_rgb, width, height, *params);
  if (!check_cuda("kernel launch", cudaGetLastError())) return false;
  if (!check_cuda("cudaDeviceSynchronize", cudaDeviceSynchronize())) return false;

  // D2H
  if (!check_cuda("cudaMemcpy D2H", cudaMemcpy(host_rgb, g_d_rgb, bytes, cudaMemcpyDeviceToHost))) return false;

  return true;
}
