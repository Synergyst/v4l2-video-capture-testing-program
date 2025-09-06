// vignette_cuda.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Parameters for CUDA vignette (all normalized except angle)
// - center_x, center_y: vignette center in normalized coords [0..1] across width/height
// - inner_radius: normalized radius where effect begins (no darkening inside)
// - outer_radius: normalized radius where effect reaches full strength
// - strength: 0..2 typically (1.0 is a good start). 0 disables effect.
// - axis_scale_x, axis_scale_y: elliptical scaling (1.0 = circle)
// - angle_rad: rotation in radians
typedef struct VignetteParamsCUDA {
  float center_x;
  float center_y;
  float inner_radius;
  float outer_radius;
  float strength;
  float axis_scale_x;
  float axis_scale_y;
  float angle_rad;
} VignetteParamsCUDA;

// Applies vignette in-place on an RGB24 buffer (host memory).
// - host_rgb: pointer to width*height*3 bytes (RGB24, tightly packed).
// - width, height: image dimensions.
// - params: pointer to configured VignetteParamsCUDA.
// Returns true on success, false on CUDA error.
// Internally reuses a persistent device buffer sized to the latest (w*h*3).
bool apply_vignette_cuda(uint8_t* host_rgb, int width, int height, const VignetteParamsCUDA* params);

#ifdef __cplusplus
} // extern "C"
#endif
