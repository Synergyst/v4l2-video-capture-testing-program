#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

enum class VignetteBlendMode { Multiply, LerpToColor };

struct VignetteParams {
  float center_x = 0.5f, center_y = 0.5f;
  float inner_radius = 0.6f, outer_radius = 0.95f;
  float axis_scale_x = 1.0f, axis_scale_y = 1.0f, angle_deg = 0.0f;
  float strength = 1.0f;
  float color[3] = {0.0f, 0.0f, 0.0f};
  bool  gamma_correct = true; float gamma = 2.2f;
  bool invert = false;
  bool clamp_output = true;
  VignetteBlendMode mode = VignetteBlendMode::Multiply;
};

// New: parameters for a rounded-rectangle (bounding-box) vignette
struct VignetteBoxParams {
  // Box in normalized coordinates [0..1], axis-aligned
  // x0 < x1, y0 < y1 (will be clamped/ordered in code)
  float x0 = 0.1f;
  float y0 = 0.1f;
  float x1 = 0.9f;
  float y1 = 0.9f;

  // Corner radius normalized to the box min dimension
  // radius_px = corner_radius_norm * min(box_w, box_h)
  float corner_radius_norm = 0.08f;

  // Edge feather width (how wide the falloff band is)
  // If <= 0, no feather. If in (0..1], treated as normalized to min(box_w, box_h)
  // If > 1, treated as pixels. Typical normalized values: 0.02..0.15
  float feather = 0.08f;

  // Strength [0..1]
  float strength = 1.0f;

  // Blend target color (range [0..1], RGB), same semantics as for ellipse vignette
  float color[3] = {0.0f, 0.0f, 0.0f};

  // Blend mode
  VignetteBlendMode mode = VignetteBlendMode::Multiply;

  // Gamma-correct blending
  bool  gamma_correct = true;
  float gamma = 2.2f;

  // If false: fade on the inside edge (classic vignette inside the box).
  // If true:  fade on the outside edge (effect outside the box, inside remains intact).
  bool invert = false;

  // Clamp final output to [0..255]
  bool clamp_output = true;
};

class VignetteFilter {
public:
  VignetteFilter(int width, int height, const VignetteParams& params, std::size_t threads = 0);

  void setParams(const VignetteParams& params);
  const VignetteParams& params() const { return params_; }

  void resize(int width, int height);
  int width() const { return w_; }
  int height() const { return h_; }

  // Elliptical vignette: vector-based API
  void apply(const uint8_t* src_rgb24,
             std::vector<uint8_t>& dst_rgb24,
             std::vector<uint8_t>* optionalMaskOut = nullptr) const;

  // Elliptical vignette: pointer-based API (supports raw arrays/std::array)
  // strideBytes: 0 => tightly packed (width*3). dst shares the same stride.
  // optionalMaskOut: if non-null, must be width*height bytes.
  void apply(const uint8_t* src_rgb24,
             uint8_t* dst_rgb24,
             std::size_t strideBytes = 0,
             uint8_t* optionalMaskOut = nullptr) const;

  // Elliptical vignette: in-place convenience
  void applyInPlace(uint8_t* buf_rgb24,
                    std::size_t strideBytes = 0,
                    uint8_t* optionalMaskOut = nullptr) const;

  // Rounded-rectangle vignette: vector API
  void applyRoundedBox(const uint8_t* src_rgb24,
                       std::vector<uint8_t>& dst_rgb24,
                       const VignetteBoxParams& box,
                       std::vector<uint8_t>* optionalMaskOut = nullptr) const;

  // Rounded-rectangle vignette: pointer API
  void applyRoundedBox(const uint8_t* src_rgb24,
                       uint8_t* dst_rgb24,
                       const VignetteBoxParams& box,
                       std::size_t strideBytes = 0,
                       uint8_t* optionalMaskOut = nullptr) const;

  // Rounded-rectangle vignette: in-place convenience
  void applyRoundedBoxInPlace(uint8_t* buf_rgb24,
                              const VignetteBoxParams& box,
                              std::size_t strideBytes = 0,
                              uint8_t* optionalMaskOut = nullptr) const;

private:
  int w_ = 0, h_ = 0;
  std::size_t threads_ = 1;
  VignetteParams params_;
  float ca_ = 1.0f, sa_ = 0.0f;
  float cx_ = 0.0f, cy_ = 0.0f;
  float hx_ = 1.0f, hy_ = 1.0f;
  float color_lin_[3] = {0.0f, 0.0f, 0.0f};
  bool doGamma_ = true;

  void recomputeCached_();
  inline float toLinear_(uint8_t u8) const;
  inline uint8_t toDisplay_(float lin) const;
};
