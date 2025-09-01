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

class VignetteFilter {
public:
  VignetteFilter(int width, int height, const VignetteParams& params, std::size_t threads = 0);
  void setParams(const VignetteParams& params);
  const VignetteParams& params() const { return params_; }
  void resize(int width, int height);
  int width() const { return w_; }
  int height() const { return h_; }

  // Existing vector-based API
  void apply(const uint8_t* src_rgb24,
             std::vector<uint8_t>& dst_rgb24,
             std::vector<uint8_t>* optionalMaskOut = nullptr) const;

  // New pointer-based API (supports arrays, std::array, raw buffers).
  // strideBytes: 0 => tightly packed (width*3). dst shares the same stride.
  // optionalMaskOut: if non-null, must be width*height bytes.
  void apply(const uint8_t* src_rgb24,
             uint8_t* dst_rgb24,
             std::size_t strideBytes = 0,
             uint8_t* optionalMaskOut = nullptr) const;

  // New in-place convenience: reads from buf, writes back to buf (uses a temp copy internally).
  void applyInPlace(uint8_t* buf_rgb24,
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
