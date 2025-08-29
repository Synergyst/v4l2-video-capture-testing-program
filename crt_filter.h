// ... existing includes ...
#pragma once
#include <vector>
#include <random>
#include <cstdint>
#include "thread_pool.h"

struct CRTParams {
    float flicker_60hz = 0.04f;
    float flicker_noise = 0.01f;
    float scanline_strength = 0.25f; // 0..1
    float mask_strength = 0.15f;     // 0..1
    float grain_strength = 0.02f;    // 0..1
    float h_warp_amp = 2.0f;         // pixels
    float h_warp_freq_y = 0.03f;     // per-line
    float h_warp_freq_t = 0.8f;      // per-second
    float v_shake_amp = 1.2f;        // lines
    float wobble_line_noise = 0.25f; // extra wobble noise per line (pixels)
    size_t block_rows = 32;          // multithread chunk size (rows)
};

// Stateful multithread CRT filter (not thread-safe across apply() calls on the same instance)
class CRTFilter {
public:
    // New: fps is stored internally (default 30). Call set_fps() to change later.
    CRTFilter(int width, int height, const CRTParams& p, size_t threads, int fps = 30);

    int width() const { return w_; }
    int height() const { return h_; }

    // New: apply using internal frame counter (auto-incremented each call).
    void apply(const uint8_t* src_rgb24, std::vector<uint8_t>& dst);

    // Legacy API (still supported): explicit frame index + fps
    void apply(const uint8_t* src_rgb24, int frame_idx, int fps, std::vector<uint8_t>& dst);

    // Optional: change FPS driving the internal timeline
    void set_fps(int fps) { fps_ = (fps > 0) ? fps : 30; }

private:
    int w_;
    int h_;
    CRTParams p_;
    ThreadPool pool_;

    // State across frames
    std::mt19937 rng_;
    float phase60_ = 0.0f;
    float phaseH_ = 0.0f;
    float phaseV_ = 0.0f;
    int v_bounce_frames_ = 0;
    float v_bounce_amp_ = 0.0f;

    // New: internal timeline
    int fps_ = 30;
    long long frame_idx_ = 0;

    struct FrameConsts {
        float t;
        float flicker;
        float v_shift;
        int vphase;
    };

    FrameConsts prepare_frame_(int frame_idx, int fps);
    static inline uint8_t clamp_u8_(int v);
    static inline float hash31_(int x, int y, int t); // [-1,1]
};
