#include "crt_filter.h"
#include <cmath>
#include <algorithm>

static inline float frand(std::mt19937& rng, float a, float b) {
    std::uniform_real_distribution<float> dist(a, b);
    return dist(rng);
}

// Updated: constructor now stores fps internally
CRTFilter::CRTFilter(int width, int height, const CRTParams& p, size_t threads, int fps)
    : w_(width), h_(height), p_(p), pool_(threads), fps_((fps > 0) ? fps : 30)
{
    std::random_device rd;
    rng_.seed(rd());
    phase60_ = frand(rng_, 0.0f, 6.2831853f);
    phaseH_  = frand(rng_, 0.0f, 6.2831853f);
    phaseV_  = frand(rng_, 0.0f, 6.2831853f);
}

CRTFilter::FrameConsts CRTFilter::prepare_frame_(int frame_idx, int fps) {
    const float TWO_PI = 6.28318530718f;
    const float t = (fps > 0) ? (float)frame_idx / (float)fps : (float)frame_idx * 0.0333333f;

    // Per-frame flicker
    float flicker = 1.0f
        + p_.flicker_60hz * std::sin(TWO_PI * 60.0f * t + phase60_)
        + frand(rng_, -p_.flicker_noise, p_.flicker_noise);

    // Occasional vertical sync "bounce"
    if (v_bounce_frames_ == 0 && frand(rng_, 0.0f, 1.0f) < 0.015f) {
        v_bounce_frames_ = 50 + (int)frand(rng_, 0.0f, 40.0f);
        v_bounce_amp_    = frand(rng_, 1.0f, 3.0f);
    }

    float v_bounce = 0.0f;
    if (v_bounce_frames_ > 0) {
        float decay = (float)v_bounce_frames_ / 90.0f;
        v_bounce = v_bounce_amp_ * std::sin(TWO_PI * 2.0f * t + phaseV_) * decay;
        v_bounce_frames_--;
    }

    // Base vertical jitter
    float v_jitter = p_.v_shake_amp * std::sin(TWO_PI * 0.9f * t + phaseV_);
    float v_shift = v_bounce + v_jitter; // in lines

    FrameConsts fc;
    fc.t = t;
    fc.flicker = flicker;
    fc.v_shift = v_shift;
    fc.vphase = frame_idx & 1;
    return fc;
}

inline uint8_t CRTFilter::clamp_u8_(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

inline float CRTFilter::hash31_(int x, int y, int t) {
    uint32_t h = (uint32_t)(x * 374761393u + y * 668265263u) ^ (uint32_t)(t * 362437u);
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= (h >> 16);
    // map to [-1,1]
    return ((h & 0xFFFFFFu) / float(0x7FFFFFu)) * 2.0f - 1.0f;
}

// New: apply() that auto-advances internal frame counter
void CRTFilter::apply(const uint8_t* src, std::vector<uint8_t>& dst) {
    // Delegate to legacy API with internal timeline
    this->apply(src, static_cast<int>(frame_idx_), fps_, dst);
    ++frame_idx_;
}

void CRTFilter::apply(const uint8_t* src, int frame_idx, int fps, std::vector<uint8_t>& dst) {
    dst.resize((size_t)w_ * (size_t)h_ * 3);
    const int stride = w_ * 3;

    const auto fc = prepare_frame_(frame_idx, fps);
    const float TWO_PI = 6.28318530718f;

    // Precompute triad mask (RGB shadow mask)
    float triad[3][3] = {
        {1.00f, 0.80f, 0.75f}, // R
        {0.75f, 1.00f, 0.80f}, // G
        {0.80f, 0.75f, 1.00f}, // B
    };

    auto proc_rows = [&](size_t y0, size_t y1) {
        for (size_t y = y0; y < y1; ++y) {
            // Vertical shift
            int y_src = static_cast<int>(y) - (int)std::lround(fc.v_shift);
            if (y_src < 0) y_src = 0;
            if (y_src >= h_) y_src = h_ - 1;

            // Horizontal wobble for this scanline
            float wobble_base = p_.h_warp_amp * std::sin(TWO_PI * (p_.h_warp_freq_y * (float)y + p_.h_warp_freq_t * fc.t) + phaseH_);
            float wobble_noise = p_.wobble_line_noise * hash31_(0, (int)y, frame_idx);
            float wobble = wobble_base + wobble_noise;

            // Scanline darkening
            float scan_gain = (((int)y + fc.vphase) & 1) ? (1.0f - p_.scanline_strength) : 1.0f;

            // Combine with per-frame flicker
            float base_gain = fc.flicker * scan_gain;

            const uint8_t* src_row = &src[y_src * stride];
            uint8_t* dst_row = &dst[(size_t)y * (size_t)stride];

            for (int x = 0; x < w_; ++x) {
                int x_src = x + (int)std::lround(wobble);
                if (x_src < 0) x_src = 0;
                if (x_src >= w_) x_src = w_ - 1;

                int si = x_src * 3;
                uint8_t r = src_row[si + 0];
                uint8_t g = src_row[si + 1];
                uint8_t b = src_row[si + 2];

                // RGB triad
                int tri = x % 3;
                float mr = 1.0f - p_.mask_strength + p_.mask_strength * triad[tri][0];
                float mg = 1.0f - p_.mask_strength + p_.mask_strength * triad[tri][1];
                float mb = 1.0f - p_.mask_strength + p_.mask_strength * triad[tri][2];

                // Grain per pixel
                float grain = p_.grain_strength * hash31_(x, (int)y, frame_idx);

                float rf = (float)r * base_gain * mr + 255.0f * grain * 0.5f;
                float gf = (float)g * base_gain * mg + 255.0f * grain * 0.5f;
                float bf = (float)b * base_gain * mb + 255.0f * grain * 0.5f;

                dst_row[x * 3 + 0] = clamp_u8_((int)std::lround(rf));
                dst_row[x * 3 + 1] = clamp_u8_((int)std::lround(gf));
                dst_row[x * 3 + 2] = clamp_u8_((int)std::lround(bf));
            }
        }
    };

    pool_.parallel_for(0, (size_t)h_, std::max<size_t>(1, p_.block_rows), proc_rows);
}
