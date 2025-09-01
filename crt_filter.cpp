#include "crt_filter.h"
#include <cmath>
#include <algorithm>
#include <cstring>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  #include <arm_neon.h>
  #define CRT_NEON 1
#else
  #define CRT_NEON 0
#endif

CRTFilter::CRTFilter(int width, int height, const CRTParams& p, size_t threads, int fps)
    : w_(width), h_(height), p_(p), pool_(threads), fps_((fps > 0) ? fps : 30) {
    std::random_device rd;
    rng_.seed(rd());
    const float TWO_PI = 6.2831853071795864769f;
    phase60_ = frand_(rng_, 0.0f, TWO_PI);
    phaseH_  = frand_(rng_, 0.0f, TWO_PI);
    phaseV_  = frand_(rng_, 0.0f, TWO_PI);
}

CRTFilter::FrameConsts CRTFilter::prepare_frame_(int frame_idx, int fps) {
    const float TWO_PI = 6.2831853071795864769f;
    const float t = (fps > 0) ? (float)frame_idx / (float)fps : (float)frame_idx * (1.0f / 30.0f);

    // Per-frame flicker
    float flicker = 1.0f
        + p_.flicker_60hz * std::sin(TWO_PI * 60.0f * t + phase60_)
        + frand_(rng_, -p_.flicker_noise, p_.flicker_noise);

    // Occasional vertical sync "bounce"
    if (v_bounce_frames_ == 0 && frand_(rng_, 0.0f, 1.0f) < 0.015f) {
        v_bounce_frames_ = 50 + (int)frand_(rng_, 0.0f, 40.0f);
        v_bounce_amp_    = frand_(rng_, 1.0f, 3.0f);
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

// Auto-advancing apply
void CRTFilter::apply(const uint8_t* src, std::vector<uint8_t>& dst) {
    this->apply(src, static_cast<int>(frame_idx_), fps_, dst);
    ++frame_idx_;
}

void CRTFilter::apply(const uint8_t* src, int frame_idx, int fps, std::vector<uint8_t>& dst) {
    dst.resize((size_t)w_ * (size_t)h_ * 3u);
    const int stride = w_ * 3;
    const auto fc = prepare_frame_(frame_idx, fps);
    const float TWO_PI = 6.2831853071795864769f;

    // RGB triad multipliers (shadow mask), blended with mask_strength per triad
    const float ms = std::clamp(p_.mask_strength, 0.0f, 1.0f);
    const float triad[3][3] = {
        {1.00f, 0.80f, 0.75f}, // for pixel x%3==0
        {0.75f, 1.00f, 0.80f}, // x%3==1
        {0.80f, 0.75f, 1.00f}, // x%3==2
    };
    float tri_gainR[3], tri_gainG[3], tri_gainB[3];
    for (int i = 0; i < 3; ++i) {
        tri_gainR[i] = (1.0f - ms) + ms * triad[i][0];
        tri_gainG[i] = (1.0f - ms) + ms * triad[i][1];
        tri_gainB[i] = (1.0f - ms) + ms * triad[i][2];
    }

    // Process rows in parallel
    auto proc_rows = [&](size_t y0, size_t y1) {
        for (size_t y = y0; y < y1; ++y) {
            // Vertical shift (line jitter/bounce)
            int y_src = static_cast<int>(y) - (int)std::lround(fc.v_shift);
            if (y_src < 0) y_src = 0;
            if (y_src >= h_) y_src = h_ - 1;

            // Horizontal wobble (constant across the line) plus line noise
            float wobble_base = p_.h_warp_amp
                              * std::sin(TWO_PI * (p_.h_warp_freq_y * (float)y + p_.h_warp_freq_t * fc.t) + phaseH_);
            float wobble_noise = p_.wobble_line_noise * hash31_(0, (int)y, frame_idx);
            int wob_i = (int)std::lround(wobble_base + wobble_noise);

            // Scanline darkening (alternating lines)
            float scan_gain = (((int)y + fc.vphase) & 1) ? (1.0f - p_.scanline_strength) : 1.0f;

            // Combined per-line gain (flicker + scanline)
            float base_gain = fc.flicker * scan_gain;

            // Grain amplitude
            const float grain_amp = p_.grain_strength; // multiplier for hash [-1,1]

            const uint8_t* src_row_base = &src[(size_t)y_src * (size_t)stride];
            uint8_t* dst_row = &dst[(size_t)y * (size_t)stride];

            // Interior region where clamping is not needed:
            // x_src = x + wob_i, so valid x in [left .. right)
            int left  = std::max(0, -wob_i);
            int right = std::min(w_, w_ - wob_i);

            // Left edge (clamped)
            for (int x = 0; x < left; ++x) {
                int tri = x % 3;
                float mr = base_gain * tri_gainR[tri];
                float mg = base_gain * tri_gainG[tri];
                float mb = base_gain * tri_gainB[tri];

                const uint8_t* sp = src_row_base + 0; // x_src clamped to 0
                float grain = grain_amp * hash31_(x, (int)y, frame_idx);

                int r = (int)std::lround(sp[0] * mr + 127.5f * grain);
                int g = (int)std::lround(sp[1] * mg + 127.5f * grain);
                int b = (int)std::lround(sp[2] * mb + 127.5f * grain);

                dst_row[x*3+0] = clamp_u8_(r);
                dst_row[x*3+1] = clamp_u8_(g);
                dst_row[x*3+2] = clamp_u8_(b);
            }

            // Interior vectorized region
            if (right > left) {
                const uint8_t* src_row = src_row_base + (size_t)(left + wob_i) * 3u;
                uint8_t* dst_ptr = dst_row + (size_t)left * 3u;
                int remaining = right - left;

#if CRT_NEON
                while (remaining >= 16) {
                    // Build per-lane Q8 gains and grain offsets
                    uint16_t gainR_lo[8], gainR_hi[8];
                    uint16_t gainG_lo[8], gainG_hi[8];
                    uint16_t gainB_lo[8], gainB_hi[8];
                    int16_t  grain_lo[8],  grain_hi[8];

                    for (int i = 0; i < 16; ++i) {
                        int xx = left + i;
                        int tri = xx % 3;
                        float gR = base_gain * tri_gainR[tri];
                        float gG = base_gain * tri_gainG[tri];
                        float gB = base_gain * tri_gainB[tri];
                        int qR = (int)std::lround(gR * 256.0f);
                        int qG = (int)std::lround(gG * 256.0f);
                        int qB = (int)std::lround(gB * 256.0f);
                        qR = std::clamp(qR, 0, 1024);
                        qG = std::clamp(qG, 0, 1024);
                        qB = std::clamp(qB, 0, 1024);

                        float gf = grain_amp * hash31_(xx, (int)y, frame_idx);
                        int go = (int)std::lround(127.5f * gf);
                        go = std::clamp(go, -255, 255);

                        if (i < 8) {
                            gainR_lo[i] = (uint16_t)qR;
                            gainG_lo[i] = (uint16_t)qG;
                            gainB_lo[i] = (uint16_t)qB;
                            grain_lo[i] = (int16_t)go;
                        } else {
                            gainR_hi[i-8] = (uint16_t)qR;
                            gainG_hi[i-8] = (uint16_t)qG;
                            gainB_hi[i-8] = (uint16_t)qB;
                            grain_hi[i-8] = (int16_t)go;
                        }
                    }

                    uint16x8_t gRlo = vld1q_u16(gainR_lo);
                    uint16x8_t gRhi = vld1q_u16(gainR_hi);
                    uint16x8_t gGlo = vld1q_u16(gainG_lo);
                    uint16x8_t gGhi = vld1q_u16(gainG_hi);
                    uint16x8_t gBlo = vld1q_u16(gainB_lo);
                    uint16x8_t gBhi = vld1q_u16(gainB_hi);

                    int16x8_t gro  = vld1q_s16(grain_lo);
                    int16x8_t grhi = vld1q_s16(grain_hi);

                    // Load 16 RGB pixels (interleaved)
                    uint8x16x3_t v = vld3q_u8(src_row);

                    // Expand to u16
                    uint16x8_t r_lo = vmovl_u8(vget_low_u8(v.val[0]));
                    uint16x8_t r_hi = vmovl_u8(vget_high_u8(v.val[0]));
                    uint16x8_t g_lo = vmovl_u8(vget_low_u8(v.val[1]));
                    uint16x8_t g_hi = vmovl_u8(vget_high_u8(v.val[1]));
                    uint16x8_t b_lo = vmovl_u8(vget_low_u8(v.val[2]));
                    uint16x8_t b_hi = vmovl_u8(vget_high_u8(v.val[2]));

                    // Multiply by Q8 gains (u16*u16 -> u32), then >>8
                    uint32x4_t r0 = vmull_u16(vget_low_u16(r_lo), vget_low_u16(gRlo));
                    uint32x4_t r1 = vmull_u16(vget_high_u16(r_lo), vget_high_u16(gRlo));
                    uint32x4_t r2 = vmull_u16(vget_low_u16(r_hi), vget_low_u16(gRhi));
                    uint32x4_t r3 = vmull_u16(vget_high_u16(r_hi), vget_high_u16(gRhi));

                    uint32x4_t g0 = vmull_u16(vget_low_u16(g_lo), vget_low_u16(gGlo));
                    uint32x4_t g1 = vmull_u16(vget_high_u16(g_lo), vget_high_u16(gGlo));
                    uint32x4_t g2 = vmull_u16(vget_low_u16(g_hi), vget_low_u16(gGhi));
                    uint32x4_t g3 = vmull_u16(vget_high_u16(g_hi), vget_high_u16(gGhi));

                    uint32x4_t b0 = vmull_u16(vget_low_u16(b_lo), vget_low_u16(gBlo));
                    uint32x4_t b1 = vmull_u16(vget_high_u16(b_lo), vget_high_u16(gBlo));
                    uint32x4_t b2 = vmull_u16(vget_low_u16(b_hi), vget_low_u16(gBhi));
                    uint32x4_t b3 = vmull_u16(vget_high_u16(b_hi), vget_high_u16(gBhi));

                    // >>8 and pack back to u16
                    uint16x8_t r_lo_s = vcombine_u16(vshrn_n_u32(r0, 8), vshrn_n_u32(r1, 8));
                    uint16x8_t r_hi_s = vcombine_u16(vshrn_n_u32(r2, 8), vshrn_n_u32(r3, 8));
                    uint16x8_t g_lo_s = vcombine_u16(vshrn_n_u32(g0, 8), vshrn_n_u32(g1, 8));
                    uint16x8_t g_hi_s = vcombine_u16(vshrn_n_u32(g2, 8), vshrn_n_u32(g3, 8));
                    uint16x8_t b_lo_s = vcombine_u16(vshrn_n_u32(b0, 8), vshrn_n_u32(b1, 8));
                    uint16x8_t b_hi_s = vcombine_u16(vshrn_n_u32(b2, 8), vshrn_n_u32(b3, 8));

                    // Add grain offsets in s16 space then narrow to u8 with saturation
                    int16x8_t rlo_i = vreinterpretq_s16_u16(r_lo_s);
                    int16x8_t rhi_i = vreinterpretq_s16_u16(r_hi_s);
                    int16x8_t glo_i = vreinterpretq_s16_u16(g_lo_s);
                    int16x8_t ghi_i = vreinterpretq_s16_u16(g_hi_s);
                    int16x8_t blo_i = vreinterpretq_s16_u16(b_lo_s);
                    int16x8_t bhi_i = vreinterpretq_s16_u16(b_hi_s);

                    rlo_i = vqaddq_s16(rlo_i, gro);
                    rhi_i = vqaddq_s16(rhi_i, grhi);
                    glo_i = vqaddq_s16(glo_i, gro);
                    ghi_i = vqaddq_s16(ghi_i, grhi);
                    blo_i = vqaddq_s16(blo_i, gro);
                    bhi_i = vqaddq_s16(bhi_i, grhi);

                    uint8x8_t r8_lo = vqmovun_s16(rlo_i);
                    uint8x8_t r8_hi = vqmovun_s16(rhi_i);
                    uint8x8_t g8_lo = vqmovun_s16(glo_i);
                    uint8x8_t g8_hi = vqmovun_s16(ghi_i);
                    uint8x8_t b8_lo = vqmovun_s16(blo_i);
                    uint8x8_t b8_hi = vqmovun_s16(bhi_i);

                    uint8x16_t r8 = vcombine_u8(r8_lo, r8_hi);
                    uint8x16_t g8 = vcombine_u8(g8_lo, g8_hi);
                    uint8x16_t b8 = vcombine_u8(b8_lo, b8_hi);

                    uint8x16x3_t outv;
                    outv.val[0] = r8;
                    outv.val[1] = g8;
                    outv.val[2] = b8;

                    vst3q_u8(dst_ptr, outv);

                    src_row += 16 * 3;
                    dst_ptr += 16 * 3;
                    left += 16;
                    remaining -= 16;
                }
#endif // CRT_NEON

                // Scalar tail of interior region
                for (int i = 0; i < remaining; ++i) {
                    int x = left + i;
                    int xs = x + wob_i;
                    const uint8_t* sp = src_row_base + (size_t)xs * 3u;
                    int tri = x % 3;
                    float mr = base_gain * tri_gainR[tri];
                    float mg = base_gain * tri_gainG[tri];
                    float mb = base_gain * tri_gainB[tri];

                    float grain = grain_amp * hash31_(x, (int)y, frame_idx);

                    int r = (int)std::lround(sp[0] * mr + 127.5f * grain);
                    int g = (int)std::lround(sp[1] * mg + 127.5f * grain);
                    int b = (int)std::lround(sp[2] * mb + 127.5f * grain);

                    uint8_t* dp = dst_row + (size_t)x * 3u;
                    dp[0] = clamp_u8_(r);
                    dp[1] = clamp_u8_(g);
                    dp[2] = clamp_u8_(b);
                }
            }

            // Right edge (clamped)
            for (int x = right; x < w_; ++x) {
                int tri = x % 3;
                float mr = base_gain * tri_gainR[tri];
                float mg = base_gain * tri_gainG[tri];
                float mb = base_gain * tri_gainB[tri];

                const uint8_t* sp = src_row_base + (size_t)(w_ - 1) * 3u;
                float grain = grain_amp * hash31_(x, (int)y, frame_idx);

                int r = (int)std::lround(sp[0] * mr + 127.5f * grain);
                int g = (int)std::lround(sp[1] * mg + 127.5f * grain);
                int b = (int)std::lround(sp[2] * mb + 127.5f * grain);

                dst_row[x*3+0] = clamp_u8_(r);
                dst_row[x*3+1] = clamp_u8_(g);
                dst_row[x*3+2] = clamp_u8_(b);
            }
        }
    };

    pool_.parallel_for(0, (size_t)h_, std::max<size_t>(1, p_.block_rows), proc_rows);
}
