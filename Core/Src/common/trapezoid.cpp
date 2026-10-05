#include "common/trapezoid.hpp"
#include <cmath>

namespace trapezoid {

uint8_t split(float d, float v_in, float v_out, float v_max, float a, float b, Part out[MAX_PARTS]) {
    // 加速度 a で上げて減速度 b で下げ、ちょうど d 進む頂点の速度（三角）
    float v_peak = std::sqrt((2.f * a * b * d + b * v_in * v_in + a * v_out * v_out) / (a + b));
    float cruise = 0.f;
    if (v_peak > v_max) {
        v_peak = v_max;
        cruise = d - (v_max * v_max - v_in * v_in) / (2.f * a) - (v_max * v_max - v_out * v_out) / (2.f * b);
    }
    // 始め・終わりの速度に届かない（d が短い）ときや、区間がごく短くなるときは、1区間で v_in → v_out
    float d_acc = (v_peak * v_peak - v_in * v_in) / (2.f * a);
    float d_dec = (v_peak * v_peak - v_out * v_out) / (2.f * b);
    bool tiny_acc = d_acc < MIN_PART_MM && v_peak - v_in > 1.f;
    bool tiny_dec = d_dec < MIN_PART_MM && v_peak - v_out > 1.f;
    if (v_peak < v_in || v_peak < v_out || tiny_acc || tiny_dec) {
        out[0] = Part{v_out, d};
        return 1;
    }

    uint8_t n = 0;
    if (d_acc >= MIN_PART_MM) out[n++] = Part{v_peak, d_acc};
    else cruise += d_acc;   // 速度がほとんど変わらない加速は等速に含める
    float d_dec_used = (d_dec >= MIN_PART_MM) ? d_dec : 0.f;
    if (d_dec_used == 0.f) cruise += d_dec;
    if (cruise >= MIN_PART_MM) {
        out[n++] = Part{v_peak, cruise};
    } else if (cruise > 0.f) {
        // ごく短い等速は減速(なければ加速)の区間に含める
        if (d_dec_used > 0.f) d_dec_used += cruise;
        else if (n > 0) out[n - 1].distance += cruise;
    }
    if (d_dec_used > 0.f) out[n++] = Part{v_out, d_dec_used};
    if (n == 0) out[n++] = Part{v_out, d};
    // 等速だけ（v_in = v_peak = v_out）のときなどで終速を合わせる
    out[n - 1].v_end = v_out;
    return n;
}

} // namespace trapezoid
