#include "common/slalom.hpp"

namespace slalom {

SegmentResult push(PlanProfile& plan, const Param& p, TurnDir dir) {
    SegmentResult r = validate(p);
    if (r != SegmentResult::ok) return r;

    // 正の角度で左旋回（PlanProfile::turn()）。シミュレータの角度は時計回りが正なので，右旋回で符号が一致する
    float s = (dir == TurnDir::left) ? 1.f : -1.f;
    Shape shape = shapeOf(p);

    // 検査は済んでいるので，ここで弾かれるのはキューの空き不足だけ。最初に失敗した結果を返す
    auto keep_first = [&r](SegmentResult result) {
        if (r == SegmentResult::ok) r = result;
    };
    if (p.pre_offset != 0.f) keep_first(plan.straight(p.speed, p.pre_offset));
    keep_first(plan.turn(s * p.omega_max, s * shape.ramp_angle));
    if (shape.cruise_angle > 0.f) keep_first(plan.turn(s * p.omega_max, s * shape.cruise_angle));
    keep_first(plan.turn(0.f, s * shape.ramp_angle));
    if (p.post_offset != 0.f) keep_first(plan.straight(p.speed, p.post_offset));
    return r;
}

const char* resultName(SegmentResult r) {
    switch (r) {
        case SegmentResult::ok: return "ok";
        case SegmentResult::zeroDistance: return "zeroDistance";
        case SegmentResult::directionMismatch: return "directionMismatch";
        case SegmentResult::accelLimit: return "accelLimit";
        case SegmentResult::tooShort: return "tooShort";
        case SegmentResult::queueFull: return "queueFull";
    }
    return "unknown";
}

} // namespace slalom
