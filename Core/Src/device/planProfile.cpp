#include "device/planProfile.hpp"

// ---- メイン側 ----

void PlanProfile::reset() {
    // update()が呼ばれていない（ISRがキューを読まない）間に呼ぶので，キューを直接空にしてよい
    queue_.clear();
    active_ = false;
    applied_generation_ = generation_.load(std::memory_order_relaxed);

    trans_.pos = 0.f;
    trans_.vel = 0.f;
    trans_.acc = 0.f;
    rot_.pos = 0.f;
    rot_.vel = 0.f;
    rot_.acc = 0.f;
    planned_vel_[0] = 0.f;
    planned_vel_[1] = 0.f;
}

SegmentResult PlanProfile::straight(float v_end, float distance) {
    return pushSegment(AxisId::translation, v_end, distance,
                       config::profile_limit::MAX_ACCEL_X, config::profile_limit::MAX_DECEL_X);
}

SegmentResult PlanProfile::turn(float omega_end, float angle) {
    return pushSegment(AxisId::rotation, omega_end, angle,
                       config::profile_limit::MAX_ALPHA, config::profile_limit::MAX_ALPHA_DECEL);
}

SegmentResult PlanProfile::setVelocityX(float velocity_x) {
    auto axis = AxisId::translation;
    SegmentResult r = push({EntryKind::setVelocity, axis, velocity_x, 0.f, generation_.load(std::memory_order_relaxed)});
    if (r == SegmentResult::ok) planned_vel_[static_cast<int>(axis)] = velocity_x;
    return r;
}

void PlanProfile::stop() {
    planned_vel_[0] = 0.f;
    planned_vel_[1] = 0.f;
    brake_decel_ = 0.f;
    // 世代を進める：ISRは次のtickで実行中の区間を打ち切り，古い世代の区間を捨てる
    generation_.fetch_add(1, std::memory_order_release);
}

void PlanProfile::brake(float decel) {
    planned_vel_[0] = 0.f;
    planned_vel_[1] = 0.f;
    brake_decel_ = (decel > 0.f) ? decel : 0.f;
    generation_.fetch_add(1, std::memory_order_release);
}

bool PlanProfile::isIdle() const {
    // キューが空かを先に読む：空なら，最後の区間はISRが取り出し済みでactive_に反映されている
    // （ISRはメイン側に割り込むがメイン側はISRに割り込まないので，取り出しとactive_の更新は一度に見える）
    bool empty = queue_.empty();
    return empty && !active_ && applied_generation_ == generation_.load(std::memory_order_acquire);
}

void PlanProfile::waitUntilIdle() const {
    while (!isIdle()) {
        // wait
    }
}

// 初速は計画上の速度（直前に積んだ区間の終速）として検査し，通れば積む
SegmentResult PlanProfile::pushSegment(AxisId axis, float v_end, float distance, float max_accel, float max_decel) {
    int i = static_cast<int>(axis);
    SegmentResult r = validateSegment(planned_vel_[i], v_end, distance, max_accel, max_decel);
    if (r != SegmentResult::ok) {
        ++rejected_count_;
        return r;
    }
    r = push({EntryKind::segment, axis, v_end, distance, generation_.load(std::memory_order_relaxed)});
    if (r == SegmentResult::ok) planned_vel_[i] = v_end;
    return r;
}

SegmentResult PlanProfile::push(const Entry& entry) {
    if (!queue_.push(entry)) {
        ++rejected_count_;
        return SegmentResult::queueFull;
    }
    return SegmentResult::ok;
}

// ---- 割り込み側 ----

void PlanProfile::update() {
    applyStop();
    if (!active_) startNextEntry();
    advance(config::control::DT_S);
}

// stop()・brake()が世代を進めていたら，実行中の区間を打ち切って目標速度・加速度を0にする（目標位置はその場で保持）。
// brake()なら並進だけは今の目標速度から0へ減速する区間を始める
void PlanProfile::applyStop() {
    uint32_t generation = generation_.load(std::memory_order_acquire);
    if (generation == applied_generation_) return;
    applied_generation_ = generation;
    active_ = false;
    float decel = brake_decel_;
    float v0 = trans_.vel;
    trans_.vel = 0.f;
    trans_.acc = 0.f;
    rot_.vel = 0.f;
    rot_.acc = 0.f;
    if (decel > 0.f && v0 != 0.f) {
        float d = v0 * (v0 > 0.f ? v0 : -v0) / (2.f * decel);   // 符号は進む向き
        float T = 2.f * d / v0;
        if (T >= config::control::DT_S) {
            trans_.vel = v0;
            segment_.axis = &trans_;
            segment_.t = 0.f;
            segment_.x0 = trans_.pos;
            segment_.v0 = v0;
            segment_.a = -v0 / T;
            segment_.T = T;
            segment_.x_end = trans_.pos + d;
            segment_.v_end = 0.f;
            active_ = true;
        }
    }
}

// キューから次の要素を取り出す。速度のステップはその場で反映して続けて取り出し，
// 区間が見つかったら今の目標値（位置・速度）を始点・初速として始める
void PlanProfile::startNextEntry() {
    Entry e;
    while (queue_.pop(e)) {
        if (e.generation != applied_generation_) continue;   // stop()より前に積まれた（取り消し済み）

        Axis& ax = axisOf(e.axis);
        if (e.kind == EntryKind::setVelocity) {
            ax.vel = e.v_end;
            ax.acc = 0.f;
            continue;
        }

        // 念のための再検査：積んだときの計画上の速度と実際の目標速度は通常一致するが，
        // ずれて向きの条件を満たさなければ（0割り・目標位置の段差になるので）この区間を捨てる
        float v0 = ax.vel;
        float d = e.distance;
        float s = (d > 0.f) ? 1.f : -1.f;
        if (d == 0.f || v0 * s < 0.f || e.v_end * s < 0.f || (v0 == 0.f && e.v_end == 0.f)) {
            dropped_count_ = dropped_count_ + 1;
            continue;
        }

        float T = 2.f * d / (v0 + e.v_end);
        segment_.axis = &ax;
        segment_.t = 0.f;
        segment_.x0 = ax.pos;
        segment_.v0 = v0;
        segment_.a = (e.v_end - v0) / T;
        segment_.T = T;
        segment_.x_end = ax.pos + d;
        segment_.v_end = e.v_end;
        active_ = true;
        return;
    }
}

// 目標軌道を1tick進める。実行中の区間の軸は解析式で位置・速度を計算し，t >= T で終点（x0 + d, v_end）に
// ちょうどそろえて終える。それ以外の軸（と区間外）は今の目標速度のまま進む
void PlanProfile::advance(float dt) {
    Axis* segment_axis = active_ ? segment_.axis : nullptr;

    for (Axis* ax : {&trans_, &rot_}) {
        if (ax != segment_axis) {
            ax->pos = ax->pos + ax->vel * dt;
            ax->acc = 0.f;
            continue;
        }

        segment_.t += dt;
        if (segment_.t >= segment_.T) {
            ax->pos = segment_.x_end;
            ax->vel = segment_.v_end;
            ax->acc = 0.f;
            active_ = false;
        } else {
            float t = segment_.t;
            ax->pos = segment_.x0 + segment_.v0 * t + 0.5f * segment_.a * t * t;
            ax->vel = segment_.v0 + segment_.a * t;
            ax->acc = segment_.a;
        }
    }
}
