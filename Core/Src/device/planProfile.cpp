#include "device/planProfile.hpp"
#include "config/mouse_config.hpp"

void PlanProfile::reset() {
    setFreeVelocity(trans_, 0.f);
    trans_.pos = 0.f;
    setFreeVelocity(rot_, 0.f);
    rot_.pos = 0.f;
}

void PlanProfile::update() {
    advance(trans_, config::control::DT_S);
    advance(rot_, config::control::DT_S);
}

// 目標軌道を1tick進める（割り込み側）。開始要求があれば今の目標位置をx0として区間を始め，
// 区間中は解析式で位置・速度を計算し，t >= T で終点（x0 + d, v_end）にちょうどそろえて終える
void PlanProfile::advance(Axis& ax, float dt) {
    if (ax.pending) {
        ax.x0 = ax.pos;
        ax.v0 = ax.req_v0;
        ax.a = ax.req_a;
        ax.T = ax.req_T;
        ax.x_end = ax.pos + ax.req_d;
        ax.v_end = ax.req_v_end;
        ax.t = 0.f;
        ax.active = true;
        ax.pending = false;
    }

    if (ax.active) {
        ax.t += dt;
        if (ax.t >= ax.T) {
            ax.pos = ax.x_end;
            ax.vel = ax.v_end;
            ax.acc = 0.f;
            ax.active = false;
        } else {
            ax.pos = ax.x0 + ax.v0 * ax.t + 0.5f * ax.a * ax.t * ax.t;
            ax.vel = ax.v0 + ax.a * ax.t;
            ax.acc = ax.a;
        }
    } else {
        ax.pos += ax.vel * dt;
        ax.acc = 0.f;
    }
}

// 区間の開始を割り込み側へ要求し，終わるまで待つ（メインコンテキスト）
void PlanProfile::runSegment(Axis& ax, float v0, float a, float T, float d, float v_end) {
    ax.req_v0 = v0;
    ax.req_a = a;
    ax.req_T = T;
    ax.req_d = d;
    ax.req_v_end = v_end;
    ax.pending = true;

    while (ax.pending || ax.active) {
        // wait
    }
}

// 区間を打ち切り，目標速度velで等速に進める（区間の終わりを待たない）
void PlanProfile::setFreeVelocity(Axis& ax, float vel) {
    ax.pending = false;
    ax.active = false;
    ax.vel = vel;
    ax.acc = 0.f;
}

// 今の目標速度v0からv_endへ，等加速度でちょうどd進む（T = 2d / (v0 + v_end), a = (v_end − v0) / T）。
// v0・v_endがdと同じ向き（または一方が0）であること。満たさなければ何もしない
void PlanProfile::segment(Axis& ax, float v_end, float d) {
    float v0 = ax.vel;
    if ((v0 + v_end) * d <= 0.f) return;
    float T = 2.f * d / (v0 + v_end);
    runSegment(ax, v0, (v_end - v0) / T, T, d, v_end);
}

void PlanProfile::straight(float v_end, float distance) {
    segment(trans_, v_end, distance);
}

void PlanProfile::turn(float omega_end, float angle) {
    segment(rot_, omega_end, angle);
}

void PlanProfile::setVelocityX(float velocity_x) {
    setFreeVelocity(trans_, velocity_x);
}

void PlanProfile::stop() {
    setFreeVelocity(trans_, 0.f);
    setFreeVelocity(rot_, 0.f);
}
