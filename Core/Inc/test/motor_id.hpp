#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

void right_set050_onenter();

// 並進の追従性検証用ログ（共通フィールド＋目標速度・実測/目標位置）
void id_init_log_velocity(void);

void velocity_step_300_onenter();
void velocity_step_600_onenter();
void velocity_step_900_onenter();
void velocity_step_000_onenter();

void omega_ramp_pos430_onenter();
void omega_ramp_neg430_onenter();
void omega_ramp_pos250_onenter();
void omega_ramp_neg250_onenter();
void omega_ramp_pos100_onenter();
void omega_ramp_neg100_onenter();
