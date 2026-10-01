#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

void plan_step_velocity_onenter();
void plan_step_accel_onenter();
void plan_vel2vel_onenter();
void plan_encoder_check_onenter();   // 50mm/sで990mm走り，エンコーダの距離を実際の距離と比べる
void plan_encoder_check_fan_onenter();   // 同上，吸引ファン20%
void plan_fast_2000_onenter();   // ファン20%で0→2000mm/s（180mm）→2000mm/s（360mm）→0（180mm）

void plan_turn_pos430_onenter();
void plan_turn_neg430_onenter();
void plan_turn_step_alpha_pos430_onenter();
