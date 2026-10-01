#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

void right_set050_onenter();

// 並進の追従性検証用ログ（共通フィールド＋目標速度・実測/目標位置）
void id_init_log_velocity(void);

// 回転の追従性検証用ログ（共通フィールド＋目標速度・目標角速度・目標/実測角度・角度PIの積分項）
void id_init_log_omega(void);

void velocity_step_300_onenter();
void velocity_step_600_onenter();
void velocity_step_900_onenter();
void velocity_step_000_onenter();
