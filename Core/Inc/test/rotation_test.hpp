#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

// 回転の制御（config::pid_rotation：角度PI → 角速度PI，どちらもジャイロ）の実機試験
void rot_angle_hold_onenter();   // 静止して角度0を保持（手で回して戻るか・符号の確認）
void rot_pivot_pos90_onenter();  // その場旋回 +90°（左）
void rot_pivot_neg90_onenter();  // その場旋回 −90°（右）
void rot_pivot_pos180_onenter(); // その場旋回 +180°（左）
void rot_pivot_neg180_onenter(); // その場旋回 −180°（右）
