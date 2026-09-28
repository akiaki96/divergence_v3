#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

// 吸引ファンのON/OFF時のバッテリ電圧を記録する（電圧降下からファン電流を推定するための試験）
// 解析: data_analysis2/05_fan/fan_current_estimate.py（手順は fan_current_estimation.md）
void fan_run_010_onenter();
void fan_run_020_onenter();
void fan_run_030_onenter();
void fan_run_040_onenter();
void fan_bringup_onenter();   // 切り分け用: v2と同じ方法で40%を3秒回しレジスタをシリアル出力

// ファンを一定dutyで回し続ける（機体を大きく傾けるか電池低下で停止）
void fan_hold_010_onenter();
void fan_hold_020_onenter();
void fan_hold_030_onenter();
void fan_hold_040_onenter();
