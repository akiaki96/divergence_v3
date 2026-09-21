#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

// 吸引ファンのON/OFF時のバッテリ電圧を記録する（電圧降下からファン電流を推定するための試験）
// 解析: data_analysis2/fan_current_estimate.py（手順は fan_current_estimation.md）
void fan_run_025_onenter();
void fan_run_050_onenter();
void fan_run_075_onenter();
void fan_run_100_onenter();
