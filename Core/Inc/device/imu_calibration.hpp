#pragma once

// 走る前の IMU（ジャイロ・加速度）の校正。ファンを回して走るなら，先にファンを回し，回転・振動・電源電圧が
// 定常になる config::fan::STEADY_MS だけ待ってから校正する（ファンを回している間のオフセットを測るため）。
// fan_duty が 0 ならファンは回さず，すぐ校正する。校正が終わるまで（約 config::imu::REFFERENCE_NUM ms）待って戻る。
// ファンは回したまま戻る（止めるのは呼び出し側）
void calibrateImuForRun(float fan_duty);
