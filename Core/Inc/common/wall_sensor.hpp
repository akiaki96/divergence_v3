#pragma once

#include <cstdint>

// IRセンサーの値を機体の位置（左・前左・前右・右）で読み，壁の有無を判定する。
//
// device_instance.hpp の irL / irFL / irFR / irR の名前は機体の位置と一致しない。メニューの操作
// （menuInputController）がその名前のまま使っているので変数名は変えず，位置との対応はこのクラスの中
// （wall_sensor.cpp の pick()）だけで持つ。
//
// AdcValue::update() の読み順（受光ピンの CubeMX の名前，そのとき点けている LED）：
//   irFL ← PA3 SENSOR_L  （IR_L + IR_FL）   irR ← PA2 SENSOR_FL （IR_L + IR_FL）
//   irL  ← PA0 SENSOR_FR （IR_FR + IR_R）   irFR ← PA1 SENSOR_R （IR_FR + IR_R）
// 2026-10-02 の前壁スイープ（feature/wall-distance，tools/WALL_DISTANCE.md）で，前の壁に反応するのは
// irFL と irFR，irL と irR は横と分かった。受光ピンの名前は位置と合わないが，LED の名前は合うとすると
// （irR は左の LED を点けて横の壁を読めている），左 = irR，前左 = irFL，前右 = irFR，右 = irL になる。
// [要確認] 前・横の区別は実測済み。左右は実機で Device → IR → Wall check を開き，手でふさいで確かめる
namespace wall {

enum Position : uint8_t { left, front_left, front_right, right, POSITION_COUNT };

struct Snapshot {
    int16_t value[POSITION_COUNT];   // 機体の位置の順（Position）
};

// 今のセンサーの値（ISRが毎tick更新する値をそのまま読む）
Snapshot read();

// 壁の有無（config::wall の閾値）。前は前左・前右の平均で判定する
bool hasLeft(const Snapshot& s);
bool hasFront(const Snapshot& s);
bool hasRight(const Snapshot& s);

} // namespace wall
