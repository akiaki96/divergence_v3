#pragma once

#include <cstdint>

// IRセンサーの値を機体の位置（左・前左・前右・右）で読み，壁の有無を判定する。
//
// device_instance.hpp の irL / irFL / irFR / irR の名前は，読み込んでいるピンと食い違っている
// （AdcValue::update() の読み順：irFL ← PA3 SENSOR_L, irR ← PA2 SENSOR_FL, irL ← PA0 SENSOR_FR,
// irFR ← PA1 SENSOR_R）。メニューの操作（menuInputController）がその名前のまま使っているので
// 変数名は変えず，位置との対応はこのクラスの中（wall_sensor.cpp の pick()）だけで持つ。
// [要確認] 実機で Device → IR → Wall check を開き，各センサーを手でふさいで対応を確かめる
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
