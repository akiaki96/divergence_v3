#pragma once

#include <cstdint>

// IRセンサーの値を機体の位置（左・前左・前右・右）で読み，壁の有無を判定する。
//
// device_instance.hpp の irL / irFL / irFR / irR は，名前どおりの位置のセンサー（左・前左・前右・右）。
// 位置との対応はこのクラスの中（wall_sensor.cpp の pick()）で持つ。
//   - 前・横：2026-10-02 の前壁スイープ（feature/wall-distance，tools/WALL_DISTANCE.md）で，前の壁に
//     反応するのは irFL と irFR，横は irL と irR と分かった
//   - 横の左右：irL が左，irR が右（2026-10-03 ユーザー確認）
//   - [要確認] 前の左右（irFL が前左，irFR が前右）は名前からの推定。Device → IR → Wall check で確かめる
// AdcValue::update() が読む受光ピンの CubeMX の名前（irFL ← PA3 SENSOR_L, irR ← PA2 SENSOR_FL,
// irL ← PA0 SENSOR_FR, irFR ← PA1 SENSOR_R）と LED の名前は，位置と合わないので当てにしない
namespace wall {

enum Position : uint8_t { left, front_left, front_right, right, POSITION_COUNT };

struct Snapshot {
    int16_t value[POSITION_COUNT];   // 機体の位置の順（Position）
};

// 今のセンサーの値（ISRが毎tick更新する値をそのまま読む）
Snapshot read();

// 壁の有無（config::wall の閾値）。前は前左・前右のどちらかが自分の閾値を超えたら壁あり
// （閾値が同じなら「大きい方の値 > 閾値」）。斜めに向いていても強く当たる方で読める
bool hasLeft(const Snapshot& s);
bool hasFront(const Snapshot& s);
bool hasFrontLeft(const Snapshot& s);    // 前左だけの判定（Wall check / Front check で閾値を確かめる用）
bool hasFrontRight(const Snapshot& s);
bool hasRight(const Snapshot& s);

} // namespace wall
