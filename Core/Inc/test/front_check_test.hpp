#pragma once

#include <cstdint>

// 前壁の閾値（config::wall::THRESH_FRONT_LEFT / THRESH_FRONT_RIGHT）を前左・前右それぞれで決める。
// メニュー：Device → IR → Front check → wall / no wall / show / reset
//
// 置き方は探索と同じ（後端を区画の後壁に当てて前へ向ける）。低速で
//   区画の長さ − config::maze::START_MM − config::search::READ_LEAD_MM
// だけ進んで止まり，探索で壁を読む位置（区画境界の READ_LEAD_MM 手前）で前左・前右の値を
// SAMPLE_COUNT 回読む。
//   wall    … 次の区画の向こう側（境界2つ先）に前壁を置く。探索が「前壁あり」と読むべき場面
//   no wall … その前壁を外す（さらに1区画先に壁を置くと，遠い壁が見える悪い場面も測れる）
// 結果は前左・前右ごとに，電源を切るか reset するまで貯める。置き方（左右の位置・向き）を変えて
// 何回か測ると，壁ありの最小と壁なしの最大の中間を閾値の候補として出す。
// 今の閾値での読み落とし（壁ありを壁なし）・読みすぎ（壁なしを壁あり）の回数も出す。
//
// 走った後は機体を持ち上げて置くと（haltByAccZ），ログ front_check/<wall|no_wall>.csv を送ってから結果を出す
enum class FrontCase : uint8_t { wall, no_wall };

void runFrontCheck(FrontCase c);
void showFrontCheck();
void resetFrontCheck();

template <FrontCase C>
void front_check_onenter() {
    runFrontCheck(C);
}
