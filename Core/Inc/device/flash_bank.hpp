#pragma once

#include <cstdint>

// 迷路の保存に使うフラッシュの2面（A面 = セクタ10，B面 = セクタ11，各128KB）。
// STM32F405XX_FLASH.ld の FLASH 領域はこの手前（768KB）までにしてあるので，プログラムとは重ならない。
//
// 消去・書き込みの間はフラッシュから命令を読めないので CPU（割り込みも）が止まる（RM0090 3.5：読みはバスで待たされる）。
// 消去は1面で1〜2s かかるので，モーターを止めてから呼ぶ。
// 書き込みは1語（32bit）で典型 16us・最大 100us（データシート tPROG）。1語ずつなら 1kHz の制御の割り込みは
// その分遅れるだけで落ちない（app/maze_store の journal はこれで走行中に書く）
namespace flash_bank {

inline constexpr uint8_t COUNT = 2;
inline constexpr uint32_t SIZE = 128 * 1024;   // [byte] 1面

// 面の先頭（メモリにマップされているので，そのまま読める）。消した直後は 0xFF
const uint8_t* address(uint8_t bank);

// 面全体を消す（0xFF にする）
bool erase(uint8_t bank);

// offset [byte]（4の倍数）から 32bit ずつ count 語書く。消した後の 0xFF にしか書けない。
// CPU は count 語のあいだ止まり続けるので，走行中は count = 1 で呼ぶ
bool program(uint8_t bank, uint32_t offset, const uint32_t* words, uint32_t count);

} // namespace flash_bank
