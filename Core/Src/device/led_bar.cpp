#include "device/led_bar.hpp"
#include "gpio.h"
#include <cstdio>

uint32_t bit_reverse(uint32_t b) {
   b = (b & 0xffff0000) >> 16 | (b & 0x0000ffff) << 16;
   b = (b & 0xff00ff00) >> 8 | (b & 0x00ff00ff) << 8;
   b = (b & 0xf0f0f0f0) >> 4 | (b & 0x0f0f0f0f) << 4;
   b = (b & 0xcccccccc) >> 2 | (b & 0x33333333) << 2;
   b = (b & 0xaaaaaaaa) >> 1 | (b & 0x55555555) << 1;
   return b;
}

uint8_t bit_reverse(uint8_t b) {
   b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
   b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
   b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
   return b;
}

void LedBar16::set(const uint16_t pattern) {
  pattern_ = pattern;
  apply();
}

void LedBar16::set(const uint16_t val, const LedBarDotMode mode) {
    switch (mode) {
        case LedBarDotMode::normal:
        set(val);
        break;

        case LedBarDotMode::dot16:
        set(0b1 << val);
        break;

        case LedBarDotMode::dot8:
        set(0b11 << (val<<1));
        break;

        case LedBarDotMode::dot4:
        set(0xf << (val<<2));
        break;

        case LedBarDotMode::dot2:
        set(0xff << (val<<3));
        break;

        default:
        set(val);
        break;
  }
}

void LedBar16::set(const float val, const LedBarValMode mode, const float maxabs = 100.f) {
    switch (mode) {
        case LedBarValMode::pmbit8:
        if (val >= maxabs) {
            set(0x00FF);
        } else if (val >= 0) {
            uint8_t ratio = val / maxabs * (float)0xFF;
            set(bit_reverse(ratio));
        } else if (val <= -maxabs) {
            set(0xFF00);
        } else {
            uint8_t ratio = -val / maxabs * (float)0xFF;
            set(ratio << 8);
        }
        break;

        case LedBarValMode::pmlinear8:
        if (val >= maxabs) {
            set(0x00FF);
        } else if (val >= 0) {
            uint8_t ratio = val / maxabs * 8.f;
            set(bit_reverse((uint8_t)((1<<ratio)-1)));
        } else if (val <= -maxabs) {
            set(0xFF00);
        } else {
            uint8_t ratio = -val / maxabs * 8.f;
            set(((1<<ratio)-1)<<8);
        }
        break;

        case LedBarValMode::pbit16:
        if (val <= 0) {
            set(0);
        } else if (val <= maxabs) {
            uint16_t ratio = val / maxabs * (float)0xFFFF;
            set(ratio);
        } else {
            set(0xFFFF);
        }
        break;

        case LedBarValMode::plinear16:
        if (val <= 0) {
            set(0);
        } else if (val <= maxabs) {
            uint16_t ratio = val / maxabs * 16.f;
            set((1<<ratio)-1);
        } else {
            set(0xFFFF);
        }
        break;

        default:
        break;
    }
}

void LedBar16::setBit(uint8_t index, bool on) {
  if (index >= 16) {
    return;
  }

  if (on) {
    pattern_ |= (1 << index);
  } else {
    pattern_ &= ~(1 << index);
  }
  apply();
}

void LedBar16::clear() {
  pattern_ = 0;
  apply();
}

void LedBar16::apply() { dirty_ = true; }

static constexpr uint8_t order[16] = {6,  5,  4,  3,  2,  1, 0, 7,
                                      14, 13, 12, 11, 10, 9, 8, 15};
void LedBar16::update() {
  if (!dirty_)
    return;

  HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);

  for (int8_t cnt = 0; cnt < 16; cnt++) {
    HAL_GPIO_WritePin(SR_SER_GPIO_Port, SR_SER_Pin,
                      (GPIO_PinState)((pattern_ >> order[cnt]) & 1));

    HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);
}
