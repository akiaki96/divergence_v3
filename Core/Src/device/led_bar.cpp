#include "device/led_bar.hpp"
#include "gpio.h"

void LedBar16::set(uint16_t pattern) {
    pattern_ = pattern;
    apply();
}

void LedBar16::setBit(uint8_t index, bool on) {
    if (index >= 16) {
        return;
    }

    if (on) {
        pattern_ |= (1<<index);
    } else {
        pattern_ &= ~(1<<index);
    }
    apply();
}

void LedBar16::clear() {
    pattern_ = 0;
    apply();
}

static constexpr uint8_t order[16] = {6,5,4,3,2,1,0,7,14,13,12,11,10,9,8,15};
void LedBar16::apply() {
    HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);

    for (int8_t cnt=0; cnt < 16; cnt++) {
        HAL_GPIO_WritePin(SR_SER_GPIO_Port, SR_SER_Pin, (GPIO_PinState)((pattern_ >> order[cnt]) & 1));

        HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_LATCH_GPIO_Port, SR_LATCH_Pin, GPIO_PIN_RESET);
}