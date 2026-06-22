#include "device/device_instance.hpp"
#include "tim.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        ledBar16.update();

        encoderLeft.update();
        encoderRight.update();
    }
}