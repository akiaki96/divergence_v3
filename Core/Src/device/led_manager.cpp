#include "device/led_manager.hpp"
#include "gpio.h"

LedManager::LedManager():
    lOut(LED_1_GPIO_Port, LED_1_Pin),
    rOut(LED_6_GPIO_Port, LED_6_Pin),
    lIn(LED_2_GPIO_Port, LED_2_Pin),
    rIn(LED_3_GPIO_Port, LED_3_Pin),
    rFront(LED_5_GPIO_Port, LED_5_Pin),
    lFront(LED_4_GPIO_Port, LED_4_Pin)
{}