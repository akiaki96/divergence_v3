#include "device/adcValue.hpp"
#include "adc.h"
#include "config/mouse_config.hpp"
#include "tim.h"
#include <algorithm>
#include "device/device_instance.hpp"

void AdcValue::init() {

}

void AdcValue::tim6_wait_us(uint32_t us) {
    uint32_t dest = __HAL_TIM_GET_COUNTER(&htim6) + us;
    while (__HAL_TIM_GET_COUNTER(&htim6) < dest);
    return;
}

void AdcValue::filter() {
    battery.filtered_ = battery.raw_ * 3.3f / 4096.f * (33000.f + 20000.f) / 20000.f;
    irR.filtered_ = std::max(0, irR.raw_on_ - irR.raw_off_);
    irFR.filtered_ = std::max(0, irFR.raw_on_ - irFR.raw_off_);
    irFL.filtered_ = std::max(0, irFL.raw_on_ - irFL.raw_off_);
    irL.filtered_ = std::max(0, irL.raw_on_ - irL.raw_off_);
}

void AdcValue::update() {
    HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);

    irFL.raw_off_ = HAL_ADC_GetValue(&hadc1);
    irR.raw_off_ = HAL_ADC_GetValue(&hadc2);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);

    irL.raw_off_ = HAL_ADC_GetValue(&hadc1);
    irFR.raw_off_ = HAL_ADC_GetValue(&hadc2);

    HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
    tim6_wait_us(50);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);

    irFL.raw_on_ = HAL_ADC_GetValue(&hadc1);
    irR.raw_on_ = HAL_ADC_GetValue(&hadc2);

    HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
    tim6_wait_us(50);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);

    irL.raw_on_ = HAL_ADC_GetValue(&hadc1);
    irFR.raw_on_ = HAL_ADC_GetValue(&hadc2);

    HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    battery.raw_ = HAL_ADC_GetValue(&hadc1);

    AdcValue::filter();
}