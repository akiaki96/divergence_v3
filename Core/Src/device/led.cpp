#include "device/led.hpp"

Led::Led(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {
}

void Led::on() {
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}
void Led::off() {
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}
void Led::toggle() {
    HAL_GPIO_TogglePin(port_, pin_);
}