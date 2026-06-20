extern "C" {
#include "usart.h"
}

extern "C" int __io_putchar(int ch)
{
    uint8_t c = static_cast<uint8_t>(ch);

    HAL_UART_Transmit(
        &huart1,
        &c,
        1,
        HAL_MAX_DELAY);

    return ch;
}