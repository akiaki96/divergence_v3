extern "C" {
#include "usart.h"
}

#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cstdint>

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

/*文字列送信*/
void uart_write(const char *str) {
    HAL_UART_Transmit(
        &huart1, 
        (uint8_t*)str, 
        std::strlen(str), 
        HAL_MAX_DELAY);
}

/*バイナリ送信*/
void uart_write(const uint8_t* data, uint32_t len) {
    // HAL_UART_Transmit only accepts a uint16_t size, so a long buffer
    // (e.g. a full logger dump) must be split into <=UINT16_MAX chunks
    // to avoid silently truncating len when passed through.
    while (len > 0) {
        const uint16_t chunk = static_cast<uint16_t>(
            len > UINT16_MAX ? UINT16_MAX : len
        );

        HAL_UART_Transmit(
            &huart1,
            const_cast<uint8_t*>(data),
            chunk,
            HAL_MAX_DELAY);

        data += chunk;
        len -= chunk;
    }
}

void uart_init() {

}

void uart_write_format(const char* fmt, ...) {
    char buffer[128];

    va_list args;
    va_start(args, fmt);
    vsniprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uart_write(buffer);
}