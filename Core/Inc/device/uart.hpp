#pragma once
#include <cstdint>

void uart_init();
void uart_write(const char* str);
void uart_write(const uint8_t* data, uint16_t len);
void uart_write_format(const char* fmt, ...);