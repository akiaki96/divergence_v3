#!/usr/bin/env bash
set -e

cmake --build --preset Debug

ELF=build/Debug/divergence_v3.elf
BIN=build/Debug/divergence_v3.bin

arm-none-eabi-objcopy -O binary "$ELF" "$BIN"

st-flash write "$BIN" 0x08000000