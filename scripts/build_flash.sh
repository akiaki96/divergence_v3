#!/usr/bin/env bash
set -e

BUILD_DIR=build/Debug

echo "[1] CMake configure"
cmake --preset Debug

echo "[2] CMake build"
cmake --build --preset Debug

ELF=build/Debug/divergence_v3.elf
BIN=build/Debug/divergence_v3.bin

arm-none-eabi-objcopy -O binary "$ELF" "$BIN"

echo "[3] Flashing"
st-flash write "$BIN" 0x08000000