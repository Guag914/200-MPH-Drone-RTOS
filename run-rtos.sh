#!/bin/bash

# Exit immediately if any command fails
set -e

echo "====================================="
echo "🛠️  Compiling RTOS for ARM Cortex-M7..."
echo "====================================="

# 1. Compile the code using the ARM toolchain
arm-none-eabi-g++ -O0 -g3 -mcpu=cortex-m7 -mthumb -specs=nosys.specs \
    -Wl,--section-start=.isr_vector=0x00000000 \
    -o RTOS.elf src/Main.cpp

echo "✅ Compilation Successful! Created RTOS.elf"
echo ""
echo "====================================="
echo "🚀 Booting QEMU Virtual Hardware..."
echo "====================================="
echo "💡 Reminder: Press 'Ctrl + A' then 'X' to exit QEMU"
echo "-------------------------------------"

# 2. Launch the binary in QEMU
qemu-system-arm -M netduinoplus2 -cpu cortex-m4 -kernel RTOS.elf -nographic