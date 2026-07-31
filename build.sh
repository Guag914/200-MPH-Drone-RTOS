#!/bin/bash
SIM_FLAG="OFF"
USER_FLAG="OFF"
DEBUG_FLAG="OFF"

# Parse arguments
MODE=""
DEBUG_ARG=""
for arg in "$@"; do
    if [ "$arg" == "sim" ] || [ "$arg" == "user" ]; then
        MODE="$arg"
    elif [ "$arg" == "--debug" ] || [ "$arg" == "debug" ]; then
        DEBUG_ARG="ON"
    fi
done

# Evaluate modes
if [ "$MODE" == "sim" ]; then
    SIM_FLAG="ON"
elif [ "$MODE" == "user" ]; then
    USER_FLAG="ON"
fi

# Enforce debug constraints: Allowed in default or sim mode, blocked in user mode
if [ "$DEBUG_ARG" == "ON" ]; then
    if [ "$USER_FLAG" == "ON" ]; then
        echo "Error: --debug flag cannot be used in 'user' mode."
        exit 1
    else
        DEBUG_FLAG="ON"
    fi
fi

# Ensure bin directory exists
mkdir -p bin

# Build profile 1
cmake --preset drone-rtos-debug \
  -DENABLE_SIM_MODE=$SIM_FLAG \
  -DENABLE_USER_TASKS=$USER_FLAG \
  -DENABLE_DEBUG=$DEBUG_FLAG

if [ $? -ne 0 ]; then exit 1; fi
cmake --build build-drone-rtos --clean-first -j10

# Build profile 2
cmake --preset drone-rtos-debug \
  -DENABLE_SIM_MODE=$SIM_FLAG \
  -DENABLE_USER_TASKS=$USER_FLAG \
  -DENABLE_DEBUG=$DEBUG_FLAG

if [ $? -ne 0 ]; then exit 1; fi
cmake --build build-drone-rtos --target DRONE-RTOS --clean-first -j 10

# Post-build copy to bin directory and generate disassembly
if [ -f "build-drone-rtos/DRONE-RTOS.elf" ]; then
    cp build-drone-rtos/DRONE-RTOS.elf bin/
    arm-none-eabi-objdump -S bin/DRONE-RTOS.elf > bin/output.dis
fi

echo ""
echo ""
echo "==================================="
echo "Build complete."
echo "Simulation mode is $SIM_FLAG"
echo "User task mode is $USER_FLAG"
echo "Debug mode is $DEBUG_FLAG"
echo "Binary location: ./bin/DRONE-RTOS.elf"
echo "Disassembly location: ./bin/output.dis"
echo "==================================="
