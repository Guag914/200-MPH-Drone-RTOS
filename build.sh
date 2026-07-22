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
cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-DENABLE_SIM_MODE=$SIM_FLAG \
-DENABLE_USER_TASKS=$USER_FLAG \
-DENABLE_DEBUG=$DEBUG_FLAG \
-G Ninja \
-S . \
-B cmake-build-debug

if [ $? -ne 0 ]; then exit 1; fi
cmake --build cmake-build-debug --clean-first -j10

# Build profile 2
cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-DENABLE_SIM_MODE=$SIM_FLAG \
-DENABLE_USER_TASKS=$USER_FLAG \
-DENABLE_DEBUG=$DEBUG_FLAG \
-G Ninja \
-S . \
-B cmake-build-debug-1

if [ $? -ne 0 ]; then exit 1; fi
cmake --build cmake-build-debug-1 --target DRONE-RTOS --clean-first -j 10

# Post-build copy to bin directory
if [ -f "cmake-build-debug-1/DRONE-RTOS.elf" ]; then
    cp cmake-build-debug-1/DRONE-RTOS.elf bin/
fi

echo ""
echo ""
echo "==================================="
echo "Build complete."
echo "Simulation mode is $SIM_FLAG"
echo "User task mode is $USER_FLAG"
echo "Debug mode is $DEBUG_FLAG"
echo "Binary location: ./bin/DRONE-RTOS.elf"
echo "==================================="