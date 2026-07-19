#!/bin/bash

# Default both flags to OFF
SIM_FLAG="OFF"
USER_FLAG="OFF"

# Evaluate command line argument
if [ "$1" == "sim" ]; then
    SIM_FLAG="ON"
elif [ "$1" == "user" ]; then
    USER_FLAG="ON"
fi

# Build profile 1
cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-DENABLE_SIM_MODE=$SIM_FLAG \
-DENABLE_USER_TASKS=$USER_FLAG \
-G Ninja \
-S . \
-B cmake-build-debug

cmake --build cmake-build-debug -j10

# Build profile 2
cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-DENABLE_SIM_MODE=$SIM_FLAG \
-DENABLE_USER_TASKS=$USER_FLAG \
-G Ninja \
-S . \
-B cmake-build-debug-1

/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake \
--build /Users/akshaygillett/CLionProjects/RTOS/cmake-build-debug-1 \
--target DRONE-RTOS -j 10

echo""
echo""
echo "==================================="
echo "Build complete."
echo "ENABLE_SIM_MODE=$SIM_FLAG"
echo "ENABLE_USER_TASKS=$USER_FLAG"
echo "==================================="