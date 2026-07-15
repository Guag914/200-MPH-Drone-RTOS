#!/bin/bash

cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-G Ninja \
-S . \
-B cmake-build-debug

cmake --build cmake-build-debug -j10

/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --build /Users/akshaygillett/CLionProjects/RTOS/cmake-build-debug-1 --target DRONE-RTOS -j 10