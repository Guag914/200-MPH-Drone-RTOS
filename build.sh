#!/bin/bash

cmake \
-DCMAKE_BUILD_TYPE=Debug \
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
-G Ninja \
-S . \
-B cmake-build-debug

cmake --build cmake-build-debug -j10