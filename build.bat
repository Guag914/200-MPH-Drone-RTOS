@echo off
echo ===================================
echo [DRONE-RTOS] Building Project...
echo ===================================

cmake ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
-G Ninja ^
-S . ^
-B cmake-build-debug

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] CMake configuration failed!
    exit /b %ERRORLEVEL%
)

cmake --build cmake-build-debug -j10

if %ERRORLEVEL% EQU 0 (
    echo [SUCCESS] Build completed! Your binary is ready.
) else (
    echo [ERROR] Build failed!
)