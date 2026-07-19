@echo off
setlocal enabledelayedexpansion

set "SIM_FLAG=OFF"
set "USER_FLAG=OFF"

if "%~1"=="sim" (
    set "SIM_FLAG=ON"
) else if "%~1"=="user" (
    set "USER_FLAG=ON"
)

cmake ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
-DENABLE_SIM_MODE=%SIM_FLAG% ^
-DENABLE_USER_TASKS=%USER_FLAG% ^
-G Ninja ^
-S . ^
-B cmake-build-debug

if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build cmake-build-debug -j10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
-DENABLE_SIM_MODE=%SIM_FLAG% ^
-DENABLE_USER_TASKS=%USER_FLAG% ^
-G Ninja ^
-S . ^
-B cmake-build-debug-1

if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build cmake-build-debug-1 --target DRONE-RTOS -j 10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo
echo
echo ===================================
echo Build complete.
echo ENABLE_SIM_MODE=%SIM_FLAG%
echo ENABLE_USER_TASKS=%USER_FLAG%
echo ===================================

endlocal