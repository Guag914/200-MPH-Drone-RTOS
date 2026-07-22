@echo off
setlocal enabledelayedexpansion

set "SIM_FLAG=OFF"
set "USER_FLAG=OFF"
set "DEBUG_FLAG=OFF"

set "MODE="
set "DEBUG_ARG="

rem Loop through arguments
for %%A in (%*) do (
    if /I "%%~A"=="sim" set "MODE=sim"
    if /I "%%~A"=="user" set "MODE=user"
    if /I "%%~A"=="--debug" set "DEBUG_ARG=ON"
    if /I "%%~A"=="debug" set "DEBUG_ARG=ON"
)

if "%MODE%"=="sim" (
    set "SIM_FLAG=ON"
) else if "%MODE%"=="user" (
    set "USER_FLAG=ON"
)

rem Validate debug flag constraint
if "%DEBUG_ARG%"=="ON" (
    if "%USER_FLAG%"=="ON" (
        echo Error: --debug flag cannot be used in 'user' mode.
        exit /b 1
    ) else (
        set "DEBUG_FLAG=ON"
    )
)

if not exist "bin" mkdir bin

cmake ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
-DENABLE_SIM_MODE=%SIM_FLAG% ^
-DENABLE_USER_TASKS=%USER_FLAG% ^
-DENABLE_DEBUG=%DEBUG_FLAG% ^
-G Ninja ^
-S . ^
-B cmake-build-debug

if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build cmake-build-debug --clean-first -j10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake ^
-DENABLE_SIM_MODE=%SIM_FLAG% ^
-DENABLE_USER_TASKS=%USER_FLAG% ^
-DENABLE_DEBUG=%DEBUG_FLAG% ^
-G Ninja ^
-S . ^
-B cmake-build-debug-1

if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build cmake-build-debug-1 --target DRONE-RTOS --clean-first -j 10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

if exist "cmake-build-debug-1\DRONE-RTOS.elf" (
    copy /Y "cmake-build-debug-1\DRONE-RTOS.elf" "bin\"
)

echo.
echo ===================================
echo Build complete.
echo ENABLE_SIM_MODE=%SIM_FLAG%
echo ENABLE_USER_TASKS=%USER_FLAG%
echo ENABLE_DEBUG=%DEBUG_FLAG%
echo Binary location: .\bin\DRONE-RTOS.elf
echo ===================================

endlocal