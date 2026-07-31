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

rem Build profile 1
cmake --preset drone-rtos-debug ^
  -DENABLE_SIM_MODE=%SIM_FLAG% ^
  -DENABLE_USER_TASKS=%USER_FLAG% ^
  -DENABLE_DEBUG=%DEBUG_FLAG%
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build build-drone-rtos --clean-first -j10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

rem Build profile 2
cmake --preset drone-rtos-debug ^
  -DENABLE_SIM_MODE=%SIM_FLAG% ^
  -DENABLE_USER_TASKS=%USER_FLAG% ^
  -DENABLE_DEBUG=%DEBUG_FLAG%
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

cmake --build build-drone-rtos --target DRONE-RTOS --clean-first -j 10
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

rem Post-build copy to bin directory and generate disassembly
if exist "build-drone-rtos\DRONE-RTOS.elf" (
    copy /Y "build-drone-rtos\DRONE-RTOS.elf" "bin\"
    arm-none-eabi-objdump -S bin\DRONE-RTOS.elf > bin\output.dis
)

echo.
echo ===================================
echo Build complete.
echo ENABLE_SIM_MODE=%SIM_FLAG%
echo ENABLE_USER_TASKS=%USER_FLAG%
echo ENABLE_DEBUG=%DEBUG_FLAG%
echo Binary location: .\bin\DRONE-RTOS.elf
echo Disassembly location: .\bin\output.dis
echo ===================================

endlocal
