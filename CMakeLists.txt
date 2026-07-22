cmake_minimum_required(VERSION 3.22)

# Set up build flags BEFORE defining the project
option(ENABLE_SIM_MODE "Enable mock buffers for simulation" OFF)
if(ENABLE_SIM_MODE)
    add_compile_definitions(SIMULATION)
    message(STATUS "Simulation mode engaged for Renode build.")
endif()

option(ENABLE_USER_TASKS "Enable user task compilation" OFF)
if(ENABLE_USER_TASKS)
    add_compile_definitions(USER_TASKS)
    message(STATUS "Enabled user task compilation for testing.")
endif()

option(ENABLE_DEBUG "Enable debug for visual output of values" OFF)
if(ENABLE_DEBUG)
    add_compile_definitions(DEBUG)
    message(STATUS "Enabled debug for simulation testing")
endif()

# Setup language standards and export options before project initialization
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME DRONE-RTOS)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Initialize project (Cross-compiler properties applied here)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

# Executable target definition
add_executable(${CMAKE_PROJECT_NAME}
        src/rtos/PendSV_Handler.S
        src/rtos/Main.cpp
        src/rtos/rtos.h
        src/flight/flight_control.h
        src/flight/Tasks.cpp
        src/flight/BufferPopulation.cpp
        src/flight/BufferPopulation.h
        User_Tasks/user_tasks.cpp
        User_Tasks/user_tasks.h
        src/flight/Helpers.cpp
        src/flight/Helpers.h
        src/rtos/Logger.cpp
        src/rtos/Logger.h
        src/sim/SimInjector.cpp
        src/sim/SimInjector.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
)