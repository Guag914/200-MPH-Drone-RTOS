cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

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

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DRONE-RTOS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        # Add user defined library search paths
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
        src/flight/Helpers.cpp
        src/flight/Helpers.cpp
        src/rtos/Logger.cpp
        src/rtos/Logger.h
        src/sim/SimInjector.cpp
        src/sim/SimInjector.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        src/rtos/Main.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
