####################################################################################################################
##                                                   RoveSoPNP                                                    ##
##                                               24.05.00 - Build 001                                             ##
##                                               RoveSoSeniorDesign                                               ##
##                                      Copyright 2025 - All Rights Reserved                                      ##
####################################################################################################################

## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## C++ and CUDA Version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

## Project Name and Version
project(RoveSoPNP VERSION 24.05.00 LANGUAGES C CXX)
## CMake Policies
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress developer warnings")

####################################################################################################################
##                                                    Options                                                     ##
####################################################################################################################

## Enable or Disable Verbose Makefile
option(BUILD_VERBOSE_MODE "Enable Verbose Makefile" OFF)

## Enable or Disable Printing All CMake Variables
option(LIST_ALL_VARS "Print all CMake Variables" OFF)

####################################################################################################################
##                                         Configuration Based on Options                                         ##
####################################################################################################################

message("-- Autonomy System Options:")

## Verbose Makefile
if (BUILD_VERBOSE_MODE)
    message("--   [x]: Verbose Makefile: Enabled")
    set(CMAKE_VERBOSE_MAKEFILE TRUE CACHE BOOL "Verbose output" FORCE)
else()
    message("--   [ ]: Verbose Makefile: Disabled")
endif()

## Print All CMake Variables
if (LIST_ALL_VARS)
    message("--   [x]: Print All CMake Variables: Enabled")
    
else()
    message("--   [ ]: Print All CMake Variables: Disabled")
endif()

####################################################################################################################
##                                           Dependencies and Libraries                                           ##
####################################################################################################################

# Configure BSThreadPool
add_compile_definitions(BS_THREAD_POOL_ENABLE_PAUSE=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_PRIORITY=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK=1)

## Find Python3
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

## Find Threads
find_package(Threads REQUIRED)

## Find Quill
find_package(quill REQUIRED)

## Fine nlohmann JSON.
find_package(nlohmann_json 3.2.0 REQUIRED)

## Find Eigen3.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

## Find OpenCV.
find_package(OpenCV REQUIRED)

## Check if all variables should be listed.
if (LIST_ALL_VARS)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif()

####################################################################################################################
##                                                Build Executable                                                ##
####################################################################################################################

## Search Project Directories for CPP Files
file(GLOB_RECURSE SRC           CONFIGURE_DEPENDS  "src/*.cpp")
set(EXE_NAME "${PROJECT_NAME}")
add_executable(${EXE_NAME} ${SRC})

## Compile Options
if (MSVC)
    target_compile_options(${EXE_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${EXE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

## Link Libraries
set(AUTONOMY_LIBRARIES  Threads::Threads
                        Eigen3::Eigen
                        Python3::Python
                        quill::quill
                        nlohmann_json::nlohmann_json
                        ${OpenCV_LIBS}
)

target_link_libraries(${EXE_NAME} PRIVATE ${AUTONOMY_LIBRARIES})

####################################################################################################################
##                                           Final Build Configuration                                            ##
####################################################################################################################

message("-- Final RoveSoPNP Configuration Summary")
message("--   Executable Name: ${EXE_NAME}")
message("--   Verbose Makefile: ${BUILD_VERBOSE_MODE}")