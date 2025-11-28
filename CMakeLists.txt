####################################################################################################################
##                                               RoveSoPNP Vision                                                 ##
##                                               24.05.00 - Build 002                                             ##
##                                               RoveSoSeniorDesign                                               ##
##                                      Copyright 2025 - All Rights Reserved                                      ##
####################################################################################################################

## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## C++ and CUDA Version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Project Name and Version
project(RoveSoPNP_Vision VERSION 24.05.00 LANGUAGES C CXX)

####################################################################################################################
##                                                    Options                                                     ##
####################################################################################################################

## Enable or Disable Verbose Makefile
option(BUILD_VERBOSE_MODE "Enable Verbose Makefile" OFF)

## Enable or Disable Printing All CMake Variables
option(LIST_ALL_VARS "Print all CMake Variables" OFF)

## Enable building the standalone executable (for testing vision independently)
option(BUILD_STANDALONE "Build the standalone runner executable (main.cpp)" ON)

message("-- Autonomy Vision System Configuration:")
message("--   [INFO] Build Standalone Runner: ${BUILD_STANDALONE}")

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

####################################################################################################################
##                                                Build Library                                                   ##
####################################################################################################################

## 1. Gather Source Files (Exclude main.cpp for the library)
file(GLOB_RECURSE LIB_SRC CONFIGURE_DEPENDS "src/*.cpp")
list(FILTER LIB_SRC EXCLUDE REGEX "src/main.cpp$")

## 2. Define the Library Target
# We do not specify STATIC or SHARED to allow the parent project to decide via BUILD_SHARED_LIBS.
add_library(RoveVision ${LIB_SRC})

## 3. Setup Include Directories
# PUBLIC means both this library AND projects linking to it can see these folders.
target_include_directories(RoveVision PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

## 4. Link Dependencies
# PUBLIC means projects linking RoveVision will also inherit these links.
target_link_libraries(RoveVision PUBLIC
    Threads::Threads
    Eigen3::Eigen
    Python3::Python
    quill::quill
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
)

## 5. Compile Options
if (MSVC)
    target_compile_options(RoveVision PRIVATE /W4 /WX)
else()
    target_compile_options(RoveVision PRIVATE -Wall -Wextra -Wpedantic)
endif()

####################################################################################################################
##                                           Build Standalone Runner                                              ##
####################################################################################################################

if (BUILD_STANDALONE)
    # Define the executable.
    set(EXE_NAME "RoveVisionRunner")
    add_executable(${EXE_NAME} src/main.cpp)

    # Link against our new library.
    target_link_libraries(${EXE_NAME} PRIVATE RoveVision)
    
    # Ensure it finds the headers.
    target_include_directories(${EXE_NAME} PRIVATE src)

    if (MSVC)
        target_compile_options(${EXE_NAME} PRIVATE /W4 /WX)
    else()
        target_compile_options(${EXE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    message("--   [INFO] Vision Runner Executable: ${EXE_NAME}")
endif()

####################################################################################################################
##                                              Final Configuration                                               ##
####################################################################################################################

if (LIST_ALL_VARS)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif()