cmake_minimum_required(VERSION 3.12.2)
message(STATUS "CMake version: ${CMAKE_VERSION}")

# ------------------------------------------------------------------------------
# Setup project.

# Project with not optional CUDA.
project(mqtt_maeve_cpp LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 99)
set(PROJECT_NAME mqtt_maeve_cpp)
set(MQTT_DEPENDENCIES "")

set(DEBUG OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
  message(STATUS "Build type not specified: Use Release by default")
  set(DEBUG ON)
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Debug mode and logs.
set(DEBUG_LOG_LEVEL 5 CACHE STRING "Logging level")
if(NOT DEBUG_LOG_LEVEL MATCHES "^[0-9]+$")
    message(FATAL_ERROR "DEBUG_LOG_LEVEL must be a number")
endif()

if(DEBUG)
    add_definitions(-DDEBUG=1 -DDEBUG_LOG_LEVEL=${DEBUG_LOG_LEVEL})
    message(STATUS "Enable DEBUG mode with DEBUG_LOG_LEVEL=${DEBUG_LOG_LEVEL}")
else(DEBUG)
    add_definitions(-DDEBUG=0 -DDEBUG_LOG_LEVEL=0)
    message(STATUS "Disable DEBUG mode")
endif(DEBUG)

set(BUILD_TEST    OFF CACHE BOOL "Enable tests generation")
set(BUILD_UTILITY ON CACHE BOOL "Enable utilities generation")

# enable_testing()
# include(CTest)

# ------------------------------------------------------------------------------
# Setup lib_maeve_cpp.

find_program(PYTHON3 python3)
execute_process(COMMAND "${PYTHON3}" -c "import torch;print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE TORCH_CMAKE_PREFIX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "TORCH_CMAKE_PREFIX_PATH: ${TORCH_CMAKE_PREFIX_PATH}")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${TORCH_CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(LIB_POSE
    "TRTPOSE"
    CACHE STRING
    "Choose the pose estimation library: TRTPOSE, OPENPOSE")
set_property(
    CACHE LIB_POSE PROPERTY STRINGS
    TRTPOSE
    OPENPOSE)
add_definitions(-D${LIB_POSE})

set(LIB_CAMERA
    "ZEDCAMERA"
    CACHE STRING
    "Choose the pose estimation library: ZEDCAMERA, REALSENSECAMERA")
set_property(
    CACHE LIB_CAMERA PROPERTY STRINGS
    ZEDCAMERA
    REALSENSECAMERA)
add_definitions(-D${LIB_CAMERA})

set(MQTT_DEPENDENCIES ${MQTT_DEPENDENCIES} maeve)

# ------------------------------------------------------------------------------
# Setup Paho C++ MQTT.
find_package(PahoMqttCpp REQUIRED)
set(PAHO_LIB paho-mqttpp3 paho-mqtt3as)
set(MQTT_DEPENDENCIES ${MQTT_DEPENDENCIES} ${PAHO_LIB})

# ------------------------------------------------------------------------------
# Set the C++ compiler options.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fomit-frame-pointer")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-builtin")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=i686")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic-errors")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-align")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-qual")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdisabled-optimization")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfloat-equal")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfloat-conversion")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-nonliteral")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-security")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-y2k")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wimport")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winit-self")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winline")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winvalid-pch")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-long-long")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmissing-field-initializers")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmissing-format-attribute")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmissing-include-dirs")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpacked")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpointer-arith")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wredundant-decls")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstack-protector")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstrict-aliasing=2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-default")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunreachable-code")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-function")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-label")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-parameter")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-value")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-variable")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wvariadic-macros")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wwrite-strings")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-compare")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")

# ------------------------------------------------------------------------------
# Set the C compiler options.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nostdlib")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nostdinc")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O1")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fomit-frame-pointer")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-builtin")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=i686")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++14")
# set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS} -Wall")
# set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS} -Wpedantic")
# set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS} -pedantic-errors")
# set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS} -Wextra")
# set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS} -Werror")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wconversion")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wcast-align")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wcast-qual")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wdisabled-optimization")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wfloat-equal")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wfloat-conversion")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wformat=2")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wformat-nonliteral")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wformat-security")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wformat-y2k")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wimport")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Winit-self")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Winline")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Winvalid-pch")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-long-long")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-field-initializers")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-format-attribute")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-include-dirs")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpacked")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpointer-arith")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wredundant-decls")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wshadow")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstack-protector")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-aliasing=2")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch-default")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wswitch-enum")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunreachable-code")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-function")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-label")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-parameter")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-value")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-variable")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wvariadic-macros")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wwrite-strings")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-conversion")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wuninitialized")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse4")

# ------------------------------------------------------------------------------
# Add the includes.
include_directories(
    include
)

# ------------------------------------------------------------------------------
# Node executables.
add_subdirectory(submodules)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} # SHARED
    source/core.cpp
)
target_link_libraries(${LIBRARY_NAME} ${MQTT_DEPENDENCIES})

set(PUBS
    pub_kp3d
    pub_kp3d_1080p
    pub_kp3d_720p
    pub_kp3d_frame
    pub_kp3d_loop
    pub_kp3d_rtsp
    pub_kp3d_stream
    pub_kp3d_stream_1080p
    pub_kp3d_stream_720p
    pub_kp3d_folder
    pub_kp3d_folder_1080p
    pub_kp3d_folder_720p
    pub_kp3d_folder_dump
    pub_kp3d_dump
    pub_kp3d_dump_1080p
    pub_kp3d_dump_720p
    pub_kp3d_stream_dump
    pub_kp3d_stream_dump_1080p
    pub_kp3d_stream_dump_720p
    pub_kp3d_stream_only
)

foreach(PUB ${PUBS})
    add_executable(${PUB} source/pub/${PUB}.cpp source/core.cpp)
    target_link_libraries(${PUB} ${LIBRARY_NAME})
endforeach()

if (BUILD_TEST)
    add_subdirectory(test)
endif()

if(BUILD_UTILITY)
    add_subdirectory(utility)
endif()

# ------------------------------------------------------------------------------
# Install directives.
