cmake_minimum_required(VERSION 3.12.2)
message(STATUS "CMake version: ${CMAKE_VERSION}")

# ------------------------------------------------------------------------------
# Setup project.

# Project with not optional CUDA.
project(maeve LANGUAGES CUDA CXX C)
set(CMAKE_CXX_STANDARD 14)
set(DEPENDENCIES "")
set(INCLUDES "")

include(GNUInstallDirs)
set(LIBMAEVE_TARGET_NAME                     ${PROJECT_NAME})
set(LIBMAEVE_TARGETS_EXPORT_NAME             ${PROJECT_NAME}Targets)
set(LIBMAEVE_TARGETS_EXPORT_FILE             ${LIBMAEVE_TARGETS_EXPORT_NAME}.cmake)
set(LIBMAEVE_INSTALL_LIBDIR                  "${CMAKE_INSTALL_LIBDIR}")
set(LIBMAEVE_INCLUDEDIR                      "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set(LIBMAEVE_INSTALL_INCLUDEDIR              "${CMAKE_INSTALL_INCLUDEDIR}")
set(LIBMAEVE_CONFIG_INSTALL_DIR              "${CMAKE_INSTALL_INCLUDEDIR}/cmake/${PROJECT_NAME}" CACHE INTERNAL "")
set(LIBMAEVE_CMAKE_CONFIG_TEMPLATE           "${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in")
set(LIBMAEVE_CMAKE_CONFIG_DIR                "${CMAKE_CURRENT_BINARY_DIR}")
set(LIBMAEVE_VERSION_CONFIG_FILE             "${LIBMAEVE_CMAKE_CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(LIBMAEVE_PROJECT_CONFIG_FILE             "${LIBMAEVE_CMAKE_CONFIG_DIR}/${PROJECT_NAME}Config.cmake")

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

# Tests and examples
set(BUILD_TEST OFF CACHE BOOL "Enable tests generation")
set(BUILD_EXAMPLE ON CACHE BOOL "Enable examples generation")

# Add include all libraries option.
set(LIB_ALL OFF CACHE BOOL "Include all dependencies")
add_definitions(-DLIB_ALL=${LIB_ALL})

# Add pose library option.
set(LIB_POSE
    "TRTPOSE"
    CACHE STRING
    "Choose the pose estimation library: TRTPOSE, OPENPOSE")
set_property(
    CACHE LIB_POSE PROPERTY STRINGS
    TRTPOSE
    OPENPOSE)
add_definitions(-D${LIB_POSE})

# Add camera library option.
set(LIB_CAMERA
    "ZEDCAMERA"
    CACHE STRING
    "Choose the pose estimation library: ZEDCAMERA, REALSENSECAMERA")
set_property(
    CACHE LIB_CAMERA PROPERTY STRINGS
    ZEDCAMERA
    REALSENSECAMERA)
add_definitions(-D${LIB_CAMERA})

# ------------------------------------------------------------------------------
# Setup dependencies.
include(cmake/GPUConfig.cmake)

# Test.
enable_testing()
include(CTest)

# Check if CUDA is available.
include(CheckLanguage)
check_language(CUDA)

# Logs.
if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
    set(DEPENDENCIES ${DEPENDENCIES} spdlog::spdlog)
endif()

set(DEPENDENCIES ${DEPENDENCIES} -lstdc++fs)

# ------------------------------------------------------------------------------
# Setup CUDA.

# Check if a compiler exists.
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    message(STATUS "Cuda compiler has been found: ${CMAKE_CUDA_COMPILER}")
else(CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR "Cuda compiler has not been found")
endif()

# Check host compiler.
if(DEFINED ${CMAKE_CUDA_HOST_COMPILER})
    message(STATUS "Cuda host compiler defined: ${CMAKE_CUDA_HOST_COMPILER}")
endif()

# Check correct compiler id.
if(CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
    message(STATUS "Cuda compiler id match NVIDIA")
    message(STATUS "Compute Capability: ${CMAKE_CUDA_ARCHITECTURES}")
else(CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
    message(FATAL_ERROR "Cuda compiler id doesn't match")
endif()

message(STATUS 
        "CMake setup project succeeds with NVIDIA "
        "v${CMAKE_CUDA_COMPILER_VERSION}")

# ------------------------------------------------------------------------------
# Setup OpenMP.
set(OPENMP ON CACHE BOOL "Use OpenMP library")
add_definitions(-DOPENMP=${OPENMP})
if (OPENMP)
    find_package(OpenMP REQUIRED)
    if (OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler ${OpenMP_CXX_FLAGS}")
        message(STATUS "OpenMP: enabled")
    else (OPENMP_FOUND)
        message(FATAL_ERROR "OpenMP not found")
    endif ()
endif ()

# ------------------------------------------------------------------------------
# Setup json.
option(USE_EXTERNAL_JSON "Use an external JSON library" OFF)
set(DEPENDENCIES ${DEPENDENCIES} nlohmann_json::nlohmann_json)

# ------------------------------------------------------------------------------
# Setup OpenCV.
find_package(OpenCV REQUIRED)
set(INCLUDES ${INCLUDES} ${OpenCV_INCLUDE_DIRS})
set(DEPENDENCIES ${DEPENDENCIES} ${OpenCV_LIBS})
list(APPEND CUDA_NVCC_FLAGS ${OpenCV_COMPUTE_CAPABILITIES})
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV linking library dependecies: ${OpenCV_LIBS}")
message(STATUS "OpenCV compilation compute capability: ${OpenCV_COMPUTE_CAPABILITIES}")
message(STATUS "OpenCV compilation with CUDA version: ${OpenCV_CUDA_VERSION}")
message(STATUS "OpenCV_USE_CUBLAS: ${OpenCV_USE_CUBLAS}")
message(STATUS "OpenCV_USE_CUFFT: ${OpenCV_USE_CUFFT}")
message(STATUS "OpenCV_USE_NVCUVID: ${OpenCV_USE_NVCUVID}")

# ------------------------------------------------------------------------------
# Setup TensorRT.
set(DEPENDENCIES ${DEPENDENCIES} nvinfer)
# set(DEPENDENCIES ${DEPENDENCIES} nvinfer_plugin)
# set(DEPENDENCIES ${DEPENDENCIES} nvcaffe_parser)
# set(DEPENDENCIES ${DEPENDENCIES} nvonnxparser)
# set(DEPENDENCIES ${DEPENDENCIES} jetson-inference)
# set(DEPENDENCIES ${DEPENDENCIES} jetson-utils)
# if(CUDA_VERSION_MAJOR GREATER 9)
#     set(DEPENDENCIES ${DEPENDENCIES} jetson-inference)
#     set(DEPENDENCIES ${DEPENDENCIES} nvonnxparser)
#     if(HAS_OPENCV) 
#         message("-- Linking jetson-inference with OpenCV " ${OpenCV_VERSION})
#         set(DEPENDENCIES ${DEPENDENCIES} jetson-inference)
#         set(DEPENDENCIES ${DEPENDENCIES} opencv_core)
#         set(DEPENDENCIES ${DEPENDENCIES} opencv_calib3d)
#     endif()
# endif()

# ------------------------------------------------------------------------------
# Setup torch and torchvision.
find_program(PYTHON3 python3)
execute_process(COMMAND "${PYTHON3}" -c "import torch;print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE TORCH_CMAKE_PREFIX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "TORCH_CMAKE_PREFIX_PATH: ${TORCH_CMAKE_PREFIX_PATH}")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${TORCH_CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(DEPENDENCIES ${DEPENDENCIES} ${TORCH_LIBRARIES})

execute_process(COMMAND "${PYTHON3}" -c "import torchvision;print(torchvision.__path__[0])"
    OUTPUT_VARIABLE TORCHVISION_CMAKE_PREFIX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "TORCHVISION_CMAKE_PREFIX_PATH: ${TORCHVISION_CMAKE_PREFIX_PATH}")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${TORCHVISION_CMAKE_PREFIX_PATH}")
find_package(TorchVision REQUIRED)
set(DEPENDENCIES ${DEPENDENCIES} TorchVision::TorchVision)

# ------------------------------------------------------------------------------
# Setup lib camera.
function(use_zedcamera)
    option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

    if (NOT LINK_SHARED_ZED AND MSVC)
        message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
    endif()

    find_package(ZED 3 REQUIRED)
    find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

    set(INCLUDES ${INCLUDES} ${CUDA_INCLUDE_DIRS} PARENT_SCOPE)
    set(INCLUDES ${INCLUDES} ${ZED_INCLUDE_DIRS} PARENT_SCOPE)
    if (LINK_SHARED_ZED)
        set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
    else()
        set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
    endif()

    set(DEPENDENCIES ${DEPENDENCIES} ${ZED_LIBS} PARENT_SCOPE)
endfunction()

function(use_realsensecamera)
    # Setup librealsense2.
    find_package(realsense2 REQUIRED)
    set(DEPENDENCIES ${DEPENDENCIES} ${realsense2_LIBRARY})
endfunction()

if(LIB_ALL)
    message(STATUS "LibCamera: All lib enabled")
    use_zedcamera()
    use_realsensecamera()
else(LIB_ALL)
    message(STATUS "LibCamera: ${LIB_CAMERA} enabled")
    if(LIB_CAMERA STREQUAL "ZEDCAMERA")
        use_zedcamera()
    elseif(LIB_CAMERA STREQUAL "REALSENSECAMERA")
        use_realsensecamera()
    else()
        message(FATAL_ERROR "LibCamera not recognised")   
    endif()
endif(LIB_ALL)

# ------------------------------------------------------------------------------
# Setup lib pose estimation.
function(use_openpose)
    # Set gflags
    find_package(gflags)
    set(DEPENDENCIES ${DEPENDENCIES} gflags PARENT_SCOPE)
    
    # Setup openpose.
    find_package(OpenPose REQUIRED)
    set(INCLUDES ${INCLUDES} ${OpenPose_INCLUDE_DIRS} PARENT_SCOPE)
    set(DEPENDENCIES ${DEPENDENCIES} ${OpenPose_LIBS} PARENT_SCOPE)
endfunction()

function(use_trtpose)
    # Setup trt_pose.
    set(DEPENDENCIES ${DEPENDENCIES} trt_pose PARENT_SCOPE)
endfunction()

if(LIB_ALL)
    message(STATUS "LibPose: All lib enabled")
    use_openpose()
    use_trtpose()
else(LIB_ALL)
    message(STATUS "LibPose: ${LIB_POSE} enabled")
    if(LIB_POSE STREQUAL "OPENPOSE")
        use_openpose()
    elseif(LIB_POSE STREQUAL "TRTPOSE")
        use_trtpose()
    else()
        message(FATAL_ERROR "LibPose not recognised")   
    endif()
endif(LIB_ALL)

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

if(LIB_ALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")
else(LIB_ALL)
    if(LIB_CAMERA STREQUAL "ZEDCAMERA")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")
    elseif(LIB_CAMERA STREQUAL "REALSENSECAMERA")
    else()
    endif()
endif(LIB_ALL)

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
# Set the CUDA compiler options.
set(CA ${CMAKE_CUDA_ARCHITECTURES}) # Compute Capability
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_${CA},code=sm_${CA}")
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_${CA},code=compute_${CA}")
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O1")
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O2")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -w")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++14")

# ------------------------------------------------------------------------------
# Add the source files.
set(CXX_SOURCE_FILES
    src/core.cpp
    src/filter/filter.cpp
    src/camera/device.cpp
    src/camera/sensor.cpp
    src/camera/folder/folder.cpp
    src/interpolation/interpolation_3d.cpp
)

set(CXX_ZEDCAMERA_SOURCE_FILES
    src/camera/zed/zed.cpp
)

set(CXX_REALSENSECAMERA_SOURCE_FILES
    src/camera/realsense/realsense.cpp
)

set(CXX_OPENPOSE_SOURCE_FILES
    src/nn/open_pose/open_pose_model.cpp
)

set(CXX_TRTPOSE_SOURCE_FILES
    src/nn/trt_pose/mapping/trt_pose.cpp
    src/nn/trt_pose/mapping/coco.cpp
    src/nn/trt_pose/mapping/parse_objects.cpp
    src/nn/trt_pose/trt_pose_model.cpp
)

if(LIB_ALL)
    set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_ZEDCAMERA_SOURCE_FILES})
    set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_REALSENSECAMERA_SOURCE_FILES})
else(LIB_ALL)
    if(LIB_CAMERA STREQUAL "ZEDCAMERA")
        set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_ZEDCAMERA_SOURCE_FILES})
    elseif(LIB_CAMERA STREQUAL "REALSENSECAMERA")
        set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_REALSENSECAMERA_SOURCE_FILES})
    endif()
endif(LIB_ALL)

if(LIB_ALL)
    set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_OPENPOSE_SOURCE_FILES})
    set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_TRTPOSE_SOURCE_FILES})
else(LIB_ALL)
    if(LIB_POSE STREQUAL "OPENPOSE")
        set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_OPENPOSE_SOURCE_FILES})
    elseif(LIB_POSE STREQUAL "TRTPOSE")
        set(CXX_SOURCE_FILES ${CXX_SOURCE_FILES} ${CXX_TRTPOSE_SOURCE_FILES})
    endif()
endif(LIB_ALL)

set(CUDA_SOURCE_FILES
)

# ------------------------------------------------------------------------------
# Add the library.
set(LIBRARY_NAME ${PROJECT_NAME})

set(SOURCES
    ${CXX_SOURCE_FILES}
    ${CUDA_SOURCE_FILES}
)

add_subdirectory(submodules)
add_library(${LIBRARY_NAME}
    ${SOURCES}
)

target_include_directories(${LIBRARY_NAME} PUBLIC 
    $<BUILD_INTERFACE:${LIBMAEVE_INCLUDEDIR}>
    $<INSTALL_INTERFACE:include>
)

foreach(INCLUDE ${INCLUDES})
    target_include_directories(${LIBRARY_NAME} PUBLIC 
        $<BUILD_INTERFACE:${INCLUDE}>
        $<INSTALL_INTERFACE:include>
    )
endforeach()

target_link_libraries(${LIBRARY_NAME} ${DEPENDENCIES})

# ------------------------------------------------------------------------------
# Add the tests.
if (BUILD_TEST)
    add_subdirectory(test)
endif()

# ------------------------------------------------------------------------------
# Add the examples.
if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

# ------------------------------------------------------------------------------
# Install directives.
install(
    TARGETS ${LIBMAEVE_TARGET_NAME} 
    DESTINATION ${LIBMAEVE_INSTALL_LIBDIR}
    EXPORT ${LIBMAEVE_TARGETS_EXPORT_NAME})
install(
    EXPORT ${LIBMAEVE_TARGETS_EXPORT_NAME}
    FILE ${LIBMAEVE_TARGETS_EXPORT_FILE}
    DESTINATION ${LIBMAEVE_CONFIG_INSTALL_DIR}
)
install(
    DIRECTORY ${LIBMAEVE_INCLUDEDIR} 
    DESTINATION ${LIBMAEVE_INSTALL_INCLUDEDIR}
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${LIBMAEVE_CMAKE_CONFIG_TEMPLATE}
    ${LIBMAEVE_PROJECT_CONFIG_FILE}
    INSTALL_DESTINATION "lib/cmake/example"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
# generate the version file for the config file
write_basic_package_version_file(
    ${LIBMAEVE_VERSION_CONFIG_FILE}
    VERSION "${maeve_VERSION_MAJOR}.${maeve_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
    ${LIBMAEVE_PROJECT_CONFIG_FILE}
    ${LIBMAEVE_VERSION_CONFIG_FILE}
    DESTINATION ${LIBMAEVE_CONFIG_INSTALL_DIR}
)

# generate the export targets for the build tree
# needs to be after the install(TARGETS ) command
export(EXPORT ${LIBMAEVE_TARGETS_EXPORT_NAME}
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${LIBMAEVE_TARGETS_EXPORT_FILE}"
)