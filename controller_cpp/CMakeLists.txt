cmake_minimum_required(VERSION 3.14)
project(controller_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── unitree_sdk2 ───────────────────────────────────────────────────────────────
find_package(unitree_sdk2 QUIET)
if(unitree_sdk2_FOUND)
    message(STATUS "Found unitree_sdk2")
else()
    message(WARNING "unitree_sdk2 not found; DDS controller targets will be skipped")
endif()

# ── yaml-cpp ───────────────────────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenGL QUIET)
find_package(X11 QUIET)
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
option(ENABLE_ROS2_CAMERA "Build optional ROS2 camera publisher in mujoco_loco_viewer" OFF)
if(ENABLE_ROS2_CAMERA)
    find_package(rclcpp QUIET)
    find_package(sensor_msgs QUIET)
endif()

# ── ONNX Runtime ───────────────────────────────────────────────────────────────
# Bypass the broken cmake targets file (it points to lib64/ instead of lib/).
# Override at configure time: cmake -DONNXRUNTIME_DIR=/path/to/ort ...
set(ORT_DEFAULT_DIR
    "/home/hiyio/whole_body_tracking/.tmp_pip_src_backup/src_20260215_035345/isaaclab/TAlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0"
)
set(ONNXRUNTIME_DIR "${ORT_DEFAULT_DIR}" CACHE PATH
    "Root of the pre-built onnxruntime package (must contain include/ and lib/)")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_DIR}/include" CACHE PATH
    "ONNX Runtime include directory")
set(ONNXRUNTIME_LIB "" CACHE FILEPATH
    "Full path to libonnxruntime.so; leave empty to search under ONNXRUNTIME_DIR/lib")

# Resolve the .so by globbing so the version number is not hard-coded.
if(ONNXRUNTIME_LIB)
    set(ORT_SO "${ONNXRUNTIME_LIB}")
else()
    file(GLOB ORT_SO_FILES "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so*")
    list(FILTER ORT_SO_FILES EXCLUDE REGEX "\\.so$")   # prefer versioned .so.X.Y.Z
    if(NOT ORT_SO_FILES)
        file(GLOB ORT_SO_FILES "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
    endif()
    if(ORT_SO_FILES)
        list(GET ORT_SO_FILES 0 ORT_SO)
    endif()
endif()
if(NOT ORT_SO)
    message(FATAL_ERROR "libonnxruntime.so not found; set ONNXRUNTIME_LIB or ONNXRUNTIME_DIR")
endif()
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "onnxruntime_cxx_api.h not found under ${ONNXRUNTIME_INCLUDE_DIR}")
endif()
get_filename_component(ORT_LIB_DIR "${ORT_SO}" DIRECTORY)
message(STATUS "Found ONNX Runtime: ${ORT_SO}")

# ── RPATH — prefer unitree's bundled CycloneDDS 0.10.2 over any system copy ──
# Without this, ROS Humble's libddsc.so.0 (0.10.5) is picked up at runtime and
# causes an ABI mismatch crash ("free(): invalid pointer").
#
# CMake by default emits DT_RUNPATH which is overridden by LD_LIBRARY_PATH.
# --disable-new-dtags makes the linker emit DT_RPATH instead, which takes
# priority over LD_LIBRARY_PATH and ensures the correct library is found.
set(UNITREE_DDS_RPATH "/usr/local/lib")
add_link_options(-Wl,--disable-new-dtags)

# ── MuJoCo C API ───────────────────────────────────────────────────────────────
set(MUJOCO_ROOT "" CACHE PATH "Optional root of a MuJoCo SDK/package")
if(MUJOCO_ROOT)
    set(MUJOCO_HINTS)
    list(APPEND MUJOCO_HINTS "${MUJOCO_ROOT}")
    find_path(MUJOCO_INCLUDE_DIR
        NAMES mujoco/mujoco.h
        HINTS ${MUJOCO_HINTS}
        NO_DEFAULT_PATH
        PATH_SUFFIXES include)
    find_library(MUJOCO_LIB
        NAMES mujoco libmujoco.so.3.3.4 libmujoco.so
        HINTS ${MUJOCO_HINTS}
        NO_DEFAULT_PATH
        PATH_SUFFIXES lib bin)
    if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIB)
        get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB}" DIRECTORY)
        message(STATUS "Found MuJoCo: ${MUJOCO_LIB}")
    else()
        message(WARNING "MuJoCo C API not found under MUJOCO_ROOT; mujoco_dds_simulator target will be skipped")
    endif()
else()
    message(STATUS "MUJOCO_ROOT not set; mujoco_dds_simulator target will be skipped")
endif()

# ── MagicBot Z1 SDK ────────────────────────────────────────────────────────────
set(MAGICBOT_Z1_SDK_ROOT
    "/home/hiyio/MaigcLab/magicbot-z1_sdk-main"
    CACHE PATH "Root of magicbot-z1_sdk-main")
find_library(MAGICBOT_Z1_SDK_LIB
    NAMES magicbot_z1_sdk
    PATHS "${MAGICBOT_Z1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
    NO_DEFAULT_PATH)
if(MAGICBOT_Z1_SDK_LIB AND EXISTS "${MAGICBOT_Z1_SDK_ROOT}/include/magic_robot.h")
    add_library(magicbot_z1_sdk_native SHARED IMPORTED GLOBAL)
    set_target_properties(magicbot_z1_sdk_native PROPERTIES
        IMPORTED_LOCATION "${MAGICBOT_Z1_SDK_LIB}")
    target_include_directories(magicbot_z1_sdk_native INTERFACE
        "${MAGICBOT_Z1_SDK_ROOT}/include")
    message(STATUS "Found MagicBot Z1 SDK: ${MAGICBOT_Z1_SDK_LIB}")
else()
    message(WARNING "MagicBot Z1 SDK not found; magicbot_z1_loco_onnx target will be skipped")
endif()

# ── Executables ────────────────────────────────────────────────────────────────

# Base controller build: no ONNX (pure framework, PassiveMode + FixedPose only)
if(unitree_sdk2_FOUND)
    add_executable(robot_controller src/robot_controller.cpp)
    target_include_directories(robot_controller PRIVATE
        ${unitree_sdk2_INCLUDE_DIRS}
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(robot_controller unitree_sdk2 pthread)
    target_compile_options(robot_controller PRIVATE -O2 -Wall -Wextra)
    set_target_properties(robot_controller PROPERTIES
        BUILD_RPATH  "${UNITREE_DDS_RPATH}"
        INSTALL_RPATH "${UNITREE_DDS_RPATH}"
    )

    # Full controller build: includes BeyondMimic ONNX policy
    add_executable(robot_controller_onnx src/robot_controller.cpp)
    target_compile_definitions(robot_controller_onnx PRIVATE ENABLE_BEYOND_MIMIC=1)
    target_include_directories(robot_controller_onnx PRIVATE
        ${unitree_sdk2_INCLUDE_DIRS}
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ONNXRUNTIME_INCLUDE_DIR}"
    )
    target_link_libraries(robot_controller_onnx
        unitree_sdk2
        pthread
        yaml-cpp
        ZLIB::ZLIB
        "${ORT_SO}"
    )
    target_compile_options(robot_controller_onnx PRIVATE -O2 -Wall -Wextra)
    set_target_properties(robot_controller_onnx PROPERTIES
        BUILD_RPATH  "${UNITREE_DDS_RPATH};${ORT_LIB_DIR}"
        INSTALL_RPATH "${UNITREE_DDS_RPATH};${ORT_LIB_DIR}"
    )
endif()

# pure ONNXRuntime benchmark: no DDS, no robot communication
add_executable(onnx_benchmark src/onnx_benchmark.cpp)
target_include_directories(onnx_benchmark PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")
target_link_libraries(onnx_benchmark pthread "${ORT_SO}")
target_compile_options(onnx_benchmark PRIVATE -O2 -Wall -Wextra)
set_target_properties(onnx_benchmark PROPERTIES
    BUILD_RPATH  "${ORT_LIB_DIR}"
    INSTALL_RPATH "${ORT_LIB_DIR}"
)

if(TARGET magicbot_z1_sdk_native)
    add_executable(magicbot_z1_loco_onnx
        src/magicbot_z1_loco_onnx.cpp
        src/magicbot_loco_core.cpp
        src/magicbot_loco_sdk_adapter.cpp
    )
    target_include_directories(magicbot_z1_loco_onnx PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ONNXRUNTIME_INCLUDE_DIR}"
    )
    target_link_libraries(magicbot_z1_loco_onnx
        magicbot_z1_sdk_native
        pthread
        yaml-cpp
        ZLIB::ZLIB
        "${ORT_SO}"
    )
    target_compile_options(magicbot_z1_loco_onnx PRIVATE -O2 -Wall -Wextra)
    set_target_properties(magicbot_z1_loco_onnx PROPERTIES
        BUILD_RPATH  "${MAGICBOT_Z1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR};${ORT_LIB_DIR}"
        INSTALL_RPATH "${MAGICBOT_Z1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR};${ORT_LIB_DIR}"
    )
endif()

if(MUJOCO_ROOT AND MUJOCO_INCLUDE_DIR AND MUJOCO_LIB AND unitree_sdk2_FOUND)
    add_executable(mujoco_dds_simulator src/mujoco_dds_simulator.cpp)
    target_include_directories(mujoco_dds_simulator PRIVATE
        ${unitree_sdk2_INCLUDE_DIRS}
        "${MUJOCO_INCLUDE_DIR}"
    )
    target_link_libraries(mujoco_dds_simulator
        unitree_sdk2
        pthread
        yaml-cpp
        "${MUJOCO_LIB}"
    )
    target_compile_options(mujoco_dds_simulator PRIVATE -O2 -Wall -Wextra)
    set_target_properties(mujoco_dds_simulator PROPERTIES
        BUILD_RPATH  "${UNITREE_DDS_RPATH};${MUJOCO_LIB_DIR}"
        INSTALL_RPATH "${UNITREE_DDS_RPATH};${MUJOCO_LIB_DIR}"
    )
endif()

if(MUJOCO_ROOT AND MUJOCO_INCLUDE_DIR AND MUJOCO_LIB)
    add_executable(dual_inference_rate
        src/dual_inference_rate.cpp
        src/magicbot_loco_core.cpp
    )
    target_include_directories(dual_inference_rate PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ONNXRUNTIME_INCLUDE_DIR}"
        "${MUJOCO_INCLUDE_DIR}"
    )
    target_link_libraries(dual_inference_rate
        pthread
        yaml-cpp
        "${ORT_SO}"
        "${MUJOCO_LIB}"
    )
    if(TARGET magicbot_z1_sdk_native)
        target_sources(dual_inference_rate PRIVATE src/magicbot_loco_sdk_adapter.cpp)
        target_compile_definitions(dual_inference_rate PRIVATE ENABLE_MAGICBOT_SDK=1)
        target_link_libraries(dual_inference_rate magicbot_z1_sdk_native)
        target_include_directories(dual_inference_rate PRIVATE
            "${MAGICBOT_Z1_SDK_ROOT}/include"
        )
        set_target_properties(dual_inference_rate PROPERTIES
            BUILD_RPATH  "${ORT_LIB_DIR};${MUJOCO_LIB_DIR};${MAGICBOT_Z1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
            INSTALL_RPATH "${ORT_LIB_DIR};${MUJOCO_LIB_DIR};${MAGICBOT_Z1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
        )
    else()
        set_target_properties(dual_inference_rate PROPERTIES
            BUILD_RPATH  "${ORT_LIB_DIR};${MUJOCO_LIB_DIR}"
            INSTALL_RPATH "${ORT_LIB_DIR};${MUJOCO_LIB_DIR}"
        )
    endif()
    target_compile_options(dual_inference_rate PRIVATE -O2 -Wall -Wextra)
endif()

if(MUJOCO_ROOT AND MUJOCO_INCLUDE_DIR AND MUJOCO_LIB AND OpenGL_FOUND AND X11_FOUND AND OpenCV_FOUND)
    add_executable(mujoco_loco_viewer
        src/mujoco_loco_viewer.cpp
        src/magicbot_loco_core.cpp
    )
    target_include_directories(mujoco_loco_viewer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ONNXRUNTIME_INCLUDE_DIR}"
        "${MUJOCO_INCLUDE_DIR}"
        ${X11_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(mujoco_loco_viewer
        pthread
        yaml-cpp
        ZLIB::ZLIB
        "${ORT_SO}"
        "${MUJOCO_LIB}"
        ${X11_LIBRARIES}
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        OpenGL::GL
    )
    if(ENABLE_ROS2_CAMERA AND rclcpp_FOUND AND sensor_msgs_FOUND)
        target_compile_definitions(mujoco_loco_viewer PRIVATE ENABLE_ROS2_CAMERA=1)
        target_link_libraries(mujoco_loco_viewer
            rclcpp::rclcpp
            sensor_msgs::sensor_msgs_library
        )
        message(STATUS "mujoco_loco_viewer ROS2 camera publisher enabled")
    else()
        message(STATUS "mujoco_loco_viewer ROS2 camera publisher disabled")
    endif()
    target_compile_options(mujoco_loco_viewer PRIVATE -O2 -Wall -Wextra)
    set_target_properties(mujoco_loco_viewer PROPERTIES
        BUILD_RPATH  "${ORT_LIB_DIR};${MUJOCO_LIB_DIR}"
        INSTALL_RPATH "${ORT_LIB_DIR};${MUJOCO_LIB_DIR}"
    )
elseif(MUJOCO_ROOT)
    message(WARNING "mujoco_loco_viewer target skipped; requires MuJoCo C API, OpenGL, X11, and OpenCV")
endif()

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS onnx_benchmark DESTINATION bin)
if(TARGET robot_controller)
    install(TARGETS robot_controller DESTINATION bin)
endif()
if(TARGET robot_controller_onnx)
    install(TARGETS robot_controller_onnx DESTINATION bin)
endif()
if(TARGET magicbot_z1_loco_onnx)
    install(TARGETS magicbot_z1_loco_onnx DESTINATION bin)
endif()
if(TARGET mujoco_dds_simulator)
    install(TARGETS mujoco_dds_simulator DESTINATION bin)
endif()
if(TARGET mujoco_loco_viewer)
    install(TARGETS mujoco_loco_viewer DESTINATION bin)
endif()
if(TARGET dual_inference_rate)
    install(TARGETS dual_inference_rate DESTINATION bin)
endif()
