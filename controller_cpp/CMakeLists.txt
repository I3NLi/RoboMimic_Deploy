cmake_minimum_required(VERSION 3.14)
project(controller_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── unitree_sdk2 ───────────────────────────────────────────────────────────────
find_package(unitree_sdk2 REQUIRED)

# ── yaml-cpp ───────────────────────────────────────────────────────────────────
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

# ── ONNX Runtime ───────────────────────────────────────────────────────────────
# Bypass the broken cmake targets file (it points to lib64/ instead of lib/).
# Override at configure time: cmake -DONNXRUNTIME_DIR=/path/to/ort ...
set(ORT_DEFAULT_DIR
    "/home/hiyio/whole_body_tracking/.tmp_pip_src_backup/src_20260215_035345/isaaclab/TAlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0"
)
set(ONNXRUNTIME_DIR "${ORT_DEFAULT_DIR}" CACHE PATH
    "Root of the pre-built onnxruntime package (must contain include/ and lib/)")

# Resolve the .so by globbing so the version number is not hard-coded.
file(GLOB ORT_SO_FILES "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so*")
list(FILTER ORT_SO_FILES EXCLUDE REGEX "\\.so$")   # prefer versioned .so.X.Y.Z
if(NOT ORT_SO_FILES)
    file(GLOB ORT_SO_FILES "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
endif()
list(GET ORT_SO_FILES 0 ORT_SO)
if(NOT ORT_SO)
    message(FATAL_ERROR "libonnxruntime.so not found under ${ONNXRUNTIME_DIR}/lib/")
endif()
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
set(MUJOCO_ROOT
    "/home/hiyio/anaconda3/envs/robomimic/lib/python3.10/site-packages/mujoco"
    CACHE PATH "Root of the MuJoCo Python package or SDK root")
find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}/include")
find_library(MUJOCO_LIB
    NAMES mujoco libmujoco.so.3.3.4 libmujoco.so
    HINTS "${MUJOCO_ROOT}")
if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIB)
    message(STATUS "Found MuJoCo: ${MUJOCO_LIB}")
else()
    message(WARNING "MuJoCo C API not found; mujoco_dds_simulator target will be skipped")
endif()

# ── Executables ────────────────────────────────────────────────────────────────

# Base controller build: no ONNX (pure framework, PassiveMode + FixedPose only)
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
    "${ONNXRUNTIME_DIR}/include"
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
    BUILD_RPATH  "${UNITREE_DDS_RPATH};${ONNXRUNTIME_DIR}/lib"
    INSTALL_RPATH "${UNITREE_DDS_RPATH};${ONNXRUNTIME_DIR}/lib"
)

# pure ONNXRuntime benchmark: no DDS, no robot communication
add_executable(onnx_benchmark src/onnx_benchmark.cpp)
target_include_directories(onnx_benchmark PRIVATE "${ONNXRUNTIME_DIR}/include")
target_link_libraries(onnx_benchmark pthread "${ORT_SO}")
target_compile_options(onnx_benchmark PRIVATE -O2 -Wall -Wextra)
set_target_properties(onnx_benchmark PROPERTIES
    BUILD_RPATH  "${ONNXRUNTIME_DIR}/lib"
    INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
)

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIB)
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
        BUILD_RPATH  "${UNITREE_DDS_RPATH};${MUJOCO_ROOT}"
        INSTALL_RPATH "${UNITREE_DDS_RPATH};${MUJOCO_ROOT}"
    )
endif()

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS robot_controller robot_controller_onnx onnx_benchmark DESTINATION bin)
if(TARGET mujoco_dds_simulator)
    install(TARGETS mujoco_dds_simulator DESTINATION bin)
endif()
