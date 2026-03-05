cmake_minimum_required(VERSION 3.14)
project(deploy_real_c CXX)

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

# ── Executables ────────────────────────────────────────────────────────────────

# base build: no ONNX (pure framework, PassiveMode + FixedPose only)
add_executable(deploy_real deploy_real.cpp)
target_include_directories(deploy_real PRIVATE ${unitree_sdk2_INCLUDE_DIRS})
target_link_libraries(deploy_real unitree_sdk2 pthread)
target_compile_options(deploy_real PRIVATE -O2 -Wall -Wextra)
set_target_properties(deploy_real PROPERTIES
    BUILD_RPATH  "${UNITREE_DDS_RPATH}"
    INSTALL_RPATH "${UNITREE_DDS_RPATH}"
)

# full build: includes BeyondMimic ONNX policy
add_executable(deploy_real_onnx deploy_real.cpp)
target_compile_definitions(deploy_real_onnx PRIVATE ENABLE_BEYOND_MIMIC=1)
target_include_directories(deploy_real_onnx PRIVATE
    ${unitree_sdk2_INCLUDE_DIRS}
    "${ONNXRUNTIME_DIR}/include"
)
target_link_libraries(deploy_real_onnx
    unitree_sdk2
    pthread
    yaml-cpp
    ZLIB::ZLIB
    "${ORT_SO}"
)
target_compile_options(deploy_real_onnx PRIVATE -O2 -Wall -Wextra)
set_target_properties(deploy_real_onnx PROPERTIES
    BUILD_RPATH  "${UNITREE_DDS_RPATH};${ONNXRUNTIME_DIR}/lib"
    INSTALL_RPATH "${UNITREE_DDS_RPATH};${ONNXRUNTIME_DIR}/lib"
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS deploy_real deploy_real_onnx DESTINATION bin)
