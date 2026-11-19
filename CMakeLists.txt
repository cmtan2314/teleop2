cmake_minimum_required(VERSION 3.22)
project(openarm_teleop_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # build/compile_commands.json for IDE IntelliSense

set(PORT_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Threads REQUIRED)

# ── MuJoCo C API ─────────────────────────────────────────────────────────────
# Official prebuilt release, downloaded at configure time for this machine's
# CPU (x86_64 or aarch64). Same version as the python wheel the reference IK
# ran with. To use a local install instead: -DMUJOCO_ROOT=<dir with
# include/mujoco/mujoco.h and lib/libmujoco.so*>.
set(MUJOCO_VERSION 4.0.1)
set(MUJOCO_ROOT "" CACHE PATH "Local MuJoCo install; empty = download release ${MUJOCO_VERSION}")
if(MUJOCO_ROOT)
  set(MUJOCO_DIR ${MUJOCO_ROOT})
  message(STATUS "MuJoCo: using local install ${MUJOCO_DIR}")
else()
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" MUJOCO_CPU)
  if(MUJOCO_CPU MATCHES "^(x86_64|amd64)$")
    set(MUJOCO_ARCH x86_64)
    set(MUJOCO_SHA256 8df1308b2900a4ed6c18717ebbc9d66d53ab3fe7a4da860a7d5b21f94607299d)
  elseif(MUJOCO_CPU MATCHES "^(aarch64|arm64)$")
    set(MUJOCO_ARCH aarch64)
    set(MUJOCO_SHA256 e16dc0ed24d31279bce674bd03b9f8ef75f209d1189befe67b7c3b366978a6b3)
  else()
    message(FATAL_ERROR "No MuJoCo ${MUJOCO_VERSION} linux release for CPU "
                        "'${CMAKE_SYSTEM_PROCESSOR}'; pass -DMUJOCO_ROOT=<local install>")
  endif()
  set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz")
  message(STATUS "MuJoCo: ${MUJOCO_URL}")
  include(FetchContent)
  FetchContent_Declare(mujoco_release
    URL ${MUJOCO_URL}
    URL_HASH SHA256=${MUJOCO_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake-project)  # prebuilt binaries: download + extract only
  FetchContent_MakeAvailable(mujoco_release)
  set(MUJOCO_DIR ${mujoco_release_SOURCE_DIR})
endif()
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so.${MUJOCO_VERSION})
foreach(required ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h ${MUJOCO_LIBRARY})
  if(NOT EXISTS ${required})
    message(FATAL_ERROR "MuJoCo: missing ${required}")
  endif()
endforeach()
add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR})

# ── Shared logger ────────────────────────────────────────────────────────────
add_library(port_logging src/logging.cpp)
target_include_directories(port_logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(port_logging PRIVATE ${PORT_WARNINGS})

# ── Quest receiver + smoothing ───────────────────────────────────────────────
add_library(quest_vr
  src/types.cpp
  src/packet.cpp
  src/udp_receiver.cpp
  src/pose_processor.cpp
  src/one_euro_smoother.cpp
  src/teleop_input.cpp
)
target_include_directories(quest_vr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quest_vr PUBLIC port_logging Eigen3::Eigen nlohmann_json::nlohmann_json
                      Threads::Threads)
target_compile_options(quest_vr PRIVATE ${PORT_WARNINGS})

add_executable(quest_receiver apps/quest_receiver_main.cpp)
target_link_libraries(quest_receiver PRIVATE quest_vr)

add_executable(quest_replay apps/quest_replay.cpp)
target_link_libraries(quest_replay PRIVATE quest_vr)

# ── DAQP 0.7.2 (vendored C QP solver, the one python/mink uses) ─────────────
file(GLOB DAQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/daqp/src/*.c)
add_library(daqp STATIC ${DAQP_SOURCES})
target_include_directories(daqp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/daqp/include)
target_link_libraries(daqp PUBLIC m)
# Third-party code: built as upstream builds it, without our warning flags.

# ── IK (port of openarm_control 0.1.0 / mink) ────────────────────────────────
add_library(openarm_ik
  src/openarm_ik/arm_kinematics.cpp
  src/openarm_ik/se3.cpp
  src/openarm_ik/box_qp.cpp
  src/openarm_ik/ik_solver.cpp
)
target_include_directories(openarm_ik PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openarm_ik PUBLIC port_logging mujoco::mujoco Eigen3::Eigen
                      PRIVATE daqp)
target_compile_options(openarm_ik PRIVATE ${PORT_WARNINGS})

add_executable(ik_kinematics_dump apps/ik_kinematics_dump.cpp)
target_link_libraries(ik_kinematics_dump PRIVATE openarm_ik nlohmann_json::nlohmann_json)

add_executable(ik_check apps/ik_check.cpp)
target_link_libraries(ik_check PRIVATE openarm_ik nlohmann_json::nlohmann_json)
target_compile_options(ik_check PRIVATE ${PORT_WARNINGS})

# ── ROS 2 output stage (port of src/dora_ros2_bridge/main.py) ───────────────
find_package(yaml-cpp REQUIRED)
add_library(teleop_bridge
  src/teleop_bridge/bridge_logic.cpp
  src/teleop_bridge/grasp_presets.cpp
)
target_include_directories(teleop_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(teleop_bridge PUBLIC openarm_ik port_logging Eigen3::Eigen
                      PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(teleop_bridge PRIVATE ${PORT_WARNINGS})

add_executable(bridge_replay apps/bridge_replay.cpp)
target_link_libraries(bridge_replay PRIVATE teleop_bridge nlohmann_json::nlohmann_json)
target_compile_options(bridge_replay PRIVATE ${PORT_WARNINGS})

# The node needs a sourced ROS 2 environment (source /opt/ros/jazzy/setup.bash).
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(control_msgs REQUIRED)
  add_executable(openarm_teleop_node apps/openarm_teleop_node.cpp)
  target_link_libraries(openarm_teleop_node PRIVATE quest_vr openarm_ik teleop_bridge
    rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS}
    ${trajectory_msgs_TARGETS} ${control_msgs_TARGETS})
  target_compile_options(openarm_teleop_node PRIVATE ${PORT_WARNINGS})
  # Default model_xml / grasp_presets parameters point at the copies in config/.
  target_compile_definitions(openarm_teleop_node PRIVATE
    OPENARM_TELEOP_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
else()
  message(STATUS "rclcpp not found: source /opt/ros/jazzy/setup.bash to build openarm_teleop_node")
endif()

# ── Tests (compare against the python implementations) ───────────────────────
enable_testing()
set(IK_SCENE_XML "${CMAKE_CURRENT_SOURCE_DIR}/config/openarm_mujoco/v1/scene.xml"
    CACHE FILEPATH "MJCF scene used by the IK (copy of openarm_mujoco/v1)")
# Every test replays a golden file (tests/data) recorded from the python code
# this port replaces with tests/reference/*.py --record: standard-library
# python only, no venv.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(GOLDEN_CHECK ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_golden.py)
set(GOLDEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)

add_test(NAME parity_with_python
  COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/quest.json.gz $<TARGET_FILE:quest_replay>)
add_test(NAME ik_step1_kinematics
  COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/kinematics.json.gz
          $<TARGET_FILE:ik_kinematics_dump> ${IK_SCENE_XML})
add_test(NAME bridge_parity
  COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/bridge.json.gz $<TARGET_FILE:bridge_replay>
          --grasp-presets ${CMAKE_CURRENT_SOURCE_DIR}/config/grasp_presets.yaml)
add_test(NAME ik_se3
  COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/se3.json.gz $<TARGET_FILE:ik_check> se3)
foreach(backend daqp activeset)
  add_test(NAME ik_boxqp_${backend}
    COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/boxqp.json.gz $<TARGET_FILE:ik_check> boxqp ${backend})
  add_test(NAME ik_replay_${backend}
    COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/replay.json.gz
            $<TARGET_FILE:ik_check> replay ${IK_SCENE_XML} ${backend} single)
endforeach()
add_test(NAME ik_replay_daqp_threads
  COMMAND ${GOLDEN_CHECK} ${GOLDEN_DIR}/replay.json.gz
          $<TARGET_FILE:ik_check> replay ${IK_SCENE_XML} daqp threads)

add_executable(ik_bench apps/ik_bench.cpp)
target_link_libraries(ik_bench PRIVATE openarm_ik)
