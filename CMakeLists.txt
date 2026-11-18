cmake_minimum_required(VERSION 3.22)
project(openarm_teleop_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PORT_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Threads REQUIRED)

# MuJoCo C API. Defaults to the headers + shared library shipped inside the
# project venv's `mujoco` pip wheel; override with -DMUJOCO_ROOT=<dir with include/ and lib>.
set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.venv/lib/python3.12/site-packages/mujoco"
    CACHE PATH "Directory containing include/mujoco/mujoco.h and libmujoco.so*")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS ${MUJOCO_ROOT}/include REQUIRED NO_DEFAULT_PATH)
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so.4.0.1
             HINTS ${MUJOCO_ROOT} ${MUJOCO_ROOT}/lib REQUIRED NO_DEFAULT_PATH)
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

# ── IK (port of openarm_control 0.1.0 / mink) ────────────────────────────────
add_library(openarm_ik
  src/openarm_ik/arm_kinematics.cpp
  src/openarm_ik/se3.cpp
  src/openarm_ik/box_qp.cpp
  src/openarm_ik/ik_solver.cpp
)
target_include_directories(openarm_ik PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openarm_ik PUBLIC port_logging mujoco::mujoco Eigen3::Eigen)
target_compile_options(openarm_ik PRIVATE ${PORT_WARNINGS})

add_executable(ik_kinematics_dump apps/ik_kinematics_dump.cpp)
target_link_libraries(ik_kinematics_dump PRIVATE openarm_ik nlohmann_json::nlohmann_json)

add_executable(ik_check apps/ik_check.cpp)
target_link_libraries(ik_check PRIVATE openarm_ik nlohmann_json::nlohmann_json)
target_compile_options(ik_check PRIVATE ${PORT_WARNINGS})

# ── Tests (compare against the python implementations) ───────────────────────
enable_testing()
set(QUEST_VR_PARITY_PYTHON "${CMAKE_CURRENT_SOURCE_DIR}/../.venv/bin/python"
    CACHE FILEPATH "Python with numpy/scipy used for the receiver parity test")
# Reference stack the robot actually ran: openarm_control 0.1.0 + mink 1.1.
set(IK_REFERENCE_PYTHON "/home/tan/work/ws/dora-openarm-ros2/.venv/bin/python"
    CACHE FILEPATH "Python with openarm_control 0.1.0 + mink used for the IK checks")
set(IK_SCENE_XML "${CMAKE_CURRENT_SOURCE_DIR}/../../openarm_mujoco/v1/scene.xml"
    CACHE FILEPATH "MJCF scene used by the IK (same as the dataflow yaml)")

add_test(NAME parity_with_python
  COMMAND ${QUEST_VR_PARITY_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tests/parity_check.py
          $<TARGET_FILE:quest_replay>)
add_test(NAME ik_step1_kinematics
  COMMAND ${IK_REFERENCE_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_kinematics.py
          $<TARGET_FILE:ik_kinematics_dump> ${IK_SCENE_XML})
foreach(part se3 boxqp replay)
  add_test(NAME ik_${part}
    COMMAND ${IK_REFERENCE_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_ik.py
            ${part} $<TARGET_FILE:ik_check> ${IK_SCENE_XML})
endforeach()
