cmake_minimum_required(VERSION 2.8)
project(ur_mujoco)
add_compile_options(-std=c++11)

set(MUJOCO_DIR "$ENV{MUJOCO200_HOME}")

if (MUJOCO_DIR)
    message(STATUS "MUJOCO200_HOME at ${MUJOCO_DIR}")
else ()
    message(FATAL_ERROR "Undefined MUJOCO200_HOME env var.")
endif ()

find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${MUJOCO_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

add_library(mujoco_wrapper src/mujoco_wrapper.cpp)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)

add_executable(collect_data src/collect_data.cpp)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(mujoco_wrapper
            ${MUJOCO_DIR}/bin/libmujoco200.so
            ${MUJOCO_DIR}/bin/libglew.so
            ${MUJOCO_DIR}/bin/libglfw.so.3
            GL)

    target_link_libraries(collect_data
            ${MUJOCO_DIR}/bin/libmujoco200.so
            ${MUJOCO_DIR}/bin/libglew.so
            ${MUJOCO_DIR}/bin/libglfw.so.3
            GL mujoco_wrapper)

    target_link_libraries(keyboard_teleop
            ${MUJOCO_DIR}/bin/libmujoco200.so
            ${MUJOCO_DIR}/bin/libglew.so
            ${MUJOCO_DIR}/bin/libglfw.so.3
            GL mujoco_wrapper)

elseif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(mujoco_wrapper
            ${MUJOCO_DIR}/bin/libmujoco200.dylib
            ${MUJOCO_DIR}/bin/libglfw.3.dylib)

    target_link_libraries(keyboard_teleop
            ${MUJOCO_DIR}/bin/libmujoco200.dylib
            ${MUJOCO_DIR}/bin/libglfw.3.dylib
            mujoco_wrapper)

    target_link_libraries(collect_data
            ${MUJOCO_DIR}/bin/libmujoco200.dylib
            ${MUJOCO_DIR}/bin/libglfw.3.dylib
            mujoco_wrapper)
endif ()



