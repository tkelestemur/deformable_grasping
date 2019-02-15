cmake_minimum_required(VERSION 2.8)
project(ur_mujoco)
add_compile_options(-std=c++11)

set(MUJOCO_DIR "$ENV{MUJOCO200_HOME}")

if(MUJOCO_DIR)
  message(STATUS "MUJOCO200_HOME at ${MUJOCO_DIR}")
else()
  message(FATAL_ERROR "Undefined MUJOCO200_HOME env var.")
endif()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
    include
    ${MUJOCO_DIR}/include
)

#add_library(spnav_wrapper src/spnav_wrapper.cpp)
#target_link_libraries(spnav_wrapper spnav X11)

add_executable(env
    src/env.cpp
)

# TODO: Make it cross-compile
#target_link_libraries(basic
#    ${MUJOCO_DIR}/bin/libmujoco200.dylib
#    ${MUJOCO_DIR}/bin/libglfw.3.dylib)

target_link_libraries(env
        ${MUJOCO_DIR}/bin/libmujoco200.so
        ${MUJOCO_DIR}/bin/libglew.so
        ${MUJOCO_DIR}/bin/libglfw.so.3
        GL)


