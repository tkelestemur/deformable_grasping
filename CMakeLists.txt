cmake_minimum_required(VERSION 3.5)
project(ur_mujoco)
add_compile_options(-std=c++11)

#set(USE_GL 1)

set(MUJOCO_DIR "$ENV{MUJOCO200_HOME}")

if(MUJOCO_DIR)
  message(STATUS "MUJOCO200_HOME at ${MUJOCO_DIR}")
else()
  message(FATAL_ERROR "Undefined MUJOCO200_HOME env var.")
endif()


include_directories(
    ${MUJOCO_DIR}/include
#    /home/tarik/.mujoco/mujoco200_linux/include
)

add_executable(basic
    src/basic.cpp
)

target_link_libraries(basic
    ${MUJOCO_DIR}/bin/libmujoco200.so
    ${MUJOCO_DIR}/bin/libglew.so
    ${MUJOCO_DIR}/bin/libglfw.so.3
    GL)



