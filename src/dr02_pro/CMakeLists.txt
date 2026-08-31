cmake_minimum_required(VERSION 3.10)

project(dr02_tele)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-fPIC)
add_compile_options(-w)
add_compile_options(-Wno-psabi)

set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

option(BUILD_SIM "Build with simulation support" OFF)
if(BUILD_SIM)
  add_definitions(-DUSE_SIMULATION)
endif()

option(BUILD_DR02_PRO_STATE_MACHINE "Build DR02 Pro state machine" ON)

find_package(ament_cmake REQUIRED)
find_package(deep_robotics_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_DR02_PRO_STATE_MACHINE)
  add_subdirectory(manipulation)
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../end_effector"
    "${CMAKE_CURRENT_BINARY_DIR}/end_effector"
  )
  add_subdirectory(state_machine)
endif()

ament_package()
