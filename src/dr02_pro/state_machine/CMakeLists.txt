get_filename_component(DEEP_ROBOTICS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(DEEP_ROBOTICS_THIRD_PARTY "${DEEP_ROBOTICS_SDK_ROOT}/third_party")

find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(realsense2 QUIET)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/interface
  ${CMAKE_CURRENT_SOURCE_DIR}/state
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${DEEP_ROBOTICS_THIRD_PARTY}/eigen
  ${HDF5_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(dr02_pro_recording STATIC
  src/recording_manager.cpp
)

add_library(dr02_pro_runtime_options STATIC
  src/runtime_options.cpp
)

target_include_directories(dr02_pro_runtime_options
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dr02_pro_runtime_options
  PUBLIC
    dr02_pro_end_effector
)

target_link_libraries(dr02_pro_recording
  PUBLIC
    pthread
    ${HDF5_CXX_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(state_machine main.cpp)

ament_target_dependencies(state_machine
  deep_robotics_common
  rclcpp
  drdds
  std_msgs
)

target_link_libraries(state_machine
  dr02_pro_manipulation
  dr02_pro_end_effector
  dr02_pro_runtime_options
  pthread
  m
  rt
  dl
  dr02_pro_recording
)

if(realsense2_FOUND)
  target_link_libraries(dr02_pro_recording PUBLIC realsense2::realsense2)
  target_compile_definitions(dr02_pro_recording PRIVATE DR02_PRO_HAS_REALSENSE=1)
  message(STATUS "DR02 Pro recording: RealSense support enabled")
else()
  message(STATUS "DR02 Pro recording: RealSense unavailable; --record will keep teleoperation enabled but reject capture")
endif()

if(BUILD_TESTING)
  add_executable(runtime_options_test
    test/runtime_options_test.cpp
  )
  target_link_libraries(runtime_options_test
    dr02_pro_runtime_options
  )
  add_test(NAME runtime_options_test COMMAND runtime_options_test)
endif()

if(BUILD_TESTING AND NOT realsense2_FOUND)
  target_compile_definitions(dr02_pro_recording
    PUBLIC DR02_PRO_RECORDING_TEST_API=1
  )
  add_executable(recording_manager_test
    test/recording_manager_test.cpp
  )
  target_link_libraries(recording_manager_test
    dr02_pro_recording
    ${HDF5_CXX_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${OpenCV_LIBS}
  )
  add_test(NAME recording_manager_test COMMAND recording_manager_test)
endif()

install(TARGETS state_machine
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
