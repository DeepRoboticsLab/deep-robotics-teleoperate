get_filename_component(DEEP_ROBOTICS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(DEEP_ROBOTICS_THIRD_PARTY "${DEEP_ROBOTICS_SDK_ROOT}/third_party")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/interface
  ${CMAKE_CURRENT_SOURCE_DIR}/state
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${DEEP_ROBOTICS_THIRD_PARTY}/eigen
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
  pthread
  m
  rt
  dl
)

install(TARGETS state_machine
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
