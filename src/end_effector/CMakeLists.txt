add_library(dr02_pro_end_effector STATIC
  zx_gripper/zhixing_end_effector.cpp
)

target_include_directories(dr02_pro_end_effector
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(dr02_pro_end_effector
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  add_executable(zhixing_end_effector_test
    test/zhixing_end_effector_test.cpp
  )
  target_link_libraries(zhixing_end_effector_test
    dr02_pro_end_effector
  )
  ament_target_dependencies(zhixing_end_effector_test
    rclcpp
    std_msgs
  )
  add_test(NAME zhixing_end_effector_test COMMAND zhixing_end_effector_test)
  set_tests_properties(zhixing_end_effector_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp"
  )
endif()
