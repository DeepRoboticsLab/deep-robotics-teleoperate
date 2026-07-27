get_filename_component(DEEP_ROBOTICS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(DEEP_ROBOTICS_THIRD_PARTY "${DEEP_ROBOTICS_SDK_ROOT}/third_party")

set(PXREA_ROOT "${DEEP_ROBOTICS_THIRD_PARTY}/XRoboToolkit")
set(PINOCCHIO_ROOT "${DEEP_ROBOTICS_THIRD_PARTY}/pinocchio")
set(URDFDOM_ROOT "${DEEP_ROBOTICS_THIRD_PARTY}/urdfdom")
set(DR02_PRO_REPOSITORY_ROOT "${DEEP_ROBOTICS_SDK_ROOT}")

set(DR02_PRO_MANIPULATION_SOURCES
  src/dr2_arm_ik.cpp
  src/pico_data_receiver.cpp
  src/pico_teleoperation.cpp
)

# The bundled x86 libpinocchio.a predates the URDF parser's `mimic`
# parameter. Add a tiny ABI bridge only for that legacy static library.
if(BUILD_PLATFORM STREQUAL "x86")
  list(APPEND DR02_PRO_MANIPULATION_SOURCES
    src/pinocchio_urdf_compat.cpp
  )
endif()

add_library(dr02_pro_manipulation STATIC
  ${DR02_PRO_MANIPULATION_SOURCES}
)

target_include_directories(dr02_pro_manipulation
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${DEEP_ROBOTICS_THIRD_PARTY}/eigen
    ${DEEP_ROBOTICS_THIRD_PARTY}/nlohmann
    ${PXREA_ROOT}/include
    ${PINOCCHIO_ROOT}/include
    ${URDFDOM_ROOT}/include
)

set(PXREA_LIB "${PXREA_ROOT}/lib/${BUILD_PLATFORM}/libPXREARobotSDK.so")
set(PINOCCHIO_LIB "${PINOCCHIO_ROOT}/lib/${BUILD_PLATFORM}/libpinocchio.a")
set(PINOCCHIO_PARSERS_LIB "${PINOCCHIO_ROOT}/lib/${BUILD_PLATFORM}/libpinocchio_parsers.so")
set(URDFDOM_MODEL_LIB "${URDFDOM_ROOT}/lib/${BUILD_PLATFORM}/liburdfdom_model.a")
set(TINYXML_LIB "${URDFDOM_ROOT}/lib/${BUILD_PLATFORM}/libtinyxml.a")
set(CONSOLE_BRIDGE_LIB "${URDFDOM_ROOT}/lib/${BUILD_PLATFORM}/libconsole_bridge.a")

target_compile_definitions(dr02_pro_manipulation
  PRIVATE
    DR02_PRO_REPOSITORY_ROOT="${DR02_PRO_REPOSITORY_ROOT}"
    DR02_PRO_PXREA_LIB="${PXREA_LIB}"
)

target_link_libraries(dr02_pro_manipulation
  PUBLIC
    ${PINOCCHIO_LIB}
)

if(EXISTS "${PINOCCHIO_PARSERS_LIB}")
  target_link_libraries(dr02_pro_manipulation PUBLIC ${PINOCCHIO_PARSERS_LIB})
endif()

target_link_libraries(dr02_pro_manipulation
  PUBLIC
    ${URDFDOM_MODEL_LIB}
    ${TINYXML_LIB}
    ${CONSOLE_BRIDGE_LIB}
)

set_target_properties(dr02_pro_manipulation PROPERTIES
  BUILD_RPATH "${PXREA_ROOT}/lib/${BUILD_PLATFORM};${PINOCCHIO_ROOT}/lib/${BUILD_PLATFORM}"
  INSTALL_RPATH "${PXREA_ROOT}/lib/${BUILD_PLATFORM};${PINOCCHIO_ROOT}/lib/${BUILD_PLATFORM}"
)
