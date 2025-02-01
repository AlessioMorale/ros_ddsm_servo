cmake_minimum_required(VERSION 3.8)
project(ddsm210_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(ddsm210_driver REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface_ddsm210.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  ddsm210_driver
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
  launch/test.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  set(TEST_PROJECT_NAME test_${PROJECT_NAME})
  ament_add_gmock(${TEST_PROJECT_NAME} test/test_hardware_interface_ddsm210.cpp)
  target_include_directories(${TEST_PROJECT_NAME} PRIVATE include)
  ament_target_dependencies(
    ${TEST_PROJECT_NAME}
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ddsm210_driver
)

ament_package()
