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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ddsm210_driver::ddsm210_driver
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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
  find_package(ament_cmake_gtest REQUIRED)

  # Original test
  set(TEST_PROJECT_NAME test_${PROJECT_NAME})
  ament_add_gmock(${TEST_PROJECT_NAME} test/test_hardware_interface_ddsm210.cpp)
  target_include_directories(${TEST_PROJECT_NAME} PRIVATE include)
  target_link_libraries(
    ${TEST_PROJECT_NAME}
    ${PROJECT_NAME}
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    ros2_control_test_assets::ros2_control_test_assets
  )

  # Atomic operations and thread safety tests
  set(TEST_NAME test_hardware_interface_lifecycle)
  ament_add_gtest(${TEST_NAME} test/test_hardware_interface_lifecycle.cpp)
  target_include_directories(${TEST_NAME} PRIVATE include)
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME} rclcpp::rclcpp ddsm210_driver::ddsm210_driver)

  set(TEST_NAME test_hardware_interface_threading)
  ament_add_gtest(${TEST_NAME} test/test_hardware_interface_threading.cpp)
  target_include_directories(${TEST_NAME} PRIVATE include)
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME} rclcpp::rclcpp ddsm210_driver::ddsm210_driver)

  set(TEST_NAME test_hardware_interface_safety)
  ament_add_gtest(${TEST_NAME} test/test_hardware_interface_safety.cpp)
  target_include_directories(${TEST_NAME} PRIVATE include)
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME} rclcpp::rclcpp ddsm210_driver::ddsm210_driver)

  set(TEST_NAME test_hardware_interface_integration)
  ament_add_gtest(${TEST_NAME} test/test_hardware_interface_integration.cpp)
  target_include_directories(${TEST_NAME} PRIVATE include)
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME} rclcpp::rclcpp ddsm210_driver::ddsm210_driver)

  set(TEST_NAME test_hardware_interface_edge_cases)
  ament_add_gtest(${TEST_NAME} test/test_hardware_interface_edge_cases.cpp)
  target_include_directories(${TEST_NAME} PRIVATE include)
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME} rclcpp::rclcpp ddsm210_driver::ddsm210_driver)
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
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ddsm210_driver
)

ament_package()
