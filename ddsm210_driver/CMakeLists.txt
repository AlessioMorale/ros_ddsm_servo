cmake_minimum_required(VERSION 3.8)
project(ddsm210_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/motors.cpp
)

ament_target_dependencies(${PROJECT_NAME} "ASIO")

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

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

SET(CLI_PROJECT_NAME ${PROJECT_NAME}_cli)

ament_auto_add_executable(
  ${CLI_PROJECT_NAME}
  src/ddsm210_cli.cpp
)

target_include_directories(
  ${CLI_PROJECT_NAME}
  PUBLIC
  include
)

install(
  TARGETS
  ${CLI_PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  file(GLOB TEST_SOURCES test/*.cpp)

  ament_add_gmock(${PROJECT_NAME}_test ${TEST_SOURCES})

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
