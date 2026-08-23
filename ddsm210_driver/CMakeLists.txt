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
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/motors.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Boost::boost
)

set(CLI_PROJECT_NAME ${PROJECT_NAME}_cli)

add_executable(${CLI_PROJECT_NAME}
  src/ddsm210_cli.cpp
)

target_include_directories(${CLI_PROJECT_NAME}
  PRIVATE
    include
)

target_link_libraries(${CLI_PROJECT_NAME}
  PRIVATE
    ${PROJECT_NAME}
    Boost::boost
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${CLI_PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB TEST_SOURCES test/*.cpp)

  ament_add_gmock(${PROJECT_NAME}_test ${TEST_SOURCES})
  target_include_directories(${PROJECT_NAME}_test PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} Boost::boost)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Boost)
ament_package()
