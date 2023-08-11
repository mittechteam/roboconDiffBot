cmake_minimum_required(VERSION 3.8)
project(gazebo_assets)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  ros_gz_bridge

  # CUSTOM BRIDGES
  rclcpp
  sensor_msgs
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# ============================================================================
# Python modules
ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/gz_assets
)

# ============================================================================


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
