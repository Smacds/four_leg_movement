cmake_minimum_required(VERSION 3.8)
project(cuadrupedo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Ejecutable y fuente principal
add_executable(caminar src/caminar.cpp)

ament_target_dependencies(caminar
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS caminar
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

