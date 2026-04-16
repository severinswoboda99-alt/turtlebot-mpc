cmake_minimum_required(VERSION 3.8)
project(mpc-tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller std_msgs rclcpp nav_msgs geometry_msgs Eigen3)
target_link_libraries(controller osqp::osqp)
target_link_libraries(controller OsqpEigen::OsqpEigen)

add_executable(path_planner src/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs)
# Link to spline library
target_include_directories(path_planner
  PRIVATE
    include
)

add_executable(traveled_path src/traveled_path.cpp)
ament_target_dependencies(traveled_path rclcpp nav_msgs geometry_msgs)

install(TARGETS
  controller
  path_planner
  traveled_path
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
