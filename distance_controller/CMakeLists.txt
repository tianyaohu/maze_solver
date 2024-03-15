cmake_minimum_required(VERSION 3.8)
project(distance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pid_lib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(PUBLIC include)

########### START of xy_controller ###########
add_executable(xy_control_node
    src/xy_pid_controller.cpp)
target_link_libraries(xy_control_node armadillo) 
ament_target_dependencies(xy_control_node
    ament_cmake
	rclcpp
	pid_lib
	geometry_msgs
	nav_msgs
	tf2
	tf2_geometry_msgs
	)
########### END of xy_controller ############

install(TARGETS
  xy_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()