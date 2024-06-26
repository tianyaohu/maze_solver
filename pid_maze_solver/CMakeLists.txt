cmake_minimum_required(VERSION 3.8)
project(pid_maze_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ctrl_lib REQUIRED)

# find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

########### START of maze_solver ###########
add_executable(naive_maze_solver_node
    src/naive_maze_solver.cpp)
target_link_libraries(naive_maze_solver_node
    armadillo
	${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(naive_maze_solver_node
	rclcpp
	ctrl_lib
	geometry_msgs
	yaml-cpp
	)
########### END of maze_solver ############

# We install the waypoints folder with all the waypoints files
install(
  DIRECTORY
    waypoints
  DESTINATION
    share/${PROJECT_NAME}/
)

install(TARGETS
  naive_maze_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
