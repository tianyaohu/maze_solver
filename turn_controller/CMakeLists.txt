cmake_minimum_required(VERSION 3.8)
project(turn_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ctrl_lib REQUIRED)

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

########### START of turn_controller ###########
add_executable(turn_controller_node
    src/z_turn_controller.cpp)
target_link_libraries(turn_controller_node
    armadillo
)
ament_target_dependencies(turn_controller_node
	rclcpp
	ctrl_lib
	)
########### END of turn_controller ############

install(TARGETS
  turn_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
