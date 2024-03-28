cmake_minimum_required(VERSION 3.8)
project(ctrl_lib) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pid_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)
 
# define ctrl_lib library
set(HEADER_FILES include/ctrl_lib/controller.hpp)
add_library(ctrl_lib SHARED src/controller.cpp ${HEADER_FILES})
target_link_libraries(ctrl_lib armadillo) 
ament_target_dependencies(ctrl_lib
	rclcpp
  geometry_msgs
  pid_lib
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/ctrl_lib
  DESTINATION include
)

install(TARGETS ctrl_lib
        EXPORT ctrl_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# this line to exports the library
ament_export_targets(ctrl_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(ctrl_lib)

#To ensure that downstream packages can use ctrl_lib along with all its dependencies, export the dependencies 
ament_export_dependencies(
  rclcpp
  geometry_msgs
  pid_lib
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()
