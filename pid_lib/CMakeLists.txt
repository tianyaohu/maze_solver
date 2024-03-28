cmake_minimum_required(VERSION 3.8)
project(pid_lib) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)
 
# define pid_lib library
set(HEADER_FILES include/pid_lib/pid.hpp)
add_library(pid_lib SHARED src/pid.cpp ${HEADER_FILES})
target_link_libraries(pid_lib armadillo) 
ament_target_dependencies(pid_lib
	rclcpp
)

# this line to exports the library
ament_export_targets(pid_lib HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/pid_lib
  DESTINATION include
)

install(TARGETS pid_lib
        EXPORT pid_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(pid_lib)

ament_package()
