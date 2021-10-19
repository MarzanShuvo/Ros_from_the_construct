cmake_minimum_required(VERSION 3.5)
project(bb8_cpp_unit_3_services)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Include the hpp files so that you can find them in the cpp in this package
include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

function(custom_executable target)
  add_executable(${target}_node src/${target}.cpp)
  ament_target_dependencies(${target}_node
    "gazebo_msgs"
    "rclcpp"
    "std_msgs"
    "std_srvs"
    "geometry_msgs")
	target_link_libraries(${target}_node move_bb8)
  install(TARGETS ${target}_node
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Adding library of move_bb8 for use of others
add_library(move_bb8 SHARED
  src/move_bb8.cpp
)
ament_target_dependencies(move_bb8 rclcpp std_msgs std_srvs geometry_msgs)
ament_export_libraries(move_bb8)

# Adding Binaries
custom_executable(move_bb8)
custom_executable(square_service_server)
custom_executable(square_service_client)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

#Install launch files.
install(DIRECTORY
 launch
 DESTINATION share/${PROJECT_NAME}/
)