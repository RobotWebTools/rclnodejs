cmake_minimum_required(VERSION 3.5)

project(cpp_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable target)
  add_executable(${target} ${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "sensor_msgs"
    "rclcpp"
    "rcutils"
    "std_msgs")
endfunction()

custom_executable(publisher_msg)
custom_executable(subscription_msg)
custom_executable(listener)
custom_executable(add_two_ints_client)
