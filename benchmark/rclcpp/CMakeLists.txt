cmake_minimum_required(VERSION 3.5)

project(rclcpp_benchmark)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

function(custom_executable target)
  add_executable(${target} ${target}.cpp)
  ament_target_dependencies(${target}
    "example_interfaces"
    "nav_msgs"
    "rclcpp"
    "rcutils"
    "sensor_msgs"
    "std_msgs"
    "std_srvs")
endfunction()

custom_executable(publisher-endurance-test)
custom_executable(subscription-endurance-test)
custom_executable(publisher-stress-test)
custom_executable(subscription-stress-test)
custom_executable(client-endurance-test)
custom_executable(service-endurance-test)
custom_executable(client-stress-test)
custom_executable(service-stress-test)
