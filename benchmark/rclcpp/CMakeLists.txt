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

include_directories(util)

function(custom_executable target source)
  add_executable(${target} ${source})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
  ament_target_dependencies(${target}
    "example_interfaces"
    "nav_msgs"
    "rclcpp"
    "rcutils"
    "sensor_msgs"
    "std_msgs"
    "std_srvs")
endfunction()

custom_executable(publisher-stress-test topic/publisher-stress-test.cpp)
custom_executable(subscription-stress-test topic/subscription-stress-test.cpp)
custom_executable(client-stress-test service/client-stress-test.cpp)
custom_executable(service-stress-test service/service-stress-test.cpp)
custom_executable(startup-test startup/startup-test.cpp)

ament_package()
