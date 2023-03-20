cmake_minimum_required(VERSION 3.8)
project(geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(std_msgs1 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point.msg"
)

ament_package()
