cmake_minimum_required (VERSION 3.5)

project (rslidar_laserscan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/rslidar_laserscan.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
)

add_executable (${PROJECT_NAME}_node
  src/node.cpp src/rslidar_laserscan.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

