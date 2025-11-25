cmake_minimum_required(VERSION 3.5)
project(px4_gz_positioning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(px4_gz_positioning_node src/px4_gz_positioning_node.cpp)

ament_target_dependencies(
    px4_gz_positioning_node 
    rclcpp 
    geometry_msgs
    gz-transport13
    gz-msgs10
    px4_msgs
)

install(TARGETS px4_gz_positioning_node DESTINATION lib/${PROJECT_NAME})

ament_package()
