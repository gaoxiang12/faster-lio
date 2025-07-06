# CMakeLists.txt for testing laser_mapping ROS2 refactor
cmake_minimum_required(VERSION 3.5)
project(laser_mapping_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(glog REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ../include
  ${PCL_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(laser_mapping SHARED
  laser_mapping.cc
)
ament_target_dependencies(laser_mapping rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen Eigen3)
target_link_libraries(laser_mapping ${PCL_LIBRARIES} ${glog_LIBRARIES} Eigen3::Eigen)

add_executable(laser_mapping_node laser_mapping.cc)
target_link_libraries(laser_mapping_node laser_mapping ${PCL_LIBRARIES} ${glog_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(laser_mapping_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen Eigen3)

install(TARGETS laser_mapping laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
