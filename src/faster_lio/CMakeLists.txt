cmake_minimum_required(VERSION 3.5)
project(faster_lio)

cmake_policy(SET CMP0074 NEW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(glog REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(faster_lio_interfaces REQUIRED)
find_package(gflags REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include(cmake/packages.cmake)

# Source files
set(FASTER_LIO_SOURCES
  src/laser_mapping.cc
  src/pointcloud_preprocess.cc
  src/options.cc
  src/utils.cc
)

add_library(faster_lio SHARED ${FASTER_LIO_SOURCES})

target_include_directories(faster_lio PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(faster_lio
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen Eigen3 glog PCL faster_lio_interfaces gflags pcl_conversions
)

target_link_libraries(faster_lio ${PCL_LIBRARIES} glog Eigen3::Eigen)

add_executable(run_mapping_online app/run_mapping_online.cc)
target_link_libraries(run_mapping_online
        faster_lio gflags ${PCL_LIBRARIES} glog Eigen3::Eigen yaml-cpp
        )
ament_target_dependencies(run_mapping_online rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen Eigen3 glog PCL faster_lio_interfaces yaml-cpp)

# Fix later
add_executable(run_mapping_offline app/run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
        faster_lio gflags ${PCL_LIBRARIES} glog Eigen3::Eigen
        )
ament_target_dependencies(run_mapping_offline rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_eigen Eigen3 glog PCL faster_lio_interfaces
  rosbag2_cpp
  rosbag2_storage)

# Install launch, config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Remove faster_lio_node if not built
install(TARGETS faster_lio run_mapping_online
  run_mapping_offline
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
