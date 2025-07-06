list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find and include required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)

# glog
find_package(Glog REQUIRED)

# TBB (Intel Threading Building Blocks)
if (CUSTOM_TBB_DIR)
    set(TBB_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB_INCLUDE_DIR})
    link_directories(${TBB_LIBRARY_DIR})
endif ()

# Include directories
include_directories(
    ${GLOG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    include
)

# Declare dependencies for the package
ament_export_dependencies(
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    pcl_conversions
    Eigen3
    PCL
)

# Include directories to be exported
ament_export_include_directories(include)