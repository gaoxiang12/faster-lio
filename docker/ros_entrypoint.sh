#!/bin/bash
 
set -e


# 设置ros环境
source "/opt/ros/noetic/setup.sh"
source "/root/catkin_ws/devel/setup.sh"

cd /root/catkin_ws/src/faster-lio
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)


# 可以对一些库进行构建和install

echo "================Docker Env Ready================"

cd /root/catkin_ws

exec "$@"