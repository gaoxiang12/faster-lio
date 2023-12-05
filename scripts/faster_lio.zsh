#!/bin/zsh
CURRENT_DIR="$( cd "$( dirname "${ZSH_SOURCE[0]:-${(%):-%x}}" )" >/dev/null 2>&1 && pwd )"
source ${CURRENT_DIR}/../../../devel/setup.zsh


############# Lidar #############
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 1


############# LIO #############
roslaunch faster_lio mapping_mid360.launch & sleep 1


############# EKF #############
PASSWD="nv"
echo ${PASSWD} | sudo -S chmod 777 /dev/ttyTHS0

roslaunch mavros apm.launch & sleep 3

# RAW_IMU
rosrun mavros mavsys message_interval --id=27 --rate=200
# ATTITUDE
rosrun mavros mavsys message_interval --id=30 --rate=200
# RC_CHANNELS
rosrun mavros mavsys message_interval --id=65 --rate=20
# Ref: https://mavlink.io/en/messages/common.html

roslaunch ekf_quat ekf_quat_lidar.launch

wait