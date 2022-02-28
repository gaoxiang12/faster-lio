# Copyright (C) 2021, Hu Zhu, SUSTech
# email: zhuhu00@foxmail.com

FROM zhuhu/cuda110-ros-ubuntu20:0.1

# # Build
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3"
# WORKDIR /root/catkin_ws
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"

RUN echo -e "\nexport XDG_RUNTIME_DIR=/usr/lib/" >> /etc/profile
RUN echo -e "\nexport RUNLEVEL=3" >> /etc/profile
RUN source /etc/profile

# # Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

WORKDIR /root/catkin_ws/src