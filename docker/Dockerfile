FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y

RUN apt-get install python3-pip ros-foxy-xacro -y
RUN pip install pybullet

RUN mkdir -p /home/dev-ws/src

WORKDIR /home/dev-ws/src

RUN git clone https://github.com/packbionics/pybullet_ros
RUN git clone https://github.com/packbionics/jetleg

WORKDIR /home/dev-ws

RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --packages-select jetleg_control jetleg_description"
RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --packages-select pybullet_ros"
