
## Should include ROS2 installation
FROM {image}:{tag}

## Required for tooling
RUN pip3 install parse-cmake click docker


##Install ROS1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install ros-{ros1_version}-ros-base -y
RUN rosdep init && rosdep update
RUN apt-get install curl -y


RUN pip3 install -U colcon-common-extensions





CMD /bin/bash
