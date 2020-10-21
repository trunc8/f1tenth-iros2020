# Base image
FROM ros:melodic-robot-bionic

# Update apt repo and pip2, and install python3, pip3
RUN apt-get update --fix-missing && \
    apt-get install -y python-pip \
                       python3-dev \
                       python3-pip

# Install apt dependencies, add your apt dependencies to this list
RUN apt-get install -y git \
                       build-essential \
                       cmake \
                       vim \
                       ros-melodic-ackermann-msgs \
                       ros-melodic-genpy \
                       libeigen3-dev \
                       libarmadillo-dev \
                       python-catkin-tools

#=============================================================================                       
# For f1tenth_gym_ros
RUN apt-get install -y libzmq3-dev \
                       autoconf \
                       libtool \
                       ros-melodic-map-server
                       
RUN cp -r /usr/include/eigen3/Eigen /usr/include

RUN git clone https://github.com/protocolbuffers/protobuf.git && \
    cd protobuf && \
    git checkout tags/v3.8.0 && \
    git submodule update --init --recursive && \
    ./autogen.sh && \
    ./configure && \
    make -j8 && \
    make install && \
    ldconfig && \
    make clean && \
    cd .. && \
    rm -r protobuf

RUN pip install --upgrade pip

RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                zmq \
                pyzmq \
                Pillow \
                gym \
                protobuf==3.8.0 \
                pyyaml \
                llvmlite==0.31.0 \
                numba==0.47.0


# RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN mkdir /f1tenth_gym
COPY ./f1tenth_gym /f1tenth_gym

RUN cd f1tenth_gym && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make

RUN cd f1tenth_gym && \
    cp ./build/sim_requests_pb2.py ./gym/ && \
    pip install -e gym/

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; mkdir -p catkin_ws/src; cd catkin_ws; catkin_make"

RUN mkdir /catkin_ws/src/f1tenth_gym_ros

COPY . /catkin_ws/src/f1tenth_gym_ros
#=============================================================================
# Upgrade pip
# RUN pip install --upgrade pip

# Install pip dependencies, add your pip dependencies to this list
# RUN pip install numpy==1.16.0 \
#                 scipy==1.2.0 \
#                 pyyaml
RUN pip install opencv-python==4.2.0.32

RUN pip3 install numpy \
                 scipy \
                 pyyaml \
                 llvmlite==0.33.0 \
                 numba \
                 matplotlib \
                 rospkg \
                 netifaces

# Creating a catkin workspace
RUN mkdir -p /catkin_ws/src

# Clone or copy over your source code

# Copying
# COPY ./your_package /catkin_ws/src/
COPY . /catkin_ws/src/f1tenth_iros2020/

# Cloning
RUN cd /catkin_ws/src/ && \
    git clone https://github.com/trunc8/obstacle_detector.git

# Building your ROS packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin build; source devel/setup.bash"

# Uncomment set the shell to bash to provide the "source" command
# SHELL ["/bin/bash", "-c"] 

# Setting entry point command that runs when the container is brought up
# CMD source /catkin_ws/devel/setup.bash; roslaunch --wait your_package your_launch.launch
