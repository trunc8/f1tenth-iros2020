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
                       python-catkin-tools \
                       cppad \
                       gfortran \ 
                       wget
                       
#=============================================================================
# Installing IPOPT
RUN mkdir -p /home/docker/to_install/Ipopt

#RUN cd /IPOPT
#RUN wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.tgz
#RUN tar xvf ./Ipopt-3.12.8.tgz
#RUN "cd /IPOPT/Ipopt-3.12.8/ThirdParty/Blas; ./get.Blas;"
#RUN "cd ../Lapack; bash ./get.Lapack;"
#RUN "cd ../Mumps; bash ./get.Mumps;"
#RUN "cd ../Metis; bash ./get.Metis;"
#RUN "cd ../ASL; bash ./get.ASL"

#RUN "cd /IPOPT/Ipopt-3.12.8; mkdir build; cd build; ../configure; make -j4; make install"

#RUN "cd /IPOPT/Ipopt-3.12.8/build; cp -a include/* /usr/include/.; cp -a lib/* /usr/lib/."
# ======== Start IPOPT installation ==================================================== 
# Retrieve and copy all the dependencies needed by Ipopt 
WORKDIR /home/docker/to_install/Ipopt 
RUN wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.tgz 
RUN tar xvf ./Ipopt-3.12.8.tgz 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/ThirdParty/Blas 
RUN ./get.Blas 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/ThirdParty/Lapack 
RUN ./get.Lapack 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/ThirdParty/Mumps 
RUN ./get.Mumps 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/ThirdParty/Metis 
RUN ./get.Metis 

# Configure and compile Ipopt 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/ 
RUN mkdir build 
WORKDIR /home/docker/to_install/Ipopt/Ipopt-3.12.8/build 
RUN ../configure \ 
    && make \ 
    && make install
RUN cd /home/docker/to_install/Ipopt/Ipopt-3.12.8/build
RUN cp -a include/* /usr/include/.
RUN cp -a lib/* /usr/lib/.
#=============================================================================

# Upgrade pip
RUN pip install --upgrade pip

# Install pip dependencies, add your pip dependencies to this list
# RUN pip install numpy==1.16.0 \
#                 scipy==1.2.0 \
#                 pyyaml
RUN pip install opencv-python==4.2.0.32

# numba is weird and might require that numpy is installed first
RUN pip3 install numpy

RUN pip3 install scipy \
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
COPY ./f1tenth_iros2020 /catkin_ws/src/f1tenth_iros2020/

# Cloning
RUN cd /catkin_ws/src/ && \
    git clone https://github.com/trunc8/obstacle_detector.git

# Building your ROS packages
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin build; source devel/setup.bash"

# Uncomment set the shell to bash to provide the "source" command
SHELL ["/bin/bash", "-c"] 

# Setting entry point command that runs when the container is brought up
CMD source /catkin_ws/devel/setup.bash; roslaunch --wait f1tenth_iros2020 agent.launch
