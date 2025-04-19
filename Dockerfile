FROM osrf/ros:noetic-desktop-full

##################
# parameters
##################

# Set IP of Franka control PC
ENV FCI_IP=169.254.67.230

##################
# libfranka build
##################

# Download and build the required franka libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    libfmt-dev \
    ros-noetic-boost-sml
RUN sudo apt-get install -y lsb-release curl
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN sudo apt-get update && \
    sudo apt-get install -y robotpkg-pinocchio && \
    sudo apt install -qqy robotpkg-py3*-pinocchio && \
    sudo apt-get remove "*libfranka*"

ARG VERSION_LIBFRANKA_ARG
RUN git clone --recurse-submodules https://github.com/frankaemika/libfranka --branch ${VERSION_LIBFRANKA_ARG} # only for FR3
WORKDIR /libfranka
RUN mkdir build
WORKDIR /libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
# RUN 

# Make a Debian package and install it
RUN cpack -G DEB
RUN dpkg -i libfranka*.deb

##################
# franka_control build
##################

# Setup ROS catkin workspace
WORKDIR /catkin_ws
RUN mkdir src
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.sh && catkin_init_workspace src
# OK
# Add lines to the bashrc file that source ROS
# RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN apt install -y python3-pip
COPY requirements.txt .
RUN python3 -m pip install -r requirements.txt

ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
# RUN python3 -m pip install panda-python

# build franka controller
# RUN apt install -y ros-noetic-franka-ros

COPY deps/serl_franka_controllers src/serl_franka_controllers
COPY deps/franka_ros src/franka_ros
COPY deps/ros_control src/ros_control
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
RUN source /opt/ros/noetic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

# change /opt/ros/noetic/share/franka_control/config/franka_control_node.yaml 'realtime_config: enforce' to 'ignore'
RUN sed -i 's/realtime_config: enforce/realtime_config: ignore/g' /catkin_ws/src/franka_ros/franka_control/config/franka_control_node.yaml
RUN sed -i 's/publish_rate: 30  # \[Hz\]/publish_rate: 100  # \[Hz\]/g' /catkin_ws/src/franka_ros/franka_control/config/default_controllers.yaml

# COPY entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/bin/bash"]