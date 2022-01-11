FROM ubuntu:focal

ENV GIT_USER_NAME mrbuild
ENV GIT_USER_EMAIL mrbuild@github.com

RUN apt update && apt install -y git locales python curl wget
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt install -y gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install prerequisites
RUN export DEBIAN_FRONTEND=noninteractive && apt update && apt install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-rosdep \
  libssl-dev \
  cppcheck

RUN rosdep init
RUN rosdep update

# Configure git
RUN git config --global user.name $GIT_USER_NAME \
    && git config --global user.email $GIT_USER_EMAIL

# Get ROS2 latest package
RUN mkdir /ros2
RUN chmod 755 /ros2
ENV ROS2_WS=/ros2
WORKDIR $ROS2_WS

RUN wget https://github.com/ros2/ros2/releases/download/release-galactic-20210716/ros2-galactic-20210616-linux-focal-amd64.tar.bz2 \
    && tar xf ros2-galactic-20210616-linux-focal-amd64.tar.bz2

# [Ubuntu 20.04]
RUN rosdep install --from-paths $ROS2_WS/ros2-linux/share --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connextdds rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

RUN echo "source $ROS2_WS/ros2-linux/local_setup.bash" >> $HOME/.bashrc

# Install nvm, Node.js and node-gyp
ENV NODE_VERSION v12.20.0
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.0/install.sh | bash
ENV NODE_VERSION v12.22.1
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash \
    && . $HOME/.nvm/nvm.sh \
    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH

# set ros2 mode 777 only to support copy of compiled test msgs and libs into ros2 installation
RUN chmod 777 ${ROS2_WS}/ros2-linux
RUN chmod 777 ${ROS2_WS}/ros2-linux/include
RUN chmod 777 ${ROS2_WS}/ros2-linux/lib
RUN chmod 777 ${ROS2_WS}/ros2-linux/share

# create normal user
RUN useradd -ms /bin/bash rclnodejs-user
USER rclnodejs-user
WORKDIR /home/rclnodejs-user

RUN echo "source $ROS2_WS/ros2-linux/local_setup.bash" >> $HOME/.bashrc

# Install nvm, Node.js and node-gyp
# v12 lts/erbium
# v14 lts/fermium
# v16 lts/gallium
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
RUN  bash -c "\. .nvm/nvm.sh && nvm install v10.23.1"
RUN  bash -c "\. .nvm/nvm.sh && nvm install lts/erbium"
RUN  bash -c "\. .nvm/nvm.sh && nvm install lts/fermium"
RUN  bash -c "\. .nvm/nvm.sh && nvm install lts/gallium"
