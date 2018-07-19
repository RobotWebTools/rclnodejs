FROM ubuntu:xenial

ENV GIT_USER_NAME mrbuild
ENV GIT_USER_EMAIL mrbuild@github.com
ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y git wget locales python
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' \
&& apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install prerequisites
RUN  apt update && apt install -y \
     libopencv-dev \
     libasio-dev \
     libeigen3-dev \
     libtinyxml2.6.2v5 \
     libtinyxml2-dev \
     libfreetype6 \
     libgles2-mesa-dev \
     libglu1-mesa-dev \
     libqt5core5a \
     libqt5gui5 \
     libqt5opengl5 \
     libqt5widgets5 \
     libx11-dev \
     libxaw7 \
     libcurl4-openssl-dev \
     python3-argcomplete \
     python3-catkin-pkg-modules \
     python3-empy \
     python3-pip \
     python3-pyparsing \
     python3-setuptools \
     python3-yaml \
     python3-colcon-common-extensions \
     cmake \
     cppcheck

RUN pip3 install -U setuptools

# Configure git
RUN git config --global user.name $GIT_USER_NAME \
    && git config --global user.email $GIT_USER_EMAIL

# Get ROS2 latest package
ENV ROS2_WS=/root
WORKDIR $ROS2_WS

RUN wget https://ci.ros2.org/view/packaging/job/packaging_xenial_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2 \
    && tar xf ros2-package-linux-x86_64.tar.bz2

RUN echo "source $ROS2_WS/ros2-linux/local_setup.bash" >> $HOME/.bashrc

# Install nvm, Node.js and node-gyp
ENV NODE_VERSION v8.11.3
RUN wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash \
    && . $HOME/.nvm/nvm.sh \
    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH
