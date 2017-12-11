FROM ubuntu:xenial

ENV GIT_USER_NAME mrbuild
ENV GIT_USER_EMAIL mrbuild@github.com
ENV LANG en_US.UTF-8

RUN apt-get update && apt-get install -y git wget locales python

# Install ROS2 requirements
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv  --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && apt-get install -y build-essential cppcheck cmake libopencv-dev \
    python-empy python3-dev python3-empy python3-nose python3-pip python3-pyparsing python3-setuptools python3-vcstool libtinyxml-dev libeigen3-dev

# dependencies for RViz
RUN apt-get install -y libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev

# Dependencies for testing
RUN apt-get install -y clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify \
    && pip3 install argcomplete flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner

# Dependencies for FastRTPS
RUN apt-get install -y libasio-dev libtinyxml2-dev

# Configure git
RUN git config --global user.name $GIT_USER_NAME \
    && git config --global user.email $GIT_USER_EMAIL

# Get ROS2 latest package
ENV ROS2_WS=/root
WORKDIR $ROS2_WS

RUN wget http://ci.ros2.org/view/packaging/job/packaging_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2 \
    && tar xf ros2-package-linux-x86_64.tar.bz2

RUN echo "source $ROS2_WS/ros2-linux/local_setup.bash" >> $HOME/.bashrc

# Install nvm, Node.js and node-gyp
ENV NODE_VERSION v8.9.3
RUN wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.33.2/install.sh | bash \
    && . $HOME/.nvm/nvm.sh \
    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

ENV PATH /bin/versions/node/$NODE_VERSION/bin:$PATH
