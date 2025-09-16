# Create an image configured with ROS2 including colcon, Nodejs and rclnodejs source 
# Supported ARGS:
#   ROS_DISTRO = [humble, jazzy, kilted, rolling], default=rolling
#   NODE_MAJOR_VER = [20, 22, 24], default=22
#   BRANCH = rclnodejs git branch, default=develop
#
# examples: 
#
# Build image named 'rclnodejs' and run it with the rclnode test suite
#
#    docker build -t rclnodejs  .
#    docker run -it rclnodejs npm test
#
#
# Build an image for a specific branch of rclnodejs, version of ROS2 and Nodejs use:
#
#     docker build -t <image_name> --build-arg ROS_DISTRO=kilted .
#     docker build -t <image_name>  \
#        --build-arg ROS_DISTRO=jazzy \
#        --build-arg BRANCH=develop \
#        --build-arg NODE_MAJOR_VER=22 .
#
#
# Build and run:
#     docker run -it --rm $(docker build -q .)
#

# use 
ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}

# Install dependencies, including Nodejs
ARG NODE_MAJOR_VER=22
RUN apt-get update -y \
    && apt-get install -y curl sudo \
    && curl -fsSL https://deb.nodesource.com/setup_${NODE_MAJOR_VER}.x | sudo -E bash - \
    && apt-get install -y nodejs

# clone a branch of the rclnodejs repo, build addon libs, generate corresponding JS msgs
ARG BRANCH=develop
WORKDIR /rosdev
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get install -y ros-${ROS_DISTRO}-test-msgs \
    && apt-get install -y ros-${ROS_DISTRO}-example-interfaces \
    && git clone -b ${BRANCH} --single-branch https://github.com/RobotWebTools/rclnodejs.git \
    && cd /rosdev/rclnodejs \
    && npm i

WORKDIR /rosdev/rclnodejs
CMD [ "bash" ]
