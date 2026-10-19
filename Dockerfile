# Create a source-development image with ROS 2, colcon, Node.js and rclnodejs.
# Supported ARGS:
#   ROS_DISTRO = [humble, jazzy, kilted, lyrical, rolling], default=rolling
#   NODE_MAJOR_VER = [20, 22, 24], default=24
#   BRANCH = rclnodejs git branch, default=develop
#
# Examples:
#
# Build an image named 'rclnodejs' and start a source-development shell.
#
#    docker build -t rclnodejs .
#    docker run --rm -it rclnodejs
#
# The image clones BRANCH from GitHub; it does not copy the local checkout.
# The full npm test suite needs additional dependencies, including Electron/Xvfb.
# Build package entry points with npm run build:dist when using Node 22.18+ or 24.11+.
#
# Build an image for a specific branch of rclnodejs, version of ROS2 and Nodejs use:
#
#     docker build -t <image_name> --build-arg ROS_DISTRO=kilted .
#     docker build -t <image_name>  \
#        --build-arg ROS_DISTRO=jazzy \
#        --build-arg BRANCH=develop \
#        --build-arg NODE_MAJOR_VER=24 .
#
#
# Build and run:
#     docker run -it --rm $(docker build -q .)
#

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install dependencies, including Nodejs
ARG NODE_MAJOR_VER=24
RUN apt-get update -y \
    && apt-get install -y curl \
    && curl -fsSL https://deb.nodesource.com/setup_${NODE_MAJOR_VER}.x | bash - \
    && apt-get install -y nodejs \
    && npm install --global npm@11.19.0

# clone a branch of the rclnodejs repo, build addon libs, generate corresponding JS msgs
ARG BRANCH=develop
WORKDIR /rosdev
RUN apt-get update -y \
    && apt-get install -y ros-${ROS_DISTRO}-test-msgs ros-${ROS_DISTRO}-example-interfaces \
    && git clone -b "${BRANCH}" --single-branch https://github.com/RobotWebTools/rclnodejs.git \
    && cd /rosdev/rclnodejs \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && npm install

WORKDIR /rosdev/rclnodejs
CMD [ "bash" ]
