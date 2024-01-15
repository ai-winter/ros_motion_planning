# Use the official ROS Melodic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /workspace

# Install additional dependencies if needed
# For example, you can uncomment the line below to install a package
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    libxml2-dev \
    libxslt-dev \
    python3 \
    python3-pip\ 
    python-is-python3\
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn

RUN pip3 install setuptools
RUN pip3 install catkin-tools
# Copy your ROS packages into the workspace
COPY . /workspace/src/

WORKDIR /workspace


# Set environment variables
ENV ROS_DISTRO noetic
ENV ROS_VERSION 1
# RUN catkin build
RUN . /opt/ros/noetic/setup.sh && catkin_make


# Source the ROS setup file
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Expose ROS master port
EXPOSE 11311

# Set entry point to start ROS
CMD ["roscore"]
