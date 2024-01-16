# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /project

# Install additional dependencies if needed
RUN apt-get update \
    && apt-get -y --no-install-recommends install \
    git \
    gcc \
    vim \
    psmisc \
    libxml2-dev \
    libxslt-dev \
    python3 \
    python3-pip \ 
    python-is-python3\
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn

# python packages
RUN pip3 install setuptools && pip3 install catkin-tools

# bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy the project into the container
COPY . /project

# catkin build
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'