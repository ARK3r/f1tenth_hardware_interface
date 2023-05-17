from ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive


RUN apt update && apt install locales \
	&& locale-gen en_US en_US.UTF-8 \
	&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
	&& export LANG=en_US.UTF-8


RUN apt install -y software-properties-common \
	&& add-apt-repository universe


RUN apt update && apt install curl -y \
	&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  unzip

RUN apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures


RUN mkdir -p /ros2_ws/src 
WORKDIR /ros2_ws
COPY ./ros2.repos /ros2_ws/src/ros2.repos

RUN vcs import src < src/ros2.repos \
	&& rm src/ros2.repos

RUN cd src \
	&& wget https://github.com/ros/joint_state_publisher/archive/refs/tags/2.3.0.zip \
	&& unzip 2.3.0.zip \
	&& mv joint_state_publisher-2.3.0 joint_state_publisher \
	&& rm 2.3.0.zip

RUN apt update -y \
  && apt upgrade -y \
	&& rosdep init \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN colcon build --symlink-install

ARG CACHEBUST=1

ADD . src/f1tenth_hardware_interface

RUN . /ros2_ws/install/setup.sh \
	&& colcon build --symlink-install --packages-select f1tenth_hardware_interface
	

ENV XDG_RUNTIME_DIR=/tmp

ENV DEBIAN_FRONTEND=dialog
