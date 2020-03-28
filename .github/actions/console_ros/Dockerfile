FROM ubuntu:bionic


RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  lsb-release gnupg gnupg1 gnupg2 curl \
  && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-melodic-desktop \
  && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENV ROS_DISTRO melodic

ENV ROS_MASTER_URI http://localhost:11311

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

