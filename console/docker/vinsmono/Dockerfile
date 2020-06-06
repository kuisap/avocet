FROM poifull10/ros_melodic

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-melodic-cv-bridge ros-melodic-tf ros-melodic-message-filters ros-melodic-image-transport ros-melodic-image-transport-plugins \
  && rm -rf /var/lib/apt/lists/*

ARG WORK=/work

WORKDIR $WORK

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev software-properties-common libsuitesparse-dev \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ceres-solver/ceres-solver.git \
 && cd ceres-solver \
 && cmake . -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DPROVIDE_UNINSTALL_TARGET=OFF \
 && make -j$(nproc) && make install && cd .. && rm -rf ceres-solver

WORKDIR $WORK/src

RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git \
  && cd .. \
  && . /opt/ros/melodic/setup.sh \
  && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) \
  && . devel/setup.sh
