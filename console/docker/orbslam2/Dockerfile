FROM poifull10/ros_melodic

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  libglew-dev libeigen3-dev \
  && rm -rf /var/lib/apt/lists/*

ARG WORK="/work"

WORKDIR $WORK

RUN git clone https://github.com/stevenlovegrove/Pangolin.git

RUN cd Pangolin && mkdir -p build && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF \
  && make && make install && cd ../../ && rm -rf Pangolin

WORKDIR $WORK

RUN git clone https://github.com/poifull10/ORB_SLAM2.git \
  && cd ORB_SLAM2 \
  && ./build.sh \
  && . /opt/ros/melodic/setup.sh && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/work/ORB_SLAM2/Examples/ROS \
  && ./build_ros.sh \
  && ln -s $WORK/ORB_SLAM2/Examples/ROS/ORB_SLAM2 /opt/ros/melodic/share/ORB_SLAM2

WORKDIR $work/ORB_SLAM2
