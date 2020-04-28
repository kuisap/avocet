FROM ros:melodic-ros-base

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-compressed-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/WiringPi/WiringPi \
    && cd WiringPi \
    && ./build
