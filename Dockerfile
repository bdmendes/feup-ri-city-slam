FROM duckietown/gym-duckietown
RUN cd /
COPY --chown=duckietown:duckietown . /feup-ri-city-slam
RUN apt update
RUN apt install -y fontconfig
RUN apt install -y make
RUN apt install -y libgtk2.0-dev pkg-config
RUN apt install -y git
RUN apt install -y libgl1-mesa-dev libglew-dev cmake python3.6 libpython3.6-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libtiff5-dev libopenexr-dev python3-pip g++ git gcc
RUN export WORKDIR=/feup-ri-city-slam/lib/
RUN apt -y install cmake ffmpeg python3 libeigen3-dev
RUN cd $WORKDIR && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
RUN cd $WORKDIR/Pangolin && mkdir build && cd build && cmake .. && cmake --build .
RUN apt install -y libssl-dev libopencv-dev libboost-filesystem-dev libboost-serialization-dev
# TODO: add ROS noetic installation; add ./build-ros.sh step for ORB-SLAM3