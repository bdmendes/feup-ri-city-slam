sudo apt install open-vm-tools
sudo apt install libgl1-mesa-dev libglew-dev cmake python3.6 libpython3.6-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libtiff5-dev libopenexr-dev python3-pip g++ git gcc
sudo apt install cmake ffmpeg libeigen3-dev

# Pangolin
cd lib/Pangolin
mkdir build && cd build
cmake ..
cmake --build .

# ORB SLAM
sudo apt install libssl-dev libopencv-dev libboost-filesystem-dev libboost-serialization-dev unzip
cd ../ORB_SLAM3/
chmod +x build.sh
./build.sh

