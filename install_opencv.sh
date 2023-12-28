sudo apt install ros-noetic-librealsense2 libg2o ros-noetic-libg2o

cd ~
git clone git@github.com:opencv/opencv.git
cd opencv
git checkout 4.5.0
rm -rf build
mkdir build
cd build
cmake ..
#make -j4
sudo make install
