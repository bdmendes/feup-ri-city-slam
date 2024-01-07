# feup-ri-city-slam
SLAM project with Duckietown for Intelligent Robotics, FEUP.

## Prerequesits
- python==3.8
- Ubuntu==20.04.6

## Compiling and Running

### Install simulator dependencies and setup:

From the root directory of the repo:

```sh
./setup_sim.sh
```

### Install ROS Noetic

Follow the instructions in the [ROS Noetic Website](https://wiki.ros.org/noetic/Installation/Ubuntu).
Make sure to also install dependencies for package building.


### Installing ORB SLAM 3 Dependencies

As in the [orb_slam3_ros repository]():

1. Install Eigen:
    ```sh
    sudo apt install libeigen3-dev
    ```
2. Install Pangolin:
    ```sh
    cd ~
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```
3. Verify OpenCV is installed (required at least 3.0):
    ```sh
    python3 -c "import cv2; print(cv2.__version__)" 
    ```

### Build ROS Workspace

From the root directory of the repo:

```sh
source /opt/ros/noetic/setup.bash
cd catkin_workspace/src/duckie_sim_pkg
catkin build # This will most likely fail the first time
catkin build -j1
```

### Running ORB SLAM 3 with DuckieTown Gym Simulator

In one terminal:
```sh
source /opt/ros/noetic/setup.bash
cd catkin_workspace
source devel/setup.bash
roslaunch orb_slam3_ros duckie_sim.launch
```

In another terminal:
```sh
source /opt/ros/noetic/setup.bash
cd catkin_workspace
source devel/setup.bash
rosrun duckie_sim_pkg ros_script.py 
```
