# feup-ri-city-slam
SLAM project with Duckietown for Intelligent Robotics, FEUP.

## Prerequesites
- python==3.8
- Ubuntu==20.04.6
- Running `xhost + local:docker` on host machine (for GUI support)

## Installing

### Install simulator dependencies and setup:

From the root directory of the repo:

```sh
./setup_sim.sh
```

### Install ROS Noetic

Follow the instructions in the [ROS Noetic Website](https://wiki.ros.org/noetic/Installation/Ubuntu).
Make sure to also install dependencies for package building.

#### Install `catkin`
```sh
sudo apt install -y python3-catkin-tools
```

### Installing ORB SLAM 3 Dependencies

As in the [orb_slam3_ros repository](https://github.com/marhcouto/orb_slam3_ros):

1. Install dependencies:
    ```sh
    sudo apt install -y libeigen3-dev libopencv-dev sudo apt-get install ros-noetic-rviz
    ```
2. Install Pangolin:
    ```sh
    cd ~
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin

    # Next command requires you to confirm the installation of dependencies
    sudo ./scripts/install_prerequisites.sh recommended

    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```

3. Verify OpenCV is installed (required at least 3.0):
    ```sh
    pip3 install opencv-python
    python3 -c "import cv2; print(cv2.__version__)" 
    ```

4. (Optional) Hector trajectory server
    ```sh
    sudo apt install -y ros-[DISTRO]-hector-trajectory-server
    ```

## Run in Simulator

### Build ROS Workspace

From the root directory of the repo:

```sh
source /opt/ros/noetic/setup.bash
cd catkin_workspace
rosdep install --from-paths src --ignore-src -r -y
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

## Run in Real Life

