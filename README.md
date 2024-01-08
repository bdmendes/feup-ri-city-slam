# feup-ri-city-slam
SLAM project with Duckietown for Intelligent Robotics, FEUP.

## Prerequisites for running ORB SLAM 3 locally with the DuckieTown Gym Simulator 
- python==3.8
- Ubuntu==20.04.6
- Running `xhost + local:docker` on host machine (for GUI support)

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
    sudo apt install -y libeigen3-dev libopencv-dev ros-noetic-rviz
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

## Running ORB SLAM 3 with DuckieTown Gym Simulator
### Running without docker
From the root directory of the repo, build the `ROS` workspace:
```sh
source /opt/ros/noetic/setup.bash
cd catkin_workspace
rosdep install --from-paths src --ignore-src -r -y
catkin build # This will most likely fail the first time, if it does, run the next line
# catkin build -j1
```

In a terminal:
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

### Running with docker
```sh
docker buildx build -f Dockerfile.orbslam3 -t orbslam3 .
docker buildx build -f Dockerfile.duckiesim -t duckiesim .
```

```sh
docker run --rm -t --net=host ros:noetic roscore
```

```sh
# Allow docker to access the host's display
xhost + local:docker

docker run --gpus all --rm -it --ipc=host --net=host --privileged \
  --env="DISPLAY=unix$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --runtime=nvidia \
  duckiesim
```

```sh
# Allow docker to access the host's display
xhost + local:docker

# Run the ORB SLAM 3 container
docker run --gpus all --rm -it --ipc=host --net=host --privileged \
  --env="DISPLAY=unix$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --runtime=nvidia \
  orbslam3
```


## Run in Real Life with duckiebot
```sh
docker buildx build -f Dockerfile.orbslam3 -t orbslam3 .
docker buildx build -f Dockerfile.duckiebot-translator -t duckiebot-translator .
```

```sh
docker run --rm --net=host \
    -e ROS_MASTER_URI=http://pato.local:11311 \
    duckiebot-translator
```

```sh
# Allow docker to access the host's display
xhost + local:docker

# Run the ORB SLAM 3 container
docker run --gpus all --rm -it --ipc=host --net=host --privileged \
  --env="DISPLAY=unix$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --runtime=nvidia \
  -e ROS_MASTER_URI=http://pato.local:11311 \
  orbslam3
```

```sh
# Start the line following demo
dts duckiebot demo --demo_name lane_following --duckiebot_name pato

# Start the keyboard control demo with the line following control
dts dts duckiebot keyboard_control pato
```
