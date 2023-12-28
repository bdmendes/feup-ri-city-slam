# feup-ri-city-slam
SLAM project with Duckietown for Intelligent Robotics, FEUP.

## Compiling and Running

### Running Simulator

#### In Docker

Build docker image and run container:
```shup
./docker_build.sh
```

Tear down container:
```sh
./docker_down.sh
```

The scripts will allow and remove access from docker to x11 and initiallize a bash for the container respectively.

#### In OS

Install dependencies and setup:

```sh
./setup_sim.sh
```

Run:

### Compiling ORB SLAM

To be able to use the ORB SLAM libs (.so), you need to compile them. ORB SLAM requires OpenCV and ROS1. If you don't have them, run `install_ros1noetic.sh` and `install_opencv.sh`.

Remaining dependencies alongside ORBSLAM will be installed via `setup_orb_slam.sh`.
