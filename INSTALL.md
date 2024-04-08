# Yumi ROS Interface Intallation

Please follow the steps in [ROS-wiki](http://wiki.ros.org/ROS/Installation) to install the corresponding ROS distribution.
- in my case Ubuntu `20.04`: [ROS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Please read this [wiki](https://github.com/kth-ros-pkg/yumi/wiki) before to start.

## Dependencies

### ROS and Python3 Dependencies

```
sudo apt-get install \
        python3-pip \
        protobuf-compiler \
        protobuf-c-compiler \
        ros-$ROS_DISTRO-control-toolbox \
        ros-$ROS_DISTRO-controller-interface \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-effort-controllers \
        ros-$ROS_DISTRO-force-torque-sensor-controller \
        ros-$ROS_DISTRO-gazebo-ros-control \
        ros-$ROS_DISTRO-joint-limits-interface \
        ros-$ROS_DISTRO-joint-state-publisher \
        ros-$ROS_DISTRO-joint-state-controller \
        ros-$ROS_DISTRO-joint-trajectory-controller \
        ros-$ROS_DISTRO-moveit-commander \
        ros-$ROS_DISTRO-moveit-core \
        ros-$ROS_DISTRO-moveit-planners \
        ros-$ROS_DISTRO-moveit-ros-move-group \
        ros-$ROS_DISTRO-moveit-ros-planning \
        ros-$ROS_DISTRO-moveit-ros-visualization \
        ros-$ROS_DISTRO-moveit-simple-controller-manager \
        ros-$ROS_DISTRO-position-controllers \
        ros-$ROS_DISTRO-rqt-joint-trajectory-controller \
        ros-$ROS_DISTRO-transmission-interface \
        ros-$ROS_DISTRO-velocity-controllers \
        ros-$ROS_DISTRO-hector-xacro-tools \
        ros-$ROS_DISTRO-joint-*
```
Then:

    pip3 install --user pyftpdlib
    pip3 install --user --upgrade pyassimp
    pip3 install --user vcstool

## Quick Start

### Yumi module installation

    mkdir -p ~/ws_yumi/src && cd ~/ws_yumi/src
    git clone https://github.com/maxencegrand/yumi.git

### abb_robot_driver installation

abb_robot_driver is the driver developed by ros-industrial for ABB industrial robot control via ROS. This module is required to use the Yumi Module. Please follow the [installation steps](https://github.com/ros-industrial/abb_robot_driver/). Or simply copy and paste:

    cd ~/ws_yumi
    vcs import src --input https://github.com/ros-industrial/abb_robot_driver/raw/master/pkgs.repos

### Build Modules
    cd ~/ws_yumi
    rosdep install --from-paths src --ignore-src -r -y
    catkin build


### Network Setup:

Please be sure to follow the [Network Setup Instructions](https://github.com/kth-ros-pkg/yumi/wiki/Network-setup)
