
# arns (still under development)
Autonomous Robot Navigation System

This is a navigation system for a [differential wheeled robot](https://en.wikipedia.org/wiki/Differential_wheeled_robot) which incorporates SLAM using the [ROS2 slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and [ROS2 Nav2](https://github.com/ros-planning/navigation2) package for our Institutes Project Design.

## Installation
Prerequisites:
* ros2 Humble
> [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* python3-colcon-common-extensions
* ros-humble-rmw-cyclonedds-cpp
```bash
# python3-colcon-common-extensions
sudo apt install python3-colcon-common-extensions

# ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Install these ros2 packages using the command below (take note of the comment):
```bash
# this is for the rpi on the robot
sudo apt install ros-humble-twist-mux ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ros2-control ros-humble-ros2-controllers v4l-utils ros-humble-v4l2-camera ros-humble-image-transport-plugins  ros-humble-rplidar-ros ros-humble-image-transport-plugins ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-joy

# this is for the dev machine
sudo apt install ros-humble-twist-mux ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control v4l-utils ros-humble-v4l2-camera ros-humble-image-transport-plugins  ros-humble-rplidar-ros ros-humble-image-transport-plugins ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-gazebo-ros-pkgs ros-humble-joy ros-humble-rqt-image-view
```

Create a ROS2 workspace and clone the repository in the workspace source folder:
> do this for both the dev machine and RPi on the robot.
```bash
mkdir -p ros2_workspace_name/src

cd ros2_workspace_name/src

git clone https://github.com/ErvinPicardal19/arns.git
```

Move to your ros2 workspace root directory and build the workspace then run the install/setup.bash to source our new packages.
```bash
cd ~/ros2_workspace_name

colcon build

source install/setup.bash
```

OPTIONAL:
Add this to the end of your *.bashrc* to avoid sourcing the *install/setup.bash* of our workspace everytime a new terminal session is opened.
> change the *robot_ws* to your workspace.
```bash
#setup ROS2 Humble
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
ROS2_WS_NAME=robot_ws
ROS2_WS_PATH=/home/$USER/$ROS2_WS_NAME/install
if [ -d "$ROS2_WS_PATH" ];
then
    source $ROS2_WS_PATH/setup.bash
else
        echo "ros2 workspace has not been setup..."
fi

#setup ROS2 colcon_cd
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/$ROS2_WS_NAME

#argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

#GAZEBO MODEL PATH
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/$ROS2_WS_NAME/src/arns_description
```

