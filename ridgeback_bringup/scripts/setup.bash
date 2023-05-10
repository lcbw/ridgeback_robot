# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# Load the robot's model type and serial number
source /etc/clearpath-serial.bash

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######

# Pass through to the main ROS workspace of the system.
#source /opt/ros/foxy/setup.bash
source /opt/ros/noetic/setup.bash

# If you have a catkin workspace, source it below. e.g.
#source /home/development/ros2_port/install/setup.bash


# Any additional environment variables that depend on your workspace should be exported here
export RIDGEBACK_MICROSTRAIN_IMU=1
export RIDGEBACK_MICROSTRAIN_IMU_PREFIX=upgraded
export RIDGEBACK_MICROSTRAIN_IMU_LINK=upgraded
export RIDGEBACK_MICROSTRAIN_IMU_TOPIC=upgraded
export RIDGEBACK_MICROSTRAIN_IMU_MOUNT=mid
export RIDGEBACK_MICROSTRAIN_IMU_OFFSET="0 0 0"
export RIDGEBACK_MICROSTRAIN_IMU_RPY="0 0 0"
export RIDGEBACK_FRONT_HOKUYO_LASER=1
export RIDGEBACK_FRONT_LASER_HOST=192.168.131.14
