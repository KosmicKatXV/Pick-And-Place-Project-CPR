# Pick and Place Project

This project consists of a panda arm placed on a table in front of a small box wich will be picked and placed. The state of the robot will be tracked by means of a state machine and commanded by another process.

## Installation

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/SnehalD14/panda_simulation.git
git clone https://github.com/SnehalD14/panda_moveit_config.git
git clone https://github.com/SnehalD14/franka_ros.git
git clone https://github.com/SnehalD14/realsense_gazebo_plugin.git
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
cd ..
```
It is also important that you build the *libfranka* library from source and pass its directory to *catkin_make*  when building this ROS package as described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source).

Build the catkin workspace and run the simulation:
```
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```
## Running the project

The commands to run the project are the following:
```
source devel/setup.bash
```
Note that each of the next commands need to be run in three different terminals:
```
roslaunch pick_and_place_project simulation.launch
rosrun pick_and_place_project fsm.py
rosrun pick_and_place_project move_robot.py
```

## Acknowledgement 

This is a modified and extended version of the work done by [Erdal Pekel](https://github.com/erdalpekel/panda_simulation)
