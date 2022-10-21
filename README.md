# MMAN4020_TNSW_Robot

## Setup Instructions

1. Download ROS 1 noetic:
http://wiki.ros.org/noetic/Installation/Ubuntu

This should give you a bunch of other packages along with it including Gazebo and Opencv4

2. Download and Setup Moveit:

https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

Follow this tutorial to setup moveit and all the required packages on your computer/VM. Double check that it works by following this tutorial. If you can get the panda arm to move about successfully then moveit should be ready to go

3. Copy over all the packages in this repo under "ros_packages" to the src folder in your catkin workspace. 

## Launching the robot and world

To launch the world with the robot in it, run the following commands

`roslaunch ur5_gripper_moveit_config demo_gazebo.launch`

### Launching different worlds

If you want to spawn the robot in a different world you'll need to go into **ur5_gripper_moveit_config/launch/gazebo.launch**, locate the line 

`<arg name="world_name" value="$(find ur5_candybar_pick_and_place)/world/test.world"/>`

and replace the "value" field with the path to the world you want to launch. So for example here, I made a package called ur5_candybar_pick_and_place so I just chucked that into the value using "$(find ur5_candybar_pick_and_place)" because ros will just find that package location automatically that way, and then the rest of the path leads to the location of the world file I want to launch

### Changing robot spawn location
Navigate to **ur5_gripper_moveit_config/launch/gazebo.launch** and find the line 

` <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0 -y -0.5 -z 0.1"
    respawn="false" output="screen" /> `

edit the -x, -y and -z arguments to change the location of the robot spawned in gazebo. 
