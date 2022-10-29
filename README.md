# MMAN4020_TNSW_Robot

## Setup Instructions

1. Download ROS 1 noetic:
http://wiki.ros.org/noetic/Installation/Ubuntu

This should give you a bunch of other packages along with it including Gazebo and Opencv4

2. Download and Setup Moveit:

https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

Follow this tutorial to setup moveit and all the required packages on your computer/VM. Double check that it works by following this tutorial. If you can get the panda arm to move about successfully then moveit should be ready to go

3. If you've been following this tutorial: 

https://roboticscasual.com/robotics-tutorials/?fbclid=IwAR2pqvCAhUTzn55wen7e8GJRhOzpmYA0ypSD4DwCAzm5ZWLmcwBqXW4ggr0

you should have most or all the packages needed to launch the ur5 robot along with the camera correctly. If  not you can just copy over whatever packages you need from ur5_tutorial_packages onto your workspace src folder. 

## Launching the robot and world

To launch the world with the robot in it, run the following commands

`roslaunch ur5_gripper_moveit_config demo_gazebo.launch`

To launch the node that will command the robot to perform the pick and places function, run the following command: 

`rosrun ur5_candybar_pick_and_place pick_and_place_cone`

### Launching different worlds

If you want to spawn the robot in a different world you'll need to go into **ur5_gripper_moveit_config/launch/gazebo.launch**, locate the line 

`<arg name="world_name" value="$(find ur5_candybar_pick_and_place)/world/test.world"/>`

and replace the "value" field with the path to the world you want to launch. So for example here, I made a package called ur5_candybar_pick_and_place so I just chucked that into the value using "$(find ur5_candybar_pick_and_place)" because ros will just find that package location automatically that way, and then the rest of the path leads to the location of the world file I want to launch

### Changing robot spawn location
Navigate to **ur5_gripper_moveit_config/launch/gazebo.launch** and find the line 

` <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0 -y -0.5 -z 0.1"
    respawn="false" output="screen" /> `

edit the -x, -y and -z arguments to change the location of the robot spawned in gazebo. 

## How to create and modify your own packages

We wanna keep this git repo and the ros workspace separate, but still be able to reflect changes on both directories at the same time. The way we're gonna do that is through symbolic linking. If you wanna know what symboli linking exactly is have a look at this: 
https://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/

but in short what this will allow us to do is make changes to the package we made, and then have it not only build and run in your workspace, but also get tracked by git, and we dont have to worry about actually copy pasting the folders anywhere. 

### Steps on creating the new package

1. Make a new folder in the git repo wherever you wanna do it. E.g lets say I wanna make a new package called test_pkg under actuation packages. 

`mkdir test_pkg`
`cd test_pkg`
`catkin_create_pkg test_pkg std_msgs rospy roscpp`

2. Add a symbolic link between the package in the repo and your workspace:

`ln-s (path to where u cloned the repo)/MMAN4020_TNSW_ROBOT/actuation_packages/ (path to your workspace)/src/`

3. see if you find your packge in src folder and build and run it successfully in catkin


## Making changes to the git repo

1. Make your own git branch

`git checkout -b "your branch name"`

2. Once its ready for merge create a pull request and assign someone to review it:

https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request