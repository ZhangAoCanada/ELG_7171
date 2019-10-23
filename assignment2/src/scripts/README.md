# ELG7171 Assignment 2

## Denpendencies
1. ros-kinetic
2. python-numpy
3. husky

## Create ROS package
```
$cd ~/catkin_ws/src
$catkin_create_pkg assignment2_aozhang rospy husky_gazebo
$cd ..
$catkin_make
$. ~/catkin_ws/devel/setup.bash 
$(or source ~/.bashrc)
```

## Copy all files into the package
```
$roscd assignment2_aozhang/src
$mkdir scripts
$cd scripts
$scp -r [directory of my assignment files]/* ./
$chmod 777 ./*
```

## Run the package
Start a terminal window
```
$roslaunch husky_gazebo husky_empty_world.launch
```
Then, add a construction cone wherever you want.

Start another terminal window
```
$rosrun assignment2_aozhang ELG7171_assignment2_AoZhang.py
```
When you see the words like "Enter the x coordinate of the destination:", please enter
the value of destination's x coordinate. And when "Enter the y coordinate of the destination:", 
please enter the value of destination's y coordinate.
Then the robot will move to the destination.

## Target information
When the target has been set, the robot will find its way to the target, with the information showing in the terminal as the following example shows.
```
-------------------------- state freshing ---------------------------
Robot's position:       [2.8515694860060217, 0.9065106073210979] m
Target's position:      [5.0, 0.0] m
Distance error:         2.33184801277 m
Robot's orientation:    -23.1826622871 degrees
Target's orientation:   -22.8768620152 degrees
Orientation error:      0.305800271901 degrees
```
All the information is printed as the assignment requires.

## Collision Avoidance information
When the obstacle is in the robot's way, you will see the information as the following example,
```
---------------------------------------------------------------------
WARNING: POTENTIAL COLLISION DETECTED, ENTER COLLISION AVOIDENCE MODE
Obstacle Distance:      1.09443163872 m
Obstacle Orientation:   8.07369887979 degrees
---------------------------------------------------------------------
```
And the information about the target distance and orientation w.r.t the robot will disappear until the robot successfully avoid the collision.

## Re-run the node
If you see the tutle stops with the message
```
----------------------------------------------
target reached, please enter new destination
----------------------------------------------
```
It means you can re-enter the destination and run it again
