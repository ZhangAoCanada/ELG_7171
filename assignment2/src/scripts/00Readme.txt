ELG7171 Assignment 2  00Readme.txt
Student:            Ao Zhang
Student number:     0300039680

The format of this Readme will follow the github README.md format

# ELG7171 Assignment 2

## Denpendencies
1. ros-kinetic
2. python-numpy
3. husky

## Create ROS package
'''
$cd ~/catkin_ws/src
$catkin_create_pkg assignment1_AoZhang rospy geometry_msgs turtlesim
$cd ..
$catkin_make
$. ~/catkin_ws/devel/setup.bash 
$(or source ~/.bashrc)
'''

## Copy all files into the package
'''
$roscd assignment1_AoZhang/src
$mkdir scripts
$cd scripts
$scp -r [directory of my assignment files]/* ./
$chmod 777 ./*
'''

## Run the package
Start a terminal window
'''
$roscore
'''
Start another terminal window
'''
$rosrun turtlesim turtlesim_node
'''
Start another terminal window
'''
$rosrun assignment1_AoZhang Ao_Zhang_300039680.py
'''
When you see the words like "Enter the x coordinate of the destination:", please enter
the value of destination's x coordinate. And when "Enter the y coordinate of the destination:", 
please enter the value of destination's y coordinate.
Then the turtle will move to the destination.

## Re-run the node
If you see the tutle stops with the message
'''
----------------------------------------------
target reached, please enter new destination
----------------------------------------------
'''
It means you can re-enter the destination and run it again
