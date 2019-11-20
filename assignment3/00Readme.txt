ELG 7171: Assignment 3 00Readme.txt
Student : Ao Zhang
Student Number: 0300039680

The format of this Readme will follow the github README.md format

# ELG 7171: Assignment 3

## Dependencies
1. ros-kinetic
2. python-numpy
3. turtlebot3_gazebo

## Create ROS package
```
$cd ~/catkin_ws/src
$catkin_create_pkg assignment3 turtlebot3_gazebo rospy tf
$cd ..
$catkin_make
$. ~/catkin_ws/devel/setup.bash 
$(or source ~/.bashrc)
```

## Copy python files into the package
```
$roscd assignment3/src
$mkdir scripts
$cd scripts
$scp -r [my .py files]/*.py ./
$chmod 777 ./*
```

## Copy launch files into the package
```
$roscd assignment3
$mkdir launch
$cd launch
$scp -r [my .py files]/*.launch ./
```

## Run the package
First, start a new terminal window,
```
export TURTLEBOT3_MODEL=burger
```
Start a new terminal window
```
$roslaunch assignment3 Ao_Zhang_300039680.launch
```

## Change the initial position of the robot
Note that the arguments about the initial position set inside the launch file are
```
x_pos  y_pos  z_pos
```

Therefore, if one wants to change the initial position of the robot, he/she should do something like,
```
roslaunch assignment3 Ao_Zhang_300039680.launch x_pos:=-1 y_pos:=-1
```
