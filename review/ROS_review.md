# Robotics Operating System (ROS)

## :boom: Basic Commands

### 1. start a master
```
$ roscore
```

### 2. run a node
```
$ rosrun package_name node_name
```

### 3. see active nodes
```
$ rosnode list
```

### 4. find node info
```
$ rosnode info node_name
```

### 5. see active topic
```
$ rostopic list
```

### 6. to suscribe to and print the content of a topic
```
$ rostopic echo /topic_name
```

### 7. find infor about a topic
```
$ rostopic info /topic_name
```

### 8. find type of topic 
```
$ rostopic type /topic_name
```

### 9. publish a message to a topic
```
$ rostopic pub [-r] /topic_name type data
```

## :boom: Some comments on Nodes and Topics

* The common way of nodes communicating with each other is through topics.

* A topic is a name or a stream of messages with defined types.

* Before transmitting through topics, the topic name must first be advertised to the master .

## :boom: ROS Packages

### 1. create a package
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg pkg_name dependencies[rospy, tf, ...]
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

### 2. kill a node
from python file
```
rospy.signal_shutdown("GoodBye")
```

from bash command
```
$ rosnode kill node_name
```

### 3. loginfo
from python file
```
rospy.loginfo(string)
```

*Some commants on it*

* It prints a string onto the screen.

* It prints a string onto the node's log file.

* It publishes the string to node rosout.

### 4. subscriber

Nodes that want to receive messages on a certain topic can subscribe to that topic by making a request to *roscore*

### 5. usage of spin()
from python file
```
rospy.spin()
```

This keeps the node alive until it explicitly shut down.

## :boom: Namespace

### 1. global names
```
/turtle1/pose
```

### 2. relative names
if default namespace is ```/a/b/c``` and relative graph resource name is ```d/e/f```, then the global resource name is ```/a/b/c/d/e/f```

For *default namespace*, one can remap it by
```
$ __ns:=[new_namespace]
```
or
```
$ export ROS_NAMESPACE=new_default_ns
```

### 3. private names
begin with ```~```
```
~pose
```

### 4. anonymous names
This name is to ensure the uniqueness, and from python file
```
rospy.init_node('node_name', anonymous = True)
```

### 5. remapping arguments

* remap the default namespace
```
$ __ns:=[new_namespace]
```

* remap the node's name
```
$ __name:=[new_name]
```

* remap the default name of hte resource's log file
```
$ __log:=[new_name]
```

## :boom: Gazebo Simulation

### 1. view of the .world files
```
$ /usr/share/gazebo-7/worlds/
```

To find a ros package
```
rospack find [package_name]
```

### 2. run husky with world arg
```
$ roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/willowgarage.world
```

## :boom: Visualization

### 1. Rviz
```
$ rosrun rviz rviz
```

### 2. rqt_plot
```
$ rqt_plot
$ rqt_plot /turtle1/pose/x /turtle/pose/y
```

## :boom: Ros Bag (Record and Replay)

### 1. record
* to record specific topics
```
$ rosbag record -o filename[.bag] topic1 topic2
```

* to record all active topics
```
$ rosbag record -o filename[.bag] -a
```

### 2. stop recording
recommanded way
```
$ rosnode kill
```

force stop way
```
ctrl + C
```

### 3. play bags
* to get the info of the bag file
```
$ rosbag info filename[.bag]
```

* to play the bag
```
$ rosbag play [-r freq] filename[.bag]
```

*Note*: one can pass arg, ```-r 5``` to overwrite the topic publishing rate to 5 Hz.

*Also Note*: The reason for the error when replaying is that the ```distance = intergral of velocity``` for bag files.

### 4. alternative way
```
$ rqt_bag
```

## :boom: ROS launch file

### 1. an example of ros launch file
```
<launch>
  <node
    pkg="turtlesim"
    type="turtlesim_node"
    name="turtleA"
    respawn="true"
    >
  </node>

  <node
    pkg="ttcontrol"
    type="posesubscrib.py"
    name="control_tt"
    output="screen"
    >
  </node>

  <node
    pkg="teleop_twist_keyboard"
    type="teleop_twist_keyboard.py"
    name="teleop"
    launch-prefix="xterm -e"
    >
    <remap from="cmd_vel" to="turtle1/cmd_vel"/>
  </node>
</launch>
```

### 2. notable args of nodes

* ```type```: excutable file (e.g. pythonfile.py)

* ```name```: node name (for remapping)

### 3. run a launch file
```
$ launch /path/to/launch/file.launch
```
or
```
# roslaunch package_name launchfile.launch
```

### 4. stop a launch file
```
ctrl + C
```

### 5. some comments

* XML syntex, ```<launch>``` and ```</launch>```.

* the log files of the latest ros launch are kept at: ```/.ros/log/latest```.

* output to screen: ```output="screen"```.

* make the window always open ```respawn="true"```.

* make the window termination core ```required="true"```.

* change the namespace ```ns="new_namespace"```.

* open another terminal ```launch-prefix="xterm -e"```. [this only works on Ubuntu 16.04]

* remap in launch file
```
<remap from="original_name" to="new_name"/>
```

### 6. include launch file in launch file
```
<include file="file/path/to/launch/file"/>
```
or
```
<include file="$(find package_name)/relative/path/to/file"/>
```

*Note*: one can push the content of an included file down into a namespace by
```
<include file="..." ns="..."/>
```

### 7. arguments in the terminal
```
$ roslaunch pkg_name luanch_file arg1_name:=arg1_value arg2_name:=arg2_value
```

### 8. arguments in launch file
* declare argument
```
<arg name="arg_name">
```

* assign the default value of the argument, which *can be overwritten* from bash command
```
<arg name="arg_name" default="arg_default_value"/>
```

* assign the value of the argument, which *cannot be overwritten* from bash cmd
```
<arg name="arg_name" value="arg_value"/>
```

* get argument value inside launch file by using ```$(arg arg_name)```.

### 9. Example of arguments in launch files
*LAUNCH FILE 1*
```
<launch>
  <arg name="x" value="4"/>
  <arg name="enable-laser" value="true"/>
</launch>
```

*LAUNCH FILE 2*
```
<launch>
  <arg name="y" value="10"/>
</launch>
```

*INCLUDE LAUNCH FILE 2 INTO 1*
```
<launch>
  <arg name="x" default="4"/>
  <arg name="enable-laser" value="true"/>
  <include file=".../file2.launch">
    <arg name="y" value="$(arg x)"/>
  </include>
</launch>
```

### 10. groups

* about namespace
```
<group ns="namespace">
  : all nodes within will be pushed into this default namespace
</group>
```

* `if` condition
```
<group if="4">
  : all nodes will be included
</group>
```

* still `if` condition
```
<group if="0">
  : all nodes will not be included
</group>
```

* `unless` condition
```
<group unless="1">
  : all nodes will not be included
</group>
```

