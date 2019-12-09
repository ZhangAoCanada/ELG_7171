# Robotics Operating System (ROS)

## :boom: Basic Commands

#### 1. start a master
```
$ roscore
```

#### 2. run a node
```
$ rosrun package_name node_name
```

#### 3. see active nodes
```
$ rosnode list
```

#### 4. find node info
```
$ rosnode info node_name
```

#### 5. see active topic
```
$ rostopic list
```

#### 6. to suscribe to and print the content of a topic
```
$ rostopic echo /topic_name
```

#### 7. find infor about a topic
```
$ rostopic info /topic_name
```

#### 8. find type of topic 
```
$ rostopic type /topic_name
```

#### 9. publish a message to a topic
```
$ rostopic pub [-r] /topic_name type data
```

## :boom: Some comments on Nodes and Topics

* The common way of nodes communicating with each other is through topics.

* A topic is a name or a scream of messages with defined types.

* Before transmitting through topics, the topic name must first be advertised to the master .

## :boom: ROS Packages

#### 1. create a package
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg pkg_name dependencies[rospy, tf, ...]
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

#### 2. kill node
from python file
```
rospy.signal_shutdown("GoodBye")
```

from bash command
```
$ rosnode kill node_name
```

#### 3. loginfo
from python file
```
rospy.loginfo(string)
```

*Some commants on it*

* It prints a string onto the screen.

* It prints a string onto the node's log file.

* It publishes the string to node rosout.

#### 4. subscriber

Nodes that wnat to receive messages on a certain topic can subscribe to that topic by making a request to *roscore*

#### 5. usage of spin()
from python file
```
rospy.spin()
```

This keeps the node alive until it explicitly shut down.

## :boom: Namespace

#### 1. global names
```
/turtle1/pose
```

#### 2. relative names
if default namespace is ```/a/b/c``` and relative graph resource name is ```d/e/f```, then the global resource name is ```/a/b/c/d/e/f```