# Robotics Operating System (ROS)

## Basic Commands

#### start a master
```
roscore
```

#### run a node
```
rosrun package_name node_name
```

#### see active nodes
```
rosnode list
```

#### find node info
```
rosnode info node_name
```

#### see active topic
```
rostopic list
```

#### to suscribe to and print the content of a topic
```
rostopic echo /topic_name
```

#### find infor about a topic
```
rostopic info /topic_name
```

#### find type of topic 
```
rostopic type /topic_name
```

#### publish a message to a topic
```
rostopic pub [-r] /topic_name type data
```