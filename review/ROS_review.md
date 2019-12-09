# Robotics Operating System (ROS)

## Basic Commands

#### 1. start a master
```
roscore
```

#### 2. run a node
```
rosrun package_name node_name
```

#### 3. see active nodes
```
rosnode list
```

#### 4. find node info
```
rosnode info node_name
```

#### 5. see active topic
```
rostopic list
```

#### 6. to suscribe to and print the content of a topic
```
rostopic echo /topic_name
```

#### 7. find infor about a topic
```
rostopic info /topic_name
```

#### 8. find type of topic 
```
rostopic type /topic_name
```

#### 9. publish a message to a topic
```
rostopic pub [-r] /topic_name type data
```