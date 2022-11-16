# beginner_tutorials - How to write a custom ROS publisher and subscriber

## Overview
Custom publisher and subscriber in Ros2 frame work are written to understand the working of  ROS nodes and how communication is made through ROS Topics.Instructions on how to build the ROS package and run the executables is illustrated below

## Setting Up ROS2
Follow the instructions from [here](http://docs.ros.org/en/humble/Installation.html) to install ROS2 (binary installation) on Ubuntu 22.04 LTS.

## Instructions to run publisher and subscriber:

### Source ROS2 setup file 
In a new terminal source your ROS2 set up file.If installed from binary floow below instructions
```
source /opt/ros/humble/setup.bash
```
### Navigate to ROS workspace
Navigate to your ros workspace.If ROS workspace dose not exist create one. Follow these [instructions](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to set up your workspace
```
cd ~/ros2_ws/src
```

### Clone the Package
```
git clone https://github.com/sairampolina/beginner_tutorials.git
```
### Ensure all dependencies exist
```
rosdep install -i --from-path src --rosdistro humble -y
```
### Build the Package 
Navigate to root of your workspace and build your package
```
cd ..
colcon build --packages-select custom_pubsub
```

### Source your overlay
In a new terminal
```
source install/local_setup.bash
```

### To Run the publisher
```
ros2 run custom_pubsub talker
```

### To Run the subscriber
In a new terminal
```
ros2 run custom_pubsub talker
```
## Static code analysis
Navigate to custom_pubsub package and then run

### Cpplint
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ../results/cpplint.txt
```
### Cppcheck
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheck.txt
```
