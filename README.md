# beginner_tutorials 

## Overview
writing a custom ROS publisher and subscriber,integrating a service and publisher , followed by static tf2 frame frame broadcasting and bag recording.

## Setting up ROS2
Follow the instructions from [here](http://docs.ros.org/en/humble/Installation.html) to install ROS2 (by binary installation) on Ubuntu 22.04 LTS.

## Instructions to run publisher and subscriber:

### Source ROS2 setup file: 
In a new terminal source your ROS2 set up file.If installed from binary floow below instructions
```
source /opt/ros/humble/setup.bash
```
### Navigate to ROS workspace:
Navigate to your ros workspace.If ROS workspace dose not exist create one. Follow these [instructions](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to set up your workspace
```
cd ~/ros2_ws/src
```

### Clone the Package:
```
git clone https://github.com/sairampolina/beginner_tutorials.git
```
### Ensure all dependencies exist
```
rosdep install -i --from-path src --rosdistro humble -y
```
### Build the Package:
Navigate to root of your workspace and build your package
```
cd ..
colcon build --packages-select custom_pubsub
```

### Source your overlay:
In a new terminal
```
source install/local_setup.bash
```

### To Run only the publisher: 
```
ros2 run custom_pubsub talker
```

### To Run only the subscriber:
In a new terminal
```
ros2 run custom_pubsub talker
```

### To launch publisher and subscriber with a single command:
```
ros2 launch custom_pubsub custom_pubsub_service_launch.yaml
```

### To launch publisher,subsriber and adjust frequrncy of publishing:
Frequency of publishing data, can be altered by the user using this command:
```
ros2 launch custom_pubsub custom_pubsub_service_launch.yaml custom_pubfreq:=5.0
```

## Running Services 
A service ```/modify_message``` is written,which modifies request message and  returns response message.
```
ros2 service call /modify_message custom_pubsub/srv/ModifyString "{request_message: hello}"
```

## To check different LOGGING 
### Error log:
```
ros2 launch custom_pubsub custom_pubsub_service_launch.yaml custom_pubfreq:= -1.0
```
### Warn log and Fatal log:
```
ros2 launch custom_pubsub custom_pubsub_service_launch.yaml custom_pubfreq:= 0.0
```
### Debug log:
At the start of publisher you can visulize DEBUG log
```
ros2 launch custom_pubsub custom_pubsub_service_launch.yaml 
```
## rqtconsole output
```
ros2 run rqt_console rqt_console
```
## Publish static tf2 frame
To publish a static frame called /talk to /tf2_static topic
```
ros2 run custom_pubsub talker talk 1 0 1 0 0 0
```
## Bag recording from launch file
Open a new terminal and start a publisher so that all topics are available.
```
ros2 run custom_pubsub talker
```
In a new terminal, use the launch file to record all topics.
set record_all_topics(false/true) to disable/enable recording (default=true).

Navigate to folder where you want to store your bag.
```
ros2 launch custom_pubsub bag_recorder_launch.py record_all_topics:=true
```
### To check content in the bag:
It should show different topics recorded and no of messages recorded of each type..
```
ros2 bag info all_topics_bag/
```
### Using bag as a publisher:
Navigate to folder where you stored bag.

run the bag
```
ros2 bag play all_topics_bag/
```
run the subscriber
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
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ../results/cppcheck.txt
```
