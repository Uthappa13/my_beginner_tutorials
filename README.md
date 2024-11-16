# ROS2 Beginner Tutorials

This branch contains a C++ package that can run a ROS2 publisher and subscriber communicating a custom message.

### Dependencies
The code dependes on the following ROS2 packages:  
 - `ament_cmake`: Required for ROS2 build system integration.
 - `rclcpp`: Used for creating nodes and communication between them.
 - `std_msgs`: Provides standard message types used in ROS2.

### Building the Code

```bash
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir -p ~/ros2_ws/src
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
#Clone the repository
git clone https://github.com/Uthappa13/my_beginner_tutorials.git
#Go back to the ws directory
cd ~/ros2_ws
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select beginner_tutorials
# After successfull build source the package
source install/setup.bash

# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
```

### Launching the Pub and Sub Nodes

In order to run the publisher and subscriber node together using a launch file, the following command can be run on the terminal. It accepts an argument `publish_frequency` whose value can be initialized by the user.

```bash
ros2 launch beginner_tutorials talkernode.launch.py publish_frequency:=1000
```

### Calling the Service

A service has been added to the talker node that can change the output string of the talker based on user input. The service can be called using the following command:

```bash
ros2 service call /change_string beginner_tutorials/srv/ChangeStr "{new_string: My Custom Input}"
```

### TF Frames

The talker node in this package publishes a static tf transform between 2 frames, `world` and `talk`. To run the publisher, run
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
# Run the publisher in terminal
ros2 run beginner_tutorials talker
```
To view the tf transform, run the following commands in a separate terminal
```bash
 # In a new terminal window, echo the topic that broadcasts the static frame:
ros2 topic echo /tf_static
# In a new terminal window, get more information about the frames
ros2 run tf2_tools view_frames
```

### Integration Tests using Catch2
To check the results of the integration test, run the following commands. In a new terminal
```bash
# Source the package
source install/setup.bash
# Run the test
colcon test --packages-select beginner_tutorials
# Display the output
cat log/latest_test/integration_test/stdout_stderr.log
```

### ROS2 Bag Functionality

This package supports recording and playback of ros2 bags. The launch file has been modified to support ros2 bag recording. To record use the `ros2_bag_start` parameter (True/False).

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
# Run the launch file in terminal with the ros2_bag_start parameter as true
ros2 launch beginner_tutorials talkernode.launch.py ros2_bag_start:=True
```
The above ros2 bag is called `rosbag` and can be found in the workspace directory where the command was run.
To inspect and playback the ros2 bag.
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
# Inspect the ros2 bag
ros2 bag info rosbag
# Play back the contents of the ros2 bag
ros2 bag play rosbag
```
To check the working, in a seperate terminal run
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
# Run the listener in terminal
ros2 run beginner_tutorials listener

