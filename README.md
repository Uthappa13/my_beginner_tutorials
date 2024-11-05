# ROS 2 Programming Assignment 1 - Publisher/Subscriber

The 'beginner_tutorials' ROS package contains the code for a publisher and a subscriber node. 

First, move this directory to the `src/` folder of your colcon
workspace.  This is desirable or you may get a lot of errors from
`ament_lint_cmake`.

For example, your workspace directory structure should look like this:

```
colcon_ws/
├── build/
├── install/
├── log/
└── src/
    ├── beginner_tutorials/
    ├── ros_package1/
    ├── ros_package2/
    └── ros_package3/
```

``` bash
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/Uthappa13/my_beginner_tutorials.git
```

## How to Compile:
```bash
$ cd ~/colcon_ws/   # assuming your workspace is at '~/colcon_ws'
$ rm -rf install/ build/
$ source /opt/ros/humble/setup.bash  # if needed
$ colcon build 
```

## How to Run:
First, soruce the setup file:
```bash
$ source install/setup.bash
```
### then, run the publisher node
```bash
$ ros2 run beginner_tutorials talker
```
### open another terminal and run the subscriber node
```bash
$ ros2 run beginner_tutorials listener
```

### Check style guidelines
```bash
#In the package directory
cd ~/ros_ws/src/beginner_tutorials

# clang-tidy
$ clang-tidy -p ./ $( find . -name *.cpp | grep -v "/build/" ) > results/clang-tidy.txt

# cpplint
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt