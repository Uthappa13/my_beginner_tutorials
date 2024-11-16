from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
 
def generate_launch_description():
    ros2_bag_start = LaunchConfiguration('ros2_bag_start')
    publisher_freq_launch_arg = DeclareLaunchArgument(
        "publish_freq", default_value=TextSubstitution(text="500")
    )
    ros2_bag_start_arg = DeclareLaunchArgument(
        "ros2_bag_start", 
        default_value='False'
    )
    talker_node = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{"publish_frequency": LaunchConfiguration('publish_freq')}]
    )
    run_ros2bag_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                ros2_bag_start
            ])
        ),
        cmd=['ros2', 'bag', 'record', '-o', 'rosbag', '-a'],
        output='screen'
    )

    bag_event_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=talker_node,
                on_start=[
                    run_ros2bag_conditioned
                ]
            )
        )

    return LaunchDescription([publisher_freq_launch_arg, ros2_bag_start_arg, talker_node, bag_event_handler])

