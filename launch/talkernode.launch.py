from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    publisher_freq_launch_arg = DeclareLaunchArgument(
        "publish_freq", default_value=TextSubstitution(text="500")
    )
    talker_node = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{"publish_frequency": LaunchConfiguration('publish_freq')}]
    )
    return LaunchDescription([publisher_freq_launch_arg, talker_node])