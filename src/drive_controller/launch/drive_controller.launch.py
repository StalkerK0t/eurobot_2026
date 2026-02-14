from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    drive_params = PathJoinSubstitution(
        [FindPackageShare("drive_controller"), "config", "plan.yaml"]
    )    

    return LaunchDescription([
        Node(
            package='drive_controller',
            executable='drive_controller',
            name='drive_controller',
            output='screen',
            parameters=[drive_params],            
        ),      
    ])