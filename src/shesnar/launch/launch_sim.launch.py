import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


from launch import Action
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as rclpyNode
import threading
import time
from typing import List


class WaitForTopic(Action):
    def __init__(
        self,
        topic_name: SomeSubstitutionsType,
        timeout: float = 30.0,
        node_name: str = 'topic_waiter_node'
    ) -> None:
        super().__init__()
        self.__topic_name = normalize_to_list_of_substitutions(topic_name)
        self.__timeout = timeout
        self.__node_name = node_name

def execute(self, context: LaunchContext) -> None:
    # Perform all substitutions in the list and join them into a single string
    topic_name = ''.join([context.perform_substitution(sub) for sub in self.__topic_name])
    
    rclpy.init()
    node = rclpyNode(self.__node_name)

    def wait_for_topic():
        start = time.time()
        while (time.time() - start) < self.__timeout:
            topic_names_and_types = node.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_names_and_types]
            if topic_name in topic_names:
                node.get_logger().info(f"Detected topic '{topic_name}', continuing launch...")
                rclpy.shutdown()
                return
            time.sleep(0.5)
        node.get_logger().warn(f"Timeout waiting for topic '{topic_name}'")
        rclpy.shutdown()

    wait_thread = threading.Thread(target=wait_for_topic)
    wait_thread.start()
    wait_thread.join()


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='shesnar' #<--- CHANGE ME

    build_map = LaunchConfiguration('build_map')
    is_localization = LaunchConfiguration('is_localization')    
    map_file_path = LaunchConfiguration('map')    

    declare_build_map_cmd = DeclareLaunchArgument(
        'build_map', default_value='false', description='build map'
    )

    declare_localization_cmd = DeclareLaunchArgument(
        'is_localization', default_value='false', description='localization'
    )

    default_map = os.path.join(
            get_package_share_directory(package_name),
            'maps',
            'euro_map.yaml'
            )     
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map, description='Full path to map yaml file to load'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    config_ekf= os.path.join(get_package_share_directory("shesnar"),'config','ekf_params.yaml')

    ekf = Node(
        package = 'robot_localization',
        name = 'ekf_filter_node',
        executable = 'ekf_node',
        output="screen",
        parameters=[config_ekf],
    )


    wait_for_camera = WaitForTopic(topic_name='/camera_odom', timeout=20.0)

    ekf_after_camera = [
        wait_for_camera,
        ekf
    ]      

    default_world = os.path.join(
            get_package_share_directory(package_name),
            'worlds',
            'complete_world.world'
            )    
       
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    # launch_arguments={'gz_args': ['-r --render-engine ogre -v4 ', world], 'on_exit_shutdown': 'true'}.items()
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                #    '-name', 'my_bot', "-z", '0.111' , "-y", '-1.1', "-x", '0.75' ],
                                            '-name', 'my_bot', "-z", '0.111' , "-y", '-0.3', "-x", '1.0' ],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]        
    )

    move_control = Node(
        package="cpp_omnidrive_gazebo_controller",
        executable="omni_gz_con",
        arguments=[
 
        ],
        parameters = [{'use_sim_time': True}]
    )

    start_camera_node = Node(
        package="camera_cus",
        executable="camera_node",
        arguments=[
        ],
        parameters = [{'use_sim_time': True}]
    )


    # # to do map 
    # slam_params = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','online_async_launch.py'
    #             )]),
    #     # condition=IfCondition( build_map ),        
    #     launch_arguments={
    #         'use_sim_time': True,
    #         'params_file': slam_params,
    #     }.items()
    # )

    nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    start_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization_launch.py'
                )]), 
                # condition=IfCondition( is_localization ), 
                launch_arguments={'map': map_file_path, 'use_sim_time': 'true', 'params_file': nav_params}.items()
    )

    # start_localization = IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             PathJoinSubstitution(
    #                                 [
    #                                     FindPackageShare("lidar_localization"),
    #                                     "launch",
    #                                     "lidar_localization.launch.py",
    #                                 ]
    #                             )),
    #                             launch_arguments={'map': map_file_path, 'use_sim_time': 'true'}.items()
    #                             )
    
    nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    start_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), 
                # condition=IfCondition( is_navigation ), 
                launch_arguments={'use_sim_time': 'true', 'map_subscribe_transient_local': 'true', 'params_file': nav_params}.items()
    )
    
    drive_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("drive_controller"),
                "launch",
                "drive_controller.launch.py",
            ])
        ])  
    )


    # Launch them all!
    return LaunchDescription([        
        world_arg,
        declare_build_map_cmd,
        declare_localization_cmd,
        declare_map_yaml_cmd,

        rsp,        
        gazebo,
        ros_gz_bridge,
        spawn_entity,
        move_control,
        start_camera_node,

        *ekf_after_camera,

        start_localization,
        start_navigation,

        # drive_controller_launch,
    ])