import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.expanduser('~/gazebo/worlds/my_world.sdf')

    # Gazebo models environment
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo, 'models')
    )

    # 1. Starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v2 {world_path}'}.items()
    )

    # 2. bridge
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
    # Making sure names match exactly
    remappings=[
        ('/model/turtlebot3_burger/scan', '/scan'),
    ],
    output='screen'
)

    # 3. Spawn TurtleBot
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        bridge,
        spawn_tb3
    ])