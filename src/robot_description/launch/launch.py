import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_description = {'robot_description': robot_description_content}

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, '..')
    )

    # 1. Gazebo Server & GUI
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 empty.sdf'], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'custom_robot', '-x', '0.0', '-y', '0.0', '-z', '0.01']
    )

    # 4. ROS-Gazebo Bridge (không bridge /tf để EKF tự phát TF odom -> base_footprint)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom_gazebo@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ]
    )

    # 5. Bộ lọc EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
            get_package_share_directory('agv_ekf'),
            'config',
            'ekf.yaml'
            )
        ]
    )

    # 6. SLAM Toolbox (Online Async)
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config
        }.items()
    )

    # 7. Navigation2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_config
        }.items()
    )

    # 8. Encoder Simulator
    encoder_sim_node = Node(
        package='agv_base_interface',
        executable='encoder_sim_node',
        name='encoder_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ticks_per_wheel_rev': 2048,
            'left_joint': 'wheel_left_joint',
            'right_joint': 'wheel_right_joint',
            'input_topic': '/joint_states',
            'output_topic': '/wheel/encoder_states'
        }]
    )

    # 9. Wheel Odometry
    wheel_odometry_node = Node(
        package='agv_base_interface',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.0425,
            'wheel_separation': 0.250,
            'left_joint': 'wheel_left_joint',
            'right_joint': 'wheel_right_joint',
            'input_topic': '/wheel/encoder_states',
            'output_topic': '/odom'
        }]
    )


    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(bridge_node)
    ld.add_action(ekf_node)
    ld.add_action(slam_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(encoder_sim_node)
    ld.add_action(wheel_odometry_node)

    return ld