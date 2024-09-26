from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # declare_verbose_cmd = DeclareLaunchArgument(
    #     name='verbose',
    #     default_value='false',
    #     description='Verbose mode')
    
    # declare_points_in_cmd = DeclareLaunchArgument(
    #     name='points_in',
    #     default_value='/gazebo/points',
    #     description='Input point cloud topic')
    
    # declare_odom_in_cmd = DeclareLaunchArgument(
    #     name='odom_in',
    #     default_value='/chotto/fmu/out/vehicle_odometry',
    #     description='Input odometry topic')

    
    offboard_control = Node(
        package='offboard_companion',
        namespace='offboard_companion',
        executable='offboard_control.py',
        name='offboard_control',
        output="screen",
        emulate_tty=True,
        # arguements/parameters
        parameters=[
            # {"points_in": LaunchConfiguration('points_in')},
            # {"odom_in": LaunchConfiguration('odom_in')},
            # {"verbose": LaunchConfiguration('verbose')},
        ],
    )

    relay_simulazione = Node(
        package='offboard_companion',
        namespace='offboard_companion',
        executable='relay_simulazione.py',
        name='relay_simulazione',
        output="screen",
        emulate_tty=True,
        # arguements/parameters
        parameters=[
            # {"points_in": LaunchConfiguration('points_in')},
            # {"odom_in": LaunchConfiguration('odom_in')},
            # {"verbose": LaunchConfiguration('verbose')},
        ],
    )
    
    ld = LaunchDescription()
    
    # ld.add_action(declare_verbose_cmd)
    # ld.add_action(declare_points_in_cmd)
    # ld.add_action(declare_odom_in_cmd)
    ld.add_action(offboard_control)
    ld.add_action(relay_simulazione)
    return ld