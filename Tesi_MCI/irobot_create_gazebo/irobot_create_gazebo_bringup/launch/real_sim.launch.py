from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration , PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the namespace argument
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='default_ns',
        description='Namespace of the robot'
    )

    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    map_file = pkg_create3_gazebo_bringup + '/maps/map_aula.yaml'
    # Get the path to the launch file
    view_robot_launch_file_dir = get_package_share_directory('turtlebot4_viz')
    robot_nav_dir = get_package_share_directory('turtlebot4_navigation')
    localization_launch = PathJoinSubstitution([robot_nav_dir, 'launch', 'localization.launch.py'])
    view_robot_launch_file = PathJoinSubstitution([view_robot_launch_file_dir, 'launch', 'view_robot.launch.py'])
    nav2_launch = PathJoinSubstitution([robot_nav_dir, 'launch', 'nav2.launch.py'])

    include_localization_cmd = IncludeLaunchDescription(
        localization_launch,
        launch_arguments={'namespace': LaunchConfiguration('namespace'),'map':map_file}.items()
    )
    
    # Include the launch file and pass the namespace argument
    include_view_robot_cmd = IncludeLaunchDescription(
        view_robot_launch_file,
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )
    
    include_nav2_cmd = IncludeLaunchDescription(
        nav2_launch,
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    return LaunchDescription([
        declare_namespace_cmd,
        include_view_robot_cmd,
        include_localization_cmd,
        include_nav2_cmd
    ])
