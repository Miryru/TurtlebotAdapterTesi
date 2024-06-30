import launch

import launch_ros.actions

def generate_launch_description():
    # Create a launch description object
    ld = launch.LaunchDescription()

    # Launch Turtlebot Adapter 1
    turtlebot_adapter1_node = launch_ros.actions.Node(
        package='turtlebot_adapter',
        executable='turtlebot_adapter',
        name='turtlebot_adapter',
        output='screen'
    )
    ld.add_action(turtlebot_adapter1_node)

    # Launch Turtlebot Adapter 2
    turtlebot_adapter2_node = launch_ros.actions.Node(
        package='turtlebot_adapter',
        executable='turtlebot_adapter2',
        name='turtlebot_adapter2',
        output='screen'
    )
    ld.add_action(turtlebot_adapter2_node)

    return ld