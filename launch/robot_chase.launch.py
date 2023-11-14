from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_rick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_rick',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'morty/odom']
    )

    static_tf_morty = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_morty',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'rick/odom']
    )

    robot_chase_node = Node(
        package='robot_chase',  
        executable='robot_chase',
        name='robot_chase',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    return LaunchDescription([
        static_tf_morty,
        static_tf_rick,
        robot_chase_node
    ])
