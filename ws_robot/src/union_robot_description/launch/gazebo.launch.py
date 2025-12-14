from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_path('union_robot_description')
    gazebo_pkg = get_package_share_path('gazebo_ros')

    default_model = PathJoinSubstitution([str(pkg_share), 'urdf', 'union_robot_pinzaNodo.urdf'])

    model = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the robot URDF file'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]),
            value_type=str
        ),
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(gazebo_pkg / 'launch' / 'gazebo.launch.py')])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'union_robot'],
        output='screen'
    )

    return LaunchDescription([
        model,
        use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
