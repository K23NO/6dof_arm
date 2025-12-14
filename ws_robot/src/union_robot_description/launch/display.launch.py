from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_path('union_robot_description')

    default_rviz_config_path = PathJoinSubstitution([str(pkg_share), 'launch', 'urdf.rviz'])

    default_model_path = PathJoinSubstitution([str(pkg_share), 'urdf', 'union_robot_pinzaNodo.urdf'])

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    joint_state_gui_arg = DeclareLaunchArgument(
        name='use_joint_state_publisher_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui for interactive joint control'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    params = {
        'robot_description': robot_description_content,
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher_gui'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        joint_state_gui_arg,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2,
    ])
