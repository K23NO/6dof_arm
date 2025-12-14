from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_pkg = get_package_share_directory('union_robot_description')
    urdf_launch = os.path.join(urdf_pkg, 'launch', 'display.launch.py')
    urdf_file = PathJoinSubstitution([urdf_pkg, 'urdf', 'union_robot_pinzaNodo.urdf'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    start_servo_publisher = LaunchConfiguration('start_servo_publisher')
    start_servo_bridge = LaunchConfiguration('start_servo_bridge')
    servo_bridge_input_topic = LaunchConfiguration('servo_bridge_input_topic')

    # Launch arguments keep the file flexible when switching between manual sliders and custom nodes.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time when true.'
    )

    declare_use_joint_state_gui = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='false',
        description='Show joint_state_publisher GUI sliders (off when using hardware feedback).'
    )

    declare_start_servo_publisher = DeclareLaunchArgument(
        'start_servo_publisher',
        default_value='false',
        description='Auto-start lab1/servo_interactive_publisher for CLI angle input.'
    )

    declare_start_servo_bridge = DeclareLaunchArgument(
        'start_servo_bridge',
        default_value='true',
        description='Start lab1/servo_to_jointstate to mirror feedback into joint_states.'
    )

    declare_servo_bridge_input_topic = DeclareLaunchArgument(
        'servo_bridge_input_topic',
        default_value='servo_feedback',
        description='Topic that feeds lab1/servo_to_jointstate (servo_feedback or servo_commands).'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_joint_state_gui,
        declare_start_servo_publisher,
        declare_start_servo_bridge,
        declare_servo_bridge_input_topic,

        # Publicador interactivo o automático de ángulos (ajuste según su caso)
        Node(
            package='lab1',
            executable='servo_interactive_publisher',
            name='servo_interactive_publisher',
            output='screen',
            condition=IfCondition(start_servo_publisher),
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Puente de servo_commands → joint_states para que RViz reciba los ángulos generados por otros nodos.
        Node(
            package='lab1',
            executable='servo_to_jointstate',
            name='servo_to_jointstate',
            output='screen',
            condition=IfCondition(start_servo_bridge),
            parameters=[
                {'use_sim_time': use_sim_time},
                {'feedback_topic': servo_bridge_input_topic},
                {'command_topic': 'servo_commands'}
            ]
        ),

        # robot_state_publisher + RViz (de union_robot_description)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urdf_launch),
            launch_arguments={
                'model': urdf_file,
                'use_sim_time': use_sim_time,
                'use_joint_state_publisher_gui': use_joint_state_gui
            }.items()
        ),
    ])

