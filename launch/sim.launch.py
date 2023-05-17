from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # add log level argument
    
    logger_arg = LaunchConfiguration("log_level", default="info")
    
    log_level = DeclareLaunchArgument(
            "log_level",
            default_value=logger_arg,
            description="Logging level",
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("f1tenth_hardware_interface"), "urdf", "f1tenth.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_hardware_interface"),
            "config",
            "f1tenth_controllers.yaml",
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("f1tenth_hardware_interface"),
            "rviz",
            "sim.rviz"
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # remappings=[
        #     "/ackermann_steering_controller/reference", ""
    )
    
    static_tf_pub_node1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=["0", "0", "0", "0", "0", "1.5708", "left_steering_hinge", "left_front_wheel"],
    )
    
    static_tf_pub_node2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=["0", "0", "0", "0", "0", "1.5708", "right_steering_hinge", "right_front_wheel"],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", logger_arg],
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", logger_arg],
    )
    
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    relay_topic_to_tf_node = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
        output='screen',
    )
    
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
    )
    
    nodes = [
        log_level,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        static_tf_pub_node1, 
        static_tf_pub_node2,
        relay_topic_to_tf_node,
        rqt_node
    ]
    
    return LaunchDescription(nodes)