import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # --- MoveIt Config (SRDF/kinematics etc). URDF is created below via xacro -> robot_description ---
    moveit_config = (
        MoveItConfigsBuilder("karura_arm_2026", package_name="karura_arm_moveit_config")
        # Explicitly specify SRDF filename as karura_arm_description.srdf to suppress warnings
        .robot_description_semantic(file_path="config/karura_arm_description.srdf")
        .to_moveit_configs()
    )

    # --- Generate robot_description by expanding xacro from the description package ---
    desc_share = get_package_share_directory("karura_arm_description")
    xacro_file = os.path.join(desc_share, "urdf", "karura_arm_2026.xacro")
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    # Get parameters for the Servo node
    servo_yaml = load_yaml("karura_arm_moveit_config", "config/simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("karura_arm_description") + "/config/display.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
        ],
    )
    static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_to_base_link_broadcaster",
    arguments=["0","0","0","0","0","0","world","base_link"],
    output="screen",
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("karura_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_controller", "-c", "/controller_manager"],
    )
    arm_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=["arm_velocity_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            # ComposableNode(
            #     package="joy",
            #     plugin="joy::Joy",
            #     name="joy_node",
            # ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    joint_array_translation = Node(
        package="send_target_joint_degrees",
        executable="send_target_joint_degrees",
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            #arm_velocity_spawner,
            servo_node,
            container,
            joint_array_translation,
            static_tf
        ]
    )
