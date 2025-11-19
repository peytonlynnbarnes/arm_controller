from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # generate robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_7"),
                    "urdf",
                    "r6bot.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content
    }

    # start gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4"],
        output="screen"
    )

    # spawn robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "r6bot",
            "-topic", "robot_description"
        ],
        parameters=[robot_description],
    )

    # start controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description],
        output="screen"
    )

    # start joint_state_broadcaster
    load_jsb = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster"
        ],
        output="screen"
    )

    # load effort controller?
    load_effort_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "r6bot_controller"
        ],
        output="screen"
    )

    # send one cmd
    send_effort_once = TimerAction(
        period=3.0,   # wait so controller finishes activating
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/r6bot_controller/joint_trajectory",
                    "trajectory_msgs/msg/JointTrajectory",
                    "{header: {}, joint_names: ['joint_1'], points: [{positions: [0.0], "
                    "velocities: [0.0], efforts: [3.0], time_from_start: {sec: 1}}]}"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        controller_manager,
        spawn_robot,
        load_jsb,
        load_effort_controller,
        send_effort_once
    ])
