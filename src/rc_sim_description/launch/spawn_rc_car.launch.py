from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rc_sim_description")

    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "basic_track.world"]
    )
    default_rviz = PathJoinSubstitution(
        [pkg_share, "config", "rviz", "rc_car.rviz"]
    )
    robot_xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "rc_car.urdf.xacro"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Command expects a space between executable and path
    robot_description = Command(["xacro ", robot_xacro])

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        name="rc_car_joint_state_publisher",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rc_car_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    spawn_rc = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rc_car",
            "-topic",
            "robot_description",
            "-x",
            x_pos,
            "-y",
            y_pos,
            "-z",
            z_pos,
        ],
        output="screen",
    )

    # Small delay so gz sim is running before spawning the model
    spawn_delayed = TimerAction(period=2.0, actions=[spawn_rc])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rc_car_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulated clock",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world to load",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="0.0",
                description="Initial X position",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0.0",
                description="Initial Y position",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.1",
                description="Initial Z position",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz with the description",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz configuration file",
            ),
            gz_sim,
            joint_state_publisher,
            robot_state_publisher,
            spawn_delayed,
            rviz_node,
        ]
    )
