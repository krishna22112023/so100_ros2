import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Where is the XML scene file?
    directory = get_package_share_directory('so100_ros_controller')
    xmlScenePath = os.path.join(directory, 'model', 'scene.xml')  # absolute path inside the package

    if not os.path.exists(xmlScenePath):
        raise FileNotFoundError(f"Scene file does not exist: {xmlScenePath}.")

    # ------------------- launch-time arguments -------------------
    joint_cmd_arg = DeclareLaunchArgument(
        'joint_command_topic_name',
        default_value='position_command',
        description='Topic on which mujoco_ros listens for joint commands'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='POSITION',
        description='Control mode: POSITION | VELOCITY | TORQUE'
    )

    # ------------------- node definitions -----------------------
    mujoco = Node(
        package='so100_ros_controller',
        executable='mujoco_ros',
        output='screen',
        arguments=[xmlScenePath],  # first positional CLI arg consumed by main()
        parameters=[{
            'joint_command_topic_name': LaunchConfiguration('joint_command_topic_name'),
            'control_mode': LaunchConfiguration('control_mode'),
        }]
    )

    position_commander = Node(
        package    = "so100_ros_controller",
        executable = "position_commander",
        output     = "screen"
    )

    return LaunchDescription([
        joint_cmd_arg,
        control_mode_arg,
        mujoco,
        position_commander,
    ])
