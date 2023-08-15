import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# LINKS:
# https://answers.ros.org/question/374926/ros2-how-to-launch-rviz2-with-config-file/
# https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/multi_tb3_simulation_launch.py


def launch_setup(context: LaunchContext, *args):

    marker_id = LaunchConfiguration('marker_id').perform(context)
    marker_size = LaunchConfiguration('marker_size').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    publish_base_pose = LaunchConfiguration('publish_base_pose').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    
    executables = []

    # robot state publisher
    with open(urdf_file, 'r') as infp: 
        robot_desc = infp.read()

    # robot state publisher node
    executables.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace = camera_name,
        parameters=[{'robot_description': robot_desc}]))
    
    executables.append(Node(
        namespace = camera_name,
        package = 'aruco_detector',
        executable = 'aruco_detector',
        parameters = [{'marker_size': float(marker_size), 'marker_id': int(marker_id), 'calibration_file': calibration_file, 'camera_name': camera_name}],
        remappings = [('/my_camera/pylon_ros2_camera_node/image_raw', '/' + camera_name + '/pylon_ros2_camera_node/image_raw')]
    ))

    executables.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pylon_ros2_camera_wrapper'),'launch', 'pylon_ros2_camera.launch.py')),
                launch_arguments={'config_file': config_file,
                                  'camera_id': camera_name,
                                  'node_name': 'pylon_ros2_camera_node'}.items()))

    if bool(publish_base_pose):
        executables.append(Node(
            package = 'aruco_detector',
            executable = 'base_pose_publisher'
        ))

    return executables

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('publish_base_pose',
                              default_value = '1'),
        DeclareLaunchArgument('camera_name', 
                              default_value = 'camera', 
                              description = 'Name of the camera'),
        DeclareLaunchArgument('marker_size', 
                              default_value = '0.013', 
                              description = 'Square size in m of the aruco marker'),
        DeclareLaunchArgument('marker_id', 
                              default_value = '0', 
                              description = 'ID of the aruco marker'),
        DeclareLaunchArgument('calibration_file', 
                              default_value = os.path.join(get_package_share_directory('aruco_detector'), 'calibration/default.yaml'),
                              description = 'Location of the camera calibration file'),
        DeclareLaunchArgument('config_file', 
                              default_value = os.path.join(get_package_share_directory('aruco_detector'), 'config/default.yaml'),
                              description = 'Location of the camera configuration file'),
        DeclareLaunchArgument('urdf_file', 
                              default_value = os.path.join(get_package_share_directory('aruco_detector'), 'urdf/camera.urdf'),
                              description = 'Location of the camera urdf file'),
        OpaqueFunction(function = launch_setup)
                              ])
