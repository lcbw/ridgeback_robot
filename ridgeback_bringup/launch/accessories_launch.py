# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    urg_front = Node(
        name='urg_front',
        package='urg_node',
        executable='urg_node',
        output='screen',
        parameters=[{'ip_address': '192.168.131.20',
                     'frame_id': 'front_laser',
                     'angle_min': '-2.35619',
                     'angle_max': '2.35619'}]
        remap=[{'scan': 'front/scan'}])
    urg_rear = Node(
        name='urg_rear',
        package='urg_node',
        executable='urg_node',
        output='screen',
        parameters=[{'ip_address': '192.168.131.21',
                     'frame_id': 'rear_laser',
                     'angle_min': '-2.35619',
                     'angle_max': '2.35619'}]
        remap=[{'scan': 'rear/scan'}])
    lms1xx_front = Node(
        name='lms1xx_front',
        package='lms1xx',
        executable='LMS1xx_node',
        output='screen',
        parameters=[{'host': '192.168.131.20',
                     'frame_id': 'front_laser'}]
        remap=[{'scan': 'front/scan'}])
    lms1xx_rear = Node(
        name='lms1xx_rear',
        package='lms1xx',
        executable='LMS1xx_node',
        output='screen',
        parameters=[{'host': '192.168.131.21',
                     'frame_id': 'rear_laser'}]
        remap=[{'scan': 'rear/scan'}])
    s300_front = Node(
        name='s300_front',
        package='cob_sick_s300',
        executable='cob_sick_s300',
        output='screen',
        parameters=[{'port': '/dev/clearpath/s300_front',
                     'frame_id': 'front_laser'}]
        remap=[{'scan': 'front/scan',
                'scan_standby': '/front/scan_standby'}])
    s300_rear = Node(
        name='s300_rear',
        package='cob_sick_s300',
        executable='cob_sick_s300',
        output='screen',
        parameters=[{'port': '/dev/clearpath/s300_rear',
                     'frame_id': 'rear_laser'}]
        remap=[{'scan': 'rear/scan',
                'scan_standby': '/rear/scan_standby'}])

    rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        parameters=[{'robot_description': urdf}])

    # ******************************************************************
    # Microstrain sensor node
    # ******************************************************************
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('microstrain_inertial_driver'), 'launch'),
                                       '/microstrain_launch.py']), launch_arguments={'namespace': 'microstrain',
                                                                                     'node_name': 'microstrain_inertial_driver',
                                                                                     'params_file': '$(find-pkg-share ridgeback_bringup)/config/microstrain.yaml'}.items(),
    )

    return LaunchDescription([
        urg_front,
        urg_rear,
        lms1xx_front,
        lms1xx_rear,
        s300_front,
        s300_rear.
        launch_include
    ])  # LaunchDescription End
