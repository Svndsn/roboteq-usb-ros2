from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='roboteq_node_ros2',
            executable='roboteq_node_ros2_executable',
            name='roboteq_serial_node',
            parameters=[
                {'mode': 'can'},
                {'device': '/dev/serial/by-id/usb-Roboteq_Motor_Controller_SDC2XXX-if00'},
                {'left': '1'},
                {'right': '2'},
                {'cmd_vel_topic': 'cmd_vel'}
            ]
        )
    ])