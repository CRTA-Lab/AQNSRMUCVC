from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the launch file in another package
   
    twist_mux_launch = os.path.join(
        get_package_share_directory('twist_mux_test'),  # The package name
        'launch',  # The 'launch' folder in the other package
        'twist_mux_test.launch.py'  # The name of the launch file in the other package
    )
    teleop_launch = os.path.join(
        get_package_share_directory('tello_driver'),  # The package name
        'launch',  # The 'launch' folder in the other package
        'teleop_launch.py'  # The name of the launch file in the other package
    )
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),  # The package name
        'launch',  # The 'launch' folder in the other package
        'realsense_launch.py'  # The name of the launch file in the other package
    )
    joy_control_launch = os.path.join(
        get_package_share_directory('joy_control'),  # The package name
        'launch',  # The 'launch' folder in the other package
        'joy_control_launch.py'  # The name of the launch file in the other package
    )
    
    # Create the LaunchDescription object to include both launch files
    return LaunchDescription([
        # Include the other package's launch file
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(twist_mux_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
        ),
        
        Node(
            package='joy_filter', 
            executable='joy_filter', 
            output='screen'
            ),
       
    ])