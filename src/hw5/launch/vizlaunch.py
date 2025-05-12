# Launch file for HW5 Rviz2 People Identification
# vizlaunch.py
# Ian Spehar

#ROS launch imports
import launch
import launch_ros.actions
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():

    # Get rviz config path wherever we are
    package_path = get_package_share_directory('hw5')
    rviz_path = os.path.join(package_path,'rviz','customconfig.rviz')
    
    return launch.LaunchDescription([
        # 

        LogInfo(msg=['RViz config path: ', rviz_path]),

        launch_ros.actions.Node(
            package='hw5',
            executable='laser_filter',
            name='scan_filter',
            parameters = [
                {'threshold': 10}
            ]
        ),

        # Rviz config
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path]
        )
    ])