# Launch file for HW4 Action server and client
# launchlaunch.py
# Ian Spehar

#ROS launch imports
import launch
import launch_ros.actions

# Return launch description containing each node to run
def generate_launch_description():
    return launch.LaunchDescription([
        # Set nodes to run

        # Action client
        launch_ros.actions.Node(
            package='hw4',
            executable='actionclient',
            name='action_client',
        ),

        # Action server
        launch_ros.actions.Node(
            package='hw4',
            executable= 'actionserver',
            name='action_server',
        ),
    ])