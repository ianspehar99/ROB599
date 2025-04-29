# Launch file for HW4 Oscope node
# wave.py
# Ian Spehar

#ROS launch imports
import launch
import launch_ros.actions

# Return launch description containing each node to run
def generate_launch_description():
    return launch.LaunchDescription([
        # Set nodes to run

        # 1 Hz
        launch_ros.actions.Node(
            package='hw4',
            executable='oscope',
            name='oscope1',
            remappings=[
                ('oscope', 'oscope_1hz')
            ],
            parameters = [
                {'frequency':1.0}
            ]
        ),

        # 5hz
        launch_ros.actions.Node(
            package='hw4',
            executable='oscope',
            name='oscope5',
            remappings =[
                ('oscope','oscope_5hz')
            ],
            parameters = [
                {'frequency':5.0}
            ]
        ),

        # 10 Hz, clamp at 0.7
        launch_ros.actions.Node(
            package='hw4',
            executable='oscope',
            name='oscope10',
            remappings =[
                ('oscope','oscope_10hz')
            ],
            parameters = [
                {'frequency':10.0,'clamp':0.7}
            ]
        ),

        # PlotJuggler 
        launch_ros.actions.Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
        ),
    ])