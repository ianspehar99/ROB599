Ian Spehar ROB599 HW4

This HW includes:


------ INSTRUCTIONS ------
1. To run 3 oscope nodes at 1,5,10 HZ, with 10 Hz oscope clamped at 0.7

- Run launch file: " ros2 launch hw4 wave.py "

- Then use these service calls in a sperate terminal to start each oscope (may cause issues if you do them all at once):

 ros2 service call /oscope1/oscopestart services_messages/srv/SendData "{send: True}"
 ros2 service call /oscope5/oscopestart services_messages/srv/SendData "{send: True}"
 ros2 service call /oscope10/oscopestart services_messages/srv/SendData "{send: True}"


2. To run the launch sequence (action server and action client) :

- Run launch file " ros2 launch hw4 launchlaunch.py "

- To abort, use the service call:

ros2 service call /abortlaunch services_messages/srv/SendData "{send: True}"






Sources:
- Class examples
- Code from last 2 hws
- https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html
