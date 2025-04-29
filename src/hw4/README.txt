Ian Spehar ROB599 HW4

This HW includes:


------ INSTRUCTIONS ------
1. To run 3 oscope nodes at 1,5,10 HZ, with 10 Hz oscope clamped at 0.7

- Run launch file: "ros2 launch hw4 wave.py"

- Then, copy this code into terminal to start all nodes w service:
 ros2 service call oscope1/oscopestart services_messages/srv/SendData "{send: True}"
 ros2 service call oscope5/oscopestart services_messages/srv/SendData "{send: True}"
 ros2 service call oscope10/oscopestart services_messages/srv/SendData "{send: True}"







Sources:
- Class examples
- Code from last 2 hws
- https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html
