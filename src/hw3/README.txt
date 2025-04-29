Ian Spehar 599 HW3 

This HW includes:
- "data_sender.py": node that pubs TestPacket msg types that include data and send_time
   Uses service "SendData.srv" to start and stop data flow
-"data_receiver.py": node that subs to the sender, cals the latency time and pubs raw and avg latency
  Uses service "EnableLogging.srv" to start/stop logging and set csv file to log to


---------- Instructions -------------

1. Start the sender node: ros2 run hw3 sender
2. Start receiver node: ros2 run hw3 receiver
3. Use service call to start data stream from sender: 
    ros2 service call /senddata services_messages/srv/SendData "{send: True}"   (Or send: False to stop)
4. Use service call to start receiver node logging
    ros2 service call /logdata services_messages/srv/EnableLogging "{logging: True, filename: 'datalog.csv'}"







Sources:
- Class examples
- Node outlines from last assignment
- https://docs.python.org/3/library/csv.html#writer-objects
- https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
- https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html