# README - Instructions for HW5

# filter node: ros2 run hw5 laser_filter
# Play the bag file:
PART 1:
1. ros2 bag play /home/spehari/Downloads/hw5_rosbag_quori/hw5_good_good/
(On your computer, find it and copy the path to replace)

2. launch file: ros2 launch hw5 vizlaunch.py

service for setting filter point:
3. ros2 service call /filtering services_messages/srv/SetFilter "{setfilter: True}"