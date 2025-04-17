HW2 Package Instructions

______________________________________________
Commands (run using " ros2 run hw2 <command> " ):

standard_oscope : Produces sin wave with frequency of 1 Hz, publishes at 100 Hz to "oscope" topic
fast_oscope : Same as standard but with f at 2 Hz
slow_oscope : SAme but with f at 0.5 Hz

limiter_node : A pub/sub node that takes data from the "oscope"" topic and crops to [-0.5 0.5]. Publishes on the "cropped_wave" topic

