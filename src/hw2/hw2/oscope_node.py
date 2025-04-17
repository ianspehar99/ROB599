#!/usr/bin/env python3

#This file contains:

# oscope - publisher node, sin wave of 1 Hz, published at 100 Hz
# slow_wave,fast_wave - publisher nodes, same as oscope but at 0.5 and 2 Hz

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import msg type
from std_msgs.msg import Float32

# Oscope node
class OscopeNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self,frequency):
        # Init the parent class and give it name
        super().__init__('oscope_publisher')

        #create publisher with (msg type,topic name,queue size)
        self.pub = self.create_publisher(Float32, 'oscope',10)

        #Ceate timer, uses period, so inverse of frequency
        freq = 100 #Publish at 100 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.callback)

        #Set sin frequency
        self.frequency = frequency

        #Initialize start time for sin calc
        self.start_time = self.get_clock().now()

    def callback(self):
        # Access frequency from class call
        frequency = self.frequency

        #Get current time
        t = self.get_clock().now() - self.start_time
        t = t.nanoseconds/(1e9) #Convert to seconds

        #Init msg type
        msg = Float32()

        #Set msg
        msg.data = np.sin((2*np.pi*frequency)*t)

        #Publish msg
        self.pub.publish(msg)

        #Print published wave data
        self.get_logger().info('Wave data: "%s"' % msg.data)

#Now going to create 3 different main functions that call OscopeNode for each frequency
#In setup.py, will name each one differently so we can execute easily by name

#Main function for when f = 1 Hz
def oscope_main(args=None):
    frequency = 1  # 1 Hz for basic 
    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = OscopeNode(frequency)

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()

#Repeat for fast wave:
def fast_wave_main(args=None):
    frequency = 2  # 2 Hz for fast
    
    rclpy.init(args=args)

    publisher = OscopeNode(frequency)

    rclpy.spin(publisher)

    rclpy.shutdown()

#Then for slow wave:
def slow_wave_main(args=None):
    frequency = 0.5  # 0.5 Hz for slow
    
    rclpy.init(args=args)

    publisher = OscopeNode(frequency)

    rclpy.spin(publisher)

    rclpy.shutdown()










