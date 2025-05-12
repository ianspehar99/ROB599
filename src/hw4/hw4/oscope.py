#!/usr/bin/env python3

# HW4 - Updated oscope node from HW2, takes in frequency, clamp as params during launch call

#This file contains:

# oscope - publisher node
# slow_wave,fast_wave - publisher nodes, same as oscope but at 0.5 and 2 Hz

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import srv for turning data sending on/off
from services_messages.srv import SendData

# Import msg type
from std_msgs.msg import Float32

# Oscope node
class OscopeNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('updated_oscope')

        # ____________________________________________________
        # UPDATES: Parameters/service call to pub to
        # 1. Declare params (set default)
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('clamp',1000.0) #Functionally no clamp as default

        #Set as variables
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.clamp = self.get_parameter('clamp').get_parameter_value().double_value
    
        # 2. Bool to wait for service call:
        self.switch = False

        self.get_logger().info('Oscope node started, waiting for service call')
        # ____________________________________________________
        
        #create publisher with (msg type,topic name,queue size)
        self.pub = self.create_publisher(Float32, "oscope",10)

        # Create service  (type,ros2 callname,callback)
        self.service = self.create_service(SendData, f'{self.get_name()}/oscopestart', self.service_callback)

        #Ceate timer, uses period, so inverse of frequency
        freq = 5 #Publish at 100 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.callback)

        #Initialize start time for sin calc
        self.start_time = self.get_clock().now()
    
    def callback(self):
        pub_data = self.switch

        # Wait for service
        if pub_data:
            # Get frequency and clamp params from init
            frequency = self.frequency
            clamp = self.clamp
            

            #Get current time
            t = self.get_clock().now() - self.start_time
            t = t.nanoseconds/(1e9) #Convert to seconds

            #Init msg type
            msg = Float32()

            #Set msg
            msg.data = np.sin((2*np.pi*frequency)*t)

            # Clamp if its over, will expect clamp as positive num
            if abs(msg.data) > abs(clamp):  
                # Now check if it was neg or ps
                if msg.data < 0 :   
                    msg.data = -clamp
                else:
                    msg.data = clamp


            #Publish msg
            self.pub.publish(msg)

            #Print published wave data
            self.get_logger().info('Wave data: "%s"' % msg.data)

    def service_callback(self, request, response):
        
        # Access input from service call/client
        send_bool = request.send 

        # Set variable so that the publisher callback can access it
        self.switch = send_bool
        
        # Set response so that we can see it was received and whats happening
        if send_bool:
            response.update = "Service call received, node has been started"
        else:
            response.update = "Service call received, node has been stopped"

        return response

#Now going to create 3 different main functions that call OscopeNode for each frequency
#In setup.py, will name each one differently so we can execute easily by name

#Main function
def main(args=None):
    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = OscopeNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()












