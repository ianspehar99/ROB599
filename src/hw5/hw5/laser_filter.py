#!/usr/bin/env python3
#HW5 LAser filter node

# Takes in laser scan data, sets current scan at time of service snapshot to be background
# Filters those points out of the scan data that comes after that

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import srv for turning data sending on/off
from services_messages.srv import SetFilter

# Import msg type
from sensor_msgs.msg import LaserScan

# Laser filter node
class FilterNode(Node):
    # Init
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('scan_filter')

        # 1. Declare param (set default)
        self.declare_parameter('threshold',10)  #This determines how close the scan has to be to be considered same as noise
    
        #Set as variable
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.get_logger().info('Filter node started')
       
        # Subscriber to laser scan
        self.sub = self.create_subscription(LaserScan, 'LaserScan',self.callback,10)

        # Publisher for filtered scandata with (msg type,topic name,queue size)
        self.pub = self.create_publisher(LaserScan, 'filterscan',10)

        # Create service  (type,ros2 callname,callback)
        self.service = self.create_service(SetFilter, 'filtering', self.service_callback)

        # Variables:
        self.reference_data = None # Background that we set when service is called
        self.service_bool = None # See if service called yet


    def callback(self,msg):

        # Check if service has been activated
        snapshot_activated = self.service_bool

        # Set array
        laser_array = msg.ranges

        # Store in class in case the service callback is activated
        self.latest_scan = list(msg.ranges)

        first_angle = msg.angle_min # *********MIGHT WANT TO USE THIS LATER FOR TURNING TO 2D POINTS/CLUSTERING AND SHI ***********

        # Wait for service to start filtering
        if snapshot_activated is True:

            # Get threshold from inti/param set
            threshold = self.threshold

            # Acces reference scan from self
            ref_scan = self.reference_data

            # Remove values that are within threshold of reference scan
            for i in range(len(laser_array)):
                ref_val = ref_scan[i]
                current_val = laser_array[i]

                if abs(ref_val - current_val) > threshold:

                    laser_array[i] = float('inf')

            # Set msg
            msg.ranges = laser_array

            #Publish msg every time we get data
            self.pub.publish(msg)

            #Print published wave data
            self.get_logger().info('Filtered scan: "%s"' % msg.ranges)

        
        else:
            # Just publish the OG data
            self.pub.publish(msg)

            self.get_logger().info('Raw scan: "%s"' % msg.ranges)



    def service_callback(self, request, response):
        
        # Access input from service call/client
        send_bool = request.setfilter

        # Set class variable for main callback
        self.service_bool = send_bool
        
        # Set response so that we can see it was received and whats happening
        if send_bool:

            # Set reference data as most recent data at time of service call:
            self.reference_data = self.latest_scan
            
            response.update = "Service call received, snapshot set and filtering started"
        else:
            response.update = "Set_filter needs to be True for service to work"

        return response


#Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = FilterNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()

