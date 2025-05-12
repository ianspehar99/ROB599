#!/usr/bin/env python3
#HW5 People fning node

# Subscribes to bool from invasion node, takes picture when true
# Just saves pic from bag, no publishing
# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import msg type
from sensor_msgs.msg import Image

from std_msgs.msg import Header, Bool
 
from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker


# Picture node
class RedhandedNode(Node):

    def __init__(self):
        # Init the parent class and give it name
        super().__init__('redhanded')

        self.get_logger().info('Picture node started')
        
        # Subscriber to invasion node, get bool
        self.sub = self.create_subscription(Bool, 'invasionyes',self.callback,10)

        # Subscriber to the find_people node for people position
        self.sub = self.create_subscription(Image, 'invasionyes',self.callback,10)

        # Publisher for if invasion detected (msg type,topic name,queue size)
        self.pub = self.create_publisher(Bool, 'invasionyes',10)

        # Markers for rviz
        self.marker_pub = self.create_publisher(Marker, 'invasion_markers', 10)



    # Callback to save pic
    def callback(self,msg):

        
        
       


#Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = RedhandedNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()
