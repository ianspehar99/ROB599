#!/usr/bin/env python3
#HW5 People fning node

# Takes in xy coords of person position, checks if they are within personal space at 
# the edge and marks each invasion of space

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import msg type
from sensor_msgs.msg import LaserScan, PointCloud2

from std_msgs.msg import Header, Bool
 
from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker


# Ivasion node
class InvasionNode(Node):

    def __init__(self):
        # Init the parent class and give it name
        super().__init__('invasion')

        self.get_logger().info('Invasion node started')
        
        # Subscriber to the find_people node for people position
        self.sub = self.create_subscription(Point, 'personposition',self.callback,10)

        # Publisher for if invasion detected (msg type,topic name,queue size)
        self.pub = self.create_publisher(Bool, 'invasionyes',10)

        # Markers for rviz
        self.marker_pub = self.create_publisher(Marker, 'invasion_markers', 10)



        # FIGURE OUT ACTUAL SCALE//UNITS WE ARE IN

        # ------------------------

        # -----------------------

        # --------------------

        # Robot personal space
        self.personal_radius = 100 
        self.take_pic = False

    # Callback for when point is recevied
    def callback(self,msg):
        x = msg.x
        y = msg.y
        dist = np.sqrt(x**2+y**2)

        rmin = self.personal_radius
        # If the person is within radius but close to edge,mark as entering space
        if 0.95*rmin < dist < rmin: 
            # Set publish marker variable

            self.take_pic = True # Set invasion to true for camera node

        # Publish camera picture cmd
        cam_bool = Bool()
        cam_bool = self.take_pic
        self.pub.publish(cam_bool)

        # Publish marker
        marker = Marker()
    
        # ADD TIMER COOLDOWN TOO?

        # SHOW THE RADIUS IN RVIZ
       
#Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = InvasionNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()
