#!/usr/bin/env python3
#HW5 People fning node

# Takes in laser scan data, converts to xy coords, and does point clustering to estimate position of people

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node
import numpy as np

# Import msg type
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker

from std_msgs.msg import Header

from sklearn.cluster import DBSCAN


# People identification node
class PeopleNode(Node):

    def __init__(self):
        # Init the parent class and give it name
        super().__init__('find_people')

        self.get_logger().info('FInd people node started')
        
        # Subscriber to laser scan
        self.sub = self.create_subscription(LaserScan, 'LaserScan',self.callback,10)

        # Publisher for position of peope with (msg type,topic name,queue size)
        self.pub = self.create_publisher(Point, 'personposition',10)

    def callback(self,msg):

        # Set array
        scans = msg.ranges

        # Get first angle
        first_angle = msg.angle_min 

        # Get angle increment between each lidar reading
        angle_increment = msg.angle_increment

        #SEt up xy array
        xy = []
        
        # Convert lidar readings to xy points
        for i in range(len(scans)):
            # Get dist of each scan point (radius)
            r = scans[i]
            if np.isfinite(r): # Make sure scan exists
                #Get angle relative to zeron angle (which is along x axis)
                angle = first_angle + i * angle_increment
                # Set xy value in array
                xy.append = (r*np.cos(angle),  r*np.sin(angle))

        xy = np.array(xy)
        
      
        db = DBSCAN(eps=0.5, min_samples=5).fit(xy)
        labels = db.labels_

        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue  # Noise

            cluster_points = xy[labels == label]
            centroid = np.mean(cluster_points, axis=0)

            # Publish as Point
            point_msg = Point()
            point_msg.x = float(centroid[0])
            point_msg.y = float(centroid[1])
            point_msg.z = 0.0

            self.pub.publish(point_msg)

            # Publish as Marker
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "people"
            marker.id = int(label)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 1
            self.marker_pub.publish(marker)

            self.get_logger().info(f'Published person at ({centroid[0]:.2f}, {centroid[1]:.2f})')


#Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = PeopleNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()


