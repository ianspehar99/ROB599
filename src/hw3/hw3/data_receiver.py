#!/usr/bin/env python3

# Ian Spehar

# This node subscribes to the "data" topic from data_sender.py
# Publishes to "latency" (avg latency over last 10)
# Pubs to "raw_latency", latency of every data entry from sender
# ALso contain service for logging control/which file to log to
import rclpy
from rclpy.node import Node

# Import srv for logging
from services_messages.srv import EnableLogging

# Import testpacket msg type
from services_messages.msg import TestPacket

# And floats
from std_msgs.msg import Float32



class ReceiveNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('data_receiver')

        # SUbscribe to data topic from sender
        self.sub = self.create_subscription(TestPacket, 'data',self.sub_callback,10)

        # Publish average latency (msg type,topic name,queue size)
        self.avg_latency = self.create_publisher(Float32, 'latency',10)

        # Publish raw latency
        self.raw_latency = self.create_publisher(Float32,'raw_latency',10)

        # Creates service  (type,name,callback)
        self.service = self.create_service(EnableLogging, 'logdata', self.service_callback)

        #Ceate timer, uses period, so inverse of frequency
        freq =   10 #Publish at 10 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.publisher_callback)

        # Logging switch, False initially, change with service call
        self.switch = False
        
        # File to log to, set with service call
        self.file = None

        self.get_logger().info('Logging started, waiting for service call')

    def sub_callback(self):

        
    def publisher_callback(self):
        pub_data = self.switch
        
        if pub_data:

            block = [0,0,0]   # Block data, doesnt matter

            #Get current time
            t = self.get_clock().now()
            t = t.nanoseconds/(1e9) #Convert to seconds

            #Init msg type
            msg = TestPacket()

            #Set msg 
            msg.send_time = t
            msg.payload = block

            #Publish msg
            self.pub.publish(msg)

            #Print published wave data
            self.get_logger().info('Data published, time: "%s"' % msg.send_time)

    def service_callback(self, request, response):
        
        # Access input from service call/client
        bool = request.send 

        # Set variable so that the publisher callback can access it
        self.switch = bool

        # Confirmed that it has been recieved
        self.get_logger().info('Service call received, publishing = "%s"' % bool)
        
        # Set response so that we can see it was received and whats happening
        if bool:
            response.update = "Service call received, node has been started"
        else:
            response.update = "Service call received, node has been stopped"

        return response

# Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the publisher
    publisher = SendNode()

    #Spin that thang
    rclpy.spin(publisher)

    #When done, shutdown
    rclpy.shutdown()

