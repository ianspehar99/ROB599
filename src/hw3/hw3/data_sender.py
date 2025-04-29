#!/usr/bin/env python3
#  This node pubs to the "data" topic 
# Has service that enables/disables sending
import rclpy
from rclpy.node import Node

# Import srv for turning data sending on/off
from services_messages.srv import SendData

# Import Testpacket msg type
from services_messages.msg import TestPacket


class SendNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('data_sender')

        #create publisher with (msg type,topic name,queue size)
        self.pub = self.create_publisher(TestPacket, 'data',10)

        #Ceate timer, uses period, so inverse of frequency
        freq =   10 #Publish at 10 Hz
        period = 1/freq
        self.timer = self.create_timer(period,self.publisher_callback)

        # Creates ervice  (type,ros2 callname,callback)
        self.service = self.create_service(SendData, 'senddata', self.service_callback)

        # Start publishing switch for False initially, will change with service call
        self.switch = False

        self.get_logger().info('Sender started, waiting for service call')

        
    def publisher_callback(self):
        pub_data = self.switch
        
        if pub_data:

            block = [0,0,0]   # Block data, doesnt matter

            #Get current time
            t = self.get_clock().now().to_msg()

            #Init msg type
            msg = TestPacket()

            #Set msg 
            msg.send_time = t
            msg.payload = block

            #Publish msg
            self.pub.publish(msg)

            #Print published wave data
            self.get_logger().info('Data published, time: "%s"' % t)

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