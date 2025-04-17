#!/usr/bin/env python3

# This node subscribes to the oscope topic, crops it between -0.5 and 0.5, and republishes

# Ian Spehar

# Import node dependencies
import rclpy
from rclpy.node import Node

# Import msg type
from std_msgs.msg import Float32

# Limiter node
class LimiterNode(Node):
    #Initiate, going to be function of frequency so we can vary it depending on which executable we run
    def __init__(self):
        # Init the parent class and give it name
        super().__init__('limiter_node')

        #create subscription with (msg type, topic name, callback func, queue size)
        self.sub = self.create_subscription(Float32, 'oscope',self.callback,10)

        #create publisher
        self.pub = self.create_publisher(Float32,'cropped_wave',10)

# Callback is called each time message recieved
    def callback(self,msg):
        # Handle input message:
        value = msg.data

        #Crop input data 
        if value < -0.5:
            value = -0.5
        elif value > 0.5:
            value = 0.5
        
        #Now setup publisher

        #Init msg type
        msg_out = Float32()

        #Set msg
        msg_out.data = value

        #Publish msg
        self.pub.publish(msg_out)

        #Print updated data
        self.get_logger().info('GPS Data: "%s"' % msg_out.data)

#Main funct
def main(args=None):
    frequency = 1  # 1 Hz for basic 
    #Initialize rclpy
    rclpy.init(args=args)

    # Init subscriber (which also publishes)
  
    limiter = LimiterNode()

    #Spin that thang
    rclpy.spin(limiter)

    #When done, shutdown
    rclpy.shutdown()