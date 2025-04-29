#!/usr/bin/env python3

# Ian Spehar

# This node subscribes to the "data" topic from data_sender.py
# Publishes to "latency" (avg latency over last 10)
# Pubs to "raw_latency", latency of every data entry from sender
# ALso contain service for logging control/which file to log to
import rclpy
import numpy as np
import csv
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
        self.sub = self.create_subscription(TestPacket, 'data',self.pubsub_callback,10)

        # Publish average latency (msg type,topic name,queue size)
        self.avg_latency = self.create_publisher(Float32, 'latency',10)

        # Publish raw latency
        self.raw_latency = self.create_publisher(Float32,'raw_latency',10)

        # Creates service  (type,ros2 call name,callback)
        self.service = self.create_service(EnableLogging, 'logdata', self.service_callback)

        #VARIABLES:
        # Logging switch, False initially, change with service call
        self.switch = False
        
        # File to log to, set with service call
        self.file = None

        # Create array of time delays
        self.time_difs = []

        # Csv variables
        self.csv_obj = None
        self.csv_writer = None


        self.get_logger().info('Logging started, waiting for service call')

    #pub sub callback, retrieve data from sub to the sender
    def pubsub_callback(self,msg):
        # Will check later if this is defined (every ten callbacks) for logging purposes
        avg = None
        send_time = msg.send_time

        # Convert for subtracting
        send_time = rclpy.time.Time.from_msg(send_time)

        # Calc current time
        time_now = self.get_clock().now()

        # Get latency
        latency = time_now - send_time
        
        # Init msg type and set val
        raw_latency = Float32()

        # Convert from time dif back to float
        raw_latency.data = (latency.nanoseconds)/ 1e9

        # Publish to raw latency topic:
        self.raw_latency.publish(raw_latency)

        # Add to list for avging later
        self.time_difs.append(latency.nanoseconds / 1e9)  

        # Latency averaging and publishing:
        if len(self.time_difs) > 10:     #Every time we reach 10 values
            avg = np.mean(self.time_difs)  #Get average

            # Init msg and set val
            avg_latency = Float32()

            avg_latency.data = avg

            self.avg_latency.publish(avg_latency)   #Publish avg

            # Clear array
            self.time_difs = []


        if avg is not None:
            self.get_logger().info(f'Raw latency: {raw_latency.data:.6f}s, Avg Latency: {avg:.6f}s')
        else:
            self.get_logger().info(f'Raw latency: {raw_latency.data:.6f}s')


        # If logging is currently enabled by the service callback, log data to inputted csv file
        if self.switch and self.csv_writer:

            self.get_logger().info("Logging to file ...")

            time_stamp = time_now

            row = [time_stamp.nanoseconds/1e9, raw_latency.data]

            self.get_logger().info(f"Row: {row}")

            if avg is not None:
                row.append(avg)
            
            # Write to csv file
            self.csv_writer.writerow(row)



    def service_callback(self, request, response):
        
        # Access bool input for logging on/off
        logonoff = request.logging

        # Access inputted file name to write to
        file = request.filename

        # Set variable so that the publisher callback can access it
        self.switch = logonoff
        self.file = file

      
        # Handle cases when logging is turned on and then off
        if logonoff: #LOgging turned on
            # Open selected file and create
            self.csv_obj = open(self.file, 'w', newline='') 
            self.csv_writer = csv.writer(self.csv_obj)

            # Set response
            response.update = "Service call received, logging started"
        else:  #Logging turned off
            # Close file
            if self.csv_obj:
                self.csv_obj.close()  

            self.csv_writer = None
            self.csv_obj = None
            
            # Set response
            response.update = "Service call received, logging stopped"
    
        return response

# Main function
def main(args=None):

    #Initialize rclpy
    rclpy.init(args=args)

    # Init the node
    node = ReceiveNode()

    #Spin
    rclpy.spin(node)

    #When done, shutdown
    rclpy.shutdown()

