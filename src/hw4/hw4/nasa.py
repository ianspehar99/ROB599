#!/usr/bin/env python3

# Action server for launch countdown

#Ian Spehar


# Rclpy imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# Get action definition from services_messages
from services_messages.action import Countdown

from services_messages.srv import SendData

# Import sleep
from time import sleep

# Create action server
class CountdownActionServer(Node):
	def __init__(self):
		# Superclass
		super().__init__('countdown_server')

		# Set up action pass node, type, action name, and a callback
		self.server = ActionServer(self, Countdown, 'countdown', self.callback)
		
		# Create service  (type,ros2 callname,callback)
		self.service = self.create_service(SendData, 'abortlaunch', self.service_callback)
									 
        # Abort bool from service
		self.abort = False
	
    # Action callback - Main coutndown code
	def callback(self, goal):
		# Send msg to logger
		count_start = goal.request.startnumber
		
		self.get_logger().info(f'Got {count_start}')

		# Build result, send the 'launched' result msg to it
		result = Countdown.Result()
		result.launched = 'Rocket launched successfully SIUUUUUU'
		
		for i in range(count_start+1):

			abort = self.abort

			self.get_logger().info(f'abort status = {abort}')

			if not abort:
				num = count_start - i # Publish feedback
				
				goal.publish_feedback(Countdown.Feedback(countnumber = num))
				
				self.get_logger().info(f'Countdown at {num}')
				
				# Countdown every second (SPin once allows for service callback to be processed in between)
				rclpy.spin_once(self, timeout_sec=1.0)
			
			else:
				# Return message
				self.get_logger().info('Launch aborted')
				
                # Set result
				result.launched = ('Launch was aborted')
				
				# Stop the action server (abort not related to the variable i have, its part of ros2)
				goal.abort()
				 
                # Exit for loop
				
				return result
				
				
				
				
        # Let server know that it is done
		goal.succeed()
		self.get_logger().info(f'Result: {result.launched}')
		
		return result
	
	
	def service_callback(self, request, response):
        # Pick up whether we should abort or not
		send_bool = request.send 
		
		self.abort = send_bool
		
		if send_bool:  # Abort  =  yes
			response.update = "Service call received, launch cancel in progress..."
		else:
			response.update = "Service call received, launch starting again..."
		return response
	

# Main entry
def main(args=None):
	# Init rclpy
	rclpy.init(args=args)

	# Init node
	server = CountdownActionServer()

	# Spin
	rclpy.spin(server)

	rclpy.shutdown()


# Run main
if __name__ == '__main__':
	main()
