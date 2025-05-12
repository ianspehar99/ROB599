#!/usr/bin/env python3


# Action client for launch countdown

#Ian Spehar


# Rclpy imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


# Get action definition from .action file
from services_messages.action import Countdown

# Create client node
class CountdownClient(Node):
	def __init__(self):
		# Initialize the superclass
		super().__init__('action_client')

		# Set up client w node, type, action name
		self.client = ActionClient(self, Countdown, 'countdown')

	def send_goal(self, n):
		
		# Build the action goal to send to server
		goal = Countdown.Goal()
		goal.startnumber = n

		# Wait until server ready to send goal request
		self.client.wait_for_server()

		# Action request - send goal, set callback that handles the feedback. store handle in instance variable
		self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)

		# Callback reaction for when the request is accepted/declined from the actionserver
		self.result.add_done_callback(self.response)

	# Process feedback as we get it
	def feedback_cb(self, feedback_msg):
		# Log feedback
		self.get_logger().info(f'Got feedback: {feedback_msg.feedback.countnumber}')

	# Callback for when server accepts or declines the goal req
	def response(self, future):
		# Get response for requesting the action (not final result!!)
		goal = future.result()

		# If not accepted, log and exit
		if not goal.accepted:
			self.get_logger().info('Goal rejected')
			return

		# If accepted assign handle, then associate callback to access results when available later
		self.result_handle = goal.get_result_async()
		self.result_handle.add_done_callback(self.process_result)

	# Cllback for when action finished, final result
	def process_result(self, future):
		# Result - type correponds to result section of our action definition (Countdown.action in services_msgs package)
		result = future.result().result

		# Log to info channel
		self.get_logger().info(result.launched)


# This is the main entry point.
def main(args=None):
	# Init rclpy
	rclpy.init(args=args)

	# Set up node
	client = CountdownClient()

	# Action call
	client.send_goal(20)

	# spin
	rclpy.spin(client)

	# shut down correctly
	rclpy.shutdown()


# Main entry point
if __name__ == '__main__':
	main()
