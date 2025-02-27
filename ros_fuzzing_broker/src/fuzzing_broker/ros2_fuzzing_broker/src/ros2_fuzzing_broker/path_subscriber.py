#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from .autoware_auto_vehicle_msgs.msg import VelocityReport

from .SensorFrame import LidarFrame
from .ParameterClasses import Sensor, SensorFuzzingParameter

import uuid

### !!!!!!!!!! This class is not working and not finished !!!!!!!!!!
class PathSubscriber(Node):
	'''
	This is a not finished and not working class.
	The intention was to read the published data from autoware.universe to have information for the oracle.
	'''
	def __init__(self, node_name: str):
		super().__init__(node_name)
		self.node_name = node_name

		### subscriber
		# create the topic to subsribe to
		# receive data from the topic and forward it to the broker
		velocity_status = "/vehicle/status/velocity_status"
		self.subscriber_pose = self.create_subscription(
			VelocityReport, velocity_status, self.print_message)
		self.get_logger().info(f"{self.node_name} has been started and is subscribed to {velocity_status}")

	def print_message(self, msg):
		print(str(msg))

def main(args=None):
	rclpy.init(args=args)

	node = PathSubscriber(node_name='path_subscriber')
	rclpy.spin(node)
	rclpy.shutdown()

  
if __name__ == "__main__":
	main()

