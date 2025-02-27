#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

from .SensorFrame import CameraFrame
from .ParameterClasses import Sensor, SensorFuzzingParameter

import uuid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# use parameters from global_parameters.yml file
import yaml

class FuzzingBroker_Camera(Node):
	'''
	The Fuzzing Broker Camera is the part of the Fuzzing Broker responsible for subscribing to the ROS2 camera sensor data from the simulator.
	Afterwars it is forwarding it to the Fuzzing Environment.
	Once it is back from the manipulate sensor data, it republishing for the ADS.
	It initilizes the fuzzer and all required services for manipulating the sensor data.
	A global parameters file is used for most of the parameters.
	'''
	def __init__(self, parameterfile: str, nodename: str):
		
		# read the values from the global parameters file
		# the nodename is the key to the parameters for the specific node
		self.node_params = self.read_parameters(file=parameterfile)[nodename]

		# create a Node with the node_name
		super().__init__(self.node_params["node_name"])
		self.node_name = self.node_params["node_name"]
		self.Frame: CameraFrame

		# make the path to the parameters available to the class.
		# Here the parameters are not stored into variables to allow real time adjustments by updating the params file
		self.parameterfile = parameterfile

		### publisher

		# create a ROS2 publisher with given parameters
		self.publisherFuzzed_camera = self.create_publisher(Image, self.node_params["topic_publish"], 10)

		### subscriber

		# set Quality of Service profile. This is required for autoware.universe (and probably RVIZ) to be able to subscribe
		qos_profile = QoSProfile(
		reliability=QoSReliabilityPolicy.BEST_EFFORT,
		history=QoSHistoryPolicy.KEEP_LAST,
		depth=1
		)

		# read the topic to subsribe to
		topic_subscribe = self.node_params["topic_subscribe"]
		# receive data from the topic and forward it to the broker
		self.subscriber_camera = self.create_subscription(
			Image, topic_subscribe, self.broker_camera, qos_profile=qos_profile)
		# print info text to the console
		self.get_logger().info(f"{self.node_name} has been started and is subscribed to {topic_subscribe}")

		##### Some subscriber also need to read the camera info. In such a case a second subsricber and publisher need to be created just for the camera info.
		##### RVIZ mostly needs it. RQT does not.
		##### You possibly need to adjust the code
		# topic_cameraInfo_publish = "TOPIC NAME"
		# self.publisher_cameraInfo_ = self.create_publisher(CameraInfo, topic_cameraInfo_publish, 10)
		# topic_sub_cameraInfo = "TOPIC NAME"
		# receive data from the topic and forward it to the broker
		# self.subscriber_cameraInfo_ = self.create_subscription(CameraInfo, topic_sub_cameraInfo, self.broker_cameraInfo)
		# self.get_logger().info(f"{self.node_name} is also subscribed to {topic_sub_cameraInfo} and republishes the camera info at {topic_cameraInfo_publish}")

	def broker_camera(self, msg):
		'''
		The broker initilizes the fuzzer for creating the fuzzing mask based on the values from the parameters file.
		The fuzzing mask is applied afterwards.
		This method is this whole procedure by calling services (classes and methods)
		and calls the method 'publish_camera' with the manipulated frame.
		
		Params:
		msg: Image  -- The ROS2 Message subscribed

		Returns: -
	
		'''
		# read the parameters from the file. This is done for every frame to allow real time adjustments of parameters in the file taking affect.
		fuzzing_parameter_file = self.read_parameters(file=self.parameterfile)[self.node_params["fuzzing_parameter"]]

		# create a sensor object. This is required to create a SensorFuzzingParameter object.
		# The function is limited, but was created having in mind that one sensor can have multiple fuzzing iterations and thus allows a object based linking.
		sensor = Sensor(sensor_name=fuzzing_parameter_file["sensor"]["sensor_name"],
				   			ros2_topic=fuzzing_parameter_file["sensor"]["ros2_topic"],
							sensor_type=fuzzing_parameter_file["sensor"]["sensor_type"])
		
		# creating a SensorFuzzingParameters object
		# This object is the Fuzzer. At first all parameters are set...
		fuzzing_parameter = SensorFuzzingParameter(sensor=sensor,
												 identifier=uuid.uuid4(),
												 random_seed = fuzzing_parameter_file["random_seed"],
												 amount = fuzzing_parameter_file["amount"],
												 dispersion = fuzzing_parameter_file["dispersion"],
												 location_x = fuzzing_parameter_file["location_x"],
												 location_y = fuzzing_parameter_file["location_y"],
												 distance = fuzzing_parameter_file["distance"],
												 intensity = fuzzing_parameter_file["intensity"],
												 width = fuzzing_parameter_file["width"],
												 height = fuzzing_parameter_file["height"],
												 steps = fuzzing_parameter_file["steps"]
												)

		# ...and afterwards it is handed to the CameraFrame object. This object brings/applies the fuzzing mask and the sensor data together.
		# Type conversions should happen in there.
		self.Frame = CameraFrame(image=msg, camera_fuzzing_parameter=fuzzing_parameter)

		# # remove comments for just republishing the original frame. Alternatively, set the amount to 0.0 in the params file.
		# self.publish_camera(msg=self.Frame.originalFrame)

		# publish the frame using the method
		self.publish_camera(msg=self.Frame.fuzzedFrame)


	def publish_camera(self, msg: Image):
		'''
		Extra class method to publish the camera frame
		'''
		# this is a extra method, as it has a dedicated purpose.
		# It allows possible adjustments if required. Without touching the actual broker method.
		self.publisherFuzzed_camera.publish(msg)

	# Read the parametersfile
	def read_parameters(self, file):
		return yaml.safe_load(open(file))
	

	##### Some subscriber also need to read the camera info. In such a case a second subsricber and publisher need to be created just for the camera info.
	##### RVIZ mostly needs it. RQT does not.
	##### Subscriber and Publisher are above.
	def broker_cameraInfo(self, msg):
		self.publish_cameraInfo(msg)

	def publish_cameraInfo(self, msg: CameraInfo):
		self.publisher_cameraInfo.publish(msg)


def main(args=None):
	rclpy.init(args=args)

	# provide path to parametersfile
	path_parameterfile = "fuzzingparams.yml"
	# provide key to the parameters for the node in the parameters file
	nodename = "camera_fuzzing_node"

	# initilize  and run ROS2 node
	node = FuzzingBroker_Camera(parameterfile=path_parameterfile, nodename=nodename)

	rclpy.spin(node)
	rclpy.shutdown()

  
if __name__ == "__main__":
	main()

