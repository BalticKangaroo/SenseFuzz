from sensor_msgs.msg import Image, PointCloud2
from .ParameterClasses import SensorFuzzingParameter
import numpy as np
import copy
from collections import namedtuple
import math

from . import externalros2package_p_c2 as pc2


class CameraFrame():
	def __init__(self, image: Image, camera_fuzzing_parameter: SensorFuzzingParameter):
		self.originalFrame: Image = image
		self.camera_fuzzing_parameter = camera_fuzzing_parameter
		self.camera_fuzzing_parameter.width = self.originalFrame.width
		self.camera_fuzzing_parameter.height = self.originalFrame.height
		self._fuzzedFrame: Image = None
	
	@property
	def fuzzedFrame(self):
		# if the amount of fuzzed data is 0.0, skip the fuzzing and return original frame as there should not be any manipulation
		if self.camera_fuzzing_parameter.amount == 0.0:
			self._fuzzedFrame = self.originalFrame

		# created fuzzed frame, if not done so far
		if not self._fuzzedFrame:

			self.__apply_fuzzingMask()

		return self._fuzzedFrame

	def __apply_fuzzingMask(self):
		'''
		Apply the fuzzing mask to the image frame\n
		Fuzzer needs to be set before\n
		Returns numpy array
		'''
		self._fuzzedFrame = copy.deepcopy(self.originalFrame)
		
		image_data = np.array([self._fuzzedFrame.data])[0]

		intensity = self.camera_fuzzing_parameter.intensity * 255
		if self.originalFrame.encoding == "bgr8": len_pixel_array = 3
		if self.originalFrame.encoding == "bgra8": len_pixel_array = 4
		pixel_array = np.empty(len_pixel_array, dtype=np.int)
		pixel_array.fill(intensity)

		num_pix = self.camera_fuzzing_parameter.width * self.camera_fuzzing_parameter.height

		shaped_image_data = np.reshape(image_data, (num_pix, len_pixel_array))

		fuzzingMaskIndices = self.camera_fuzzing_parameter.maskIndices
		
		for array_index in fuzzingMaskIndices:
			shaped_image_data[array_index] = pixel_array

		image_data = shaped_image_data.flatten('C')

		self._fuzzedFrame.data = image_data.tobytes()


	### This method is not in use anymore, as it is too slow and takes too many ressources. It is left for possible inspiration.
	def __calc_intensity(self, value: int, intensity: float) -> int:
		'''
		Arguments:
		value: int - the value the calculation should be applied on
		intensity: float - the value of the intensity (defined in DataClasses-SensorFuzzingParameter_Image-Intensity)
		Calculates the new value to the color of the pixel based in the given intensity
		'''
		# value: 100	intensity: 1.0		--> 255
		# value: 100	intensity: -1.0		--> 0
		# value: 100	intensity: 0		--> 100
		# value: 100	intensity: -0.5		--> 50
		# value: 100	intensity: -0.33	--> 33
		# value: 200	intensity: -0.5 	--> 100
		# value: 200	intensity: -0.75 	--> 50
		# value: x		intensity: y 		--> if y > 0: x + (255-x)*y; if y < 0: x - x * (-y)
		if not intensity: raise ValueError("intensity is missing. Please provide.")
		if intensity == 0: return value
		if intensity < 0: return value - value * (-intensity)
		return value + (255 - value) * intensity

	### This method is not in use anymore, as it is too slow and takes too many ressources. It is left for possible inspiration.
	def __get_array_index(self, x: int, y: int, width: int, height: int, encoding: str) -> int:
		'''
		Args:
		x: int - x position of coordinate
		y: int - y position of coordinate
		width: int - width of the image
		height: int - height of the image
		len_pixel_array: int - number of values per pixel (default: 3)
		Returns:
		position of the first of the three values for the pixel (x,y): int
		-- Top left is (0,0)
		--- 
		ROS2 is using a flat array to send image data.
		Each pixel has 3 values thus an array has 3 x height x width of the image
		This function returns the index of the first of the three values
		top left is (0,0) and right bottom corner (width-1,height-1)
		'''
		if encoding == "bgr8": len_pixel_array = 3
		if encoding == "bgra8": len_pixel_array = 4
		# (0,0) --> 0
		# (1,0) --> 3
		# (599,0) --> 3 * 599 = 1797
		# (0,1) --> 1800
		# (x,y) --> 3 * width * y + 3 * x
		if not width: raise ValueError("width is missing. Please provide.")
		if not height: raise ValueError("height is missing. Please provide.")
		if x >= width or y >= height:
			raise ValueError(f"X (={x}) is greater than or equal width (={width}) or y (={y}) greater then or equal height (={height})")
		if x < 0 or y < 0:
			raise ValueError(f"X (={x}) or y (={y}) are smaller than 0")
		return len_pixel_array * (width * y + x) 
		




class LidarFrame():
	def __init__(self, lidar: PointCloud2, lidar_fuzzing_parameter: SensorFuzzingParameter):
		self.originalFrame: PointCloud2 = lidar
		self.lidar_fuzzing_parameter = lidar_fuzzing_parameter
		self._fuzzedFrame: PointCloud2 = None
		# self.__pc2toarray(lidar) --> writes the pointcloud data to a csv file

	@property
	def fuzzedFrame(self):
		# if the amount of fuzzed data is 0.0, skip the fuzzing and return original frame as there should not be any manipulation
		if self.lidar_fuzzing_parameter.amount == 0.0:
			self._fuzzedFrame = self.originalFrame
		# created fuzzed frame, if not done so far
		if not self._fuzzedFrame:
			self.__apply_fuzzingMask()		
		return self._fuzzedFrame

	def __apply_fuzzingMask(self):
		'''
		Apply the fuzzing mask to the image frame\n
		Fuzzer needs to be set before\n
		It is currently optimized for the velodyne VLP 16 sensor on the AWSIM Quickstart vehicle.
		'''
		self._fuzzedFrame = copy.deepcopy(self.originalFrame)
		
		lidar_data = pc2.read_points_list(self.fuzzedFrame, skip_nans=True)

		# insert a noise cloud
		## inspiration: ### https://www.reddit.com/r/ROS/comments/kz3gu1/converting_numpy_pointcloud_to_ros_pointcloud2/?rdt=60121
		steps = self.lidar_fuzzing_parameter.steps
		width = self.lidar_fuzzing_parameter.width

		field_names = [f.name for f in self.fuzzedFrame.fields]
		Point = namedtuple('Point', field_names)
		if len(field_names) == 9:
			extended = True
		elif len(field_names) == 5:
			extended = False
		else:
			ValueError("Pointcloud format unknown -- cannot fuzz")

		for point in self.lidar_fuzzing_parameter.mask:
			y = np.float32((point[0] - width / 2) / steps)

			### old code when it was used with carla. In case it is again used with carla it might help. Therefore, it is still here.
			# the substraction of 18 is a value gained from experimental setup in carla-ros-bridge.
			# It brings the bottom line of the frame to the ground
			# z = np.float32((point[1] - 18) / steps)
			# new_point = [x, y, z, intensity, 0]
			# new_lidar_data.append(new_point)

			z = np.float32((point[1]) / steps)
			vlp_16_x = self.lidar_fuzzing_parameter.distance
			vlp_16_y = y
			vlp_16_z = z
			vlp_16_intensity = self.lidar_fuzzing_parameter.intensity
			new_point = self.__apply_lidar_point_velodyne_VLP_16(vlp_16_x, vlp_16_y, vlp_16_z, vlp_16_intensity, Point, extended)
			if new_point: lidar_data.append(new_point)
			
			

		self._fuzzedFrame = pc2.create_cloud(header=self.fuzzedFrame.header, fields=self.fuzzedFrame.fields, points=lidar_data)

	def __apply_lidar_point_velodyne_VLP_16(self, x: float, y: float, z: float, intensity: float, Point: namedtuple, extended: bool):
		'''
		Adapter between the fuzzing mask and the velodyne VLP 16 Lidar sensor.
		Returns new lidar point.
		'''
		lidar_sensor_param = namedtuple('lidar_sensor_param', "x y z max_angle delta_angle max_rings")

		## parameters of the the velodyne VLP 16 sensor on the AWSIM Quickstart vehicle.
		sensor_param = lidar_sensor_param(x=0.0,y=0.0,z=2.0, max_angle=15.0, delta_angle=2.0, max_rings=20)

		# Math magic. This is based on reverse engineering of the sensor and the attachement on the the AWSIM Quickstart vehicle.
		# For questions contact the original maintainer
		z = z-sensor_param.z
		z = z
		if z == 0:
			z += 0.000001
		distance = math.sqrt(math.pow((x-sensor_param.x), 2)+math.pow((y-sensor_param.y), 2)+math.pow((z-sensor_param.z), 2))
		angle_scan = math.degrees(math.asin(abs(z)/distance))
		number_delta_angles = angle_scan / sensor_param.delta_angle
		absolute_ring = int(math.floor(number_delta_angles))
		absolute_new_z = math.sin(math.radians(sensor_param.delta_angle * (0.5 + absolute_ring))) * distance
		if z > 0:
			ring = absolute_ring + (sensor_param.max_rings/2) + 1
			new_z = absolute_new_z
		if z < 0:
			if angle_scan > sensor_param.max_angle:
				# print(f"Lidar would not be able to scan this point. Max scan angle: -{sensor_param.max_angle} - but -{angle_scan}. It is kipped")
				return None
			ring = (sensor_param.max_rings/2) - absolute_ring
			new_z = - absolute_new_z					

		if ring < 1 or ring > sensor_param.max_rings:
			# print(f"point out of range of rings - actual ring: {ring}")
			return None
		if new_z < - sensor_param.z:
			# print(f"point below surface: {new_z} --> point is skipped")
			return None


		if not extended:
			newPoint = Point(x=x, y=y, z=new_z,
			 				intensity=intensity, ring=ring)
			return newPoint
				
		newPoint = Point(x=x, y=y, z=new_z,
			 				intensity=intensity, ring=ring, azimuth=0.0,
							distance=distance, return_type=0, time_stamp=0.0)
		return newPoint


	### legacy files might be helpful for later developement or debugging
	def __calc_intensity(self, intensity: float, min_intensity: float, max_intensity: float) -> float:
		'''
		Arguments:
		value: int - the value the calculation should be applied on
		intensity: float - the value of the intensity (defined in DataClasses-SensorFuzzingParameter_Lidar-Intensity)
		Calculates the new value to the color of the pixel based in the given intensity
		'''
		return min_intensity + ( ( (max_intensity - min_intensity) / 2)  * (1 + intensity) )

	def __pc2toarray(self, ros2_point_cloud):
		## inspiration: https://answers.ros.org/question/344096/subscribe-pointcloud-and-convert-it-to-numpy-in-python/
		xyz_intensity = np.empty([1,4])
		gen = pc2.read_points(ros2_point_cloud, skip_nans=True)
		int_data = list(gen)
		for x in int_data:
			xyz_intensity = np.append(xyz_intensity,[[x[0],x[1],x[2],x[3]]], axis = 0)
		np.savetxt("lidar_package_AWSIM.csv", xyz_intensity, delimiter=',')
		return xyz_intensity
	
	def __write2file(self, filename, header, fields,  data: list):
		with open(filename, 'w') as fp:
			# fp.write("Header:\n")
			# fp.write(str(header))
			# # for item in header:
			# # 	fp.write(str(item))
			# # 	fp.write("\n")
			# fp.write("\nFields:\n")
			# for item in fields:
			# 	fp.write(str(item))
			# 	fp.write("\n")
			fp.write("\nData:\n")
			for item in data:
				fp.write(str(item))
				fp.write("\n")
	
	def __write2fileSimple(self, filename, data: dict):
		with open(filename, 'w') as fp:
			for ring in data:
				fp.write(f"{str(ring)} - {str(data[ring])}")
				fp.write("\n")
		
