from dataclasses import dataclass
import uuid
import numpy as np

@dataclass
class Sensor:
	sensor_name: str
	ros2_topic: str
	# allowed values for sensor_type: camera; lidar
	sensor_type: str

class SensorFuzzingParameter():
	'''
	Arguments:
		sensor:         Dataclass of type sensor
		identifier:     uuid4 identifier
		random_seed:    integer ge 0                - seed for the random number generator
		amount:         float [0.0-1.0]             - The amount of generated coordinates --> as there is a chance of duplicates in coordinates, it is not the final number
		dispersion:     float [0.0-1.0]             - The standart deviation around the location of center in percentage of width and height of the image
		location_x/location_y:     float [0.0-1.0]  - location of the center of mean in two dimensional area
		distance:       float [0.0-50.0]            - Distance of the lidar noice from the vehicle (x coordinate).
		intensity:      float [-1.0-1.0]            - the intensity how much the affected pixles are changed for images (-1.0 -> 0; 1.0 ->255)\n
													- results in LiDAR Intensity value between 0.0 and 100.0
		width:          int                         - width of frame the noise is generated in
		height:         int                         - height of frame the noise is generated in
		steps:          float min 1.0 - default 1.0 - steps between lidar points. For pixles it cannot differ to 1 
	'''
	def __init__(self, sensor: Sensor=None, identifier: uuid=None, random_seed: int=None, amount: float=0.1, dispersion: float=0.1, location_x: float=0.5, location_y: float=0.5, distance: float=0.5, intensity: float=1.0, width: int=None, height: int=None, steps: float=1.0):
		self.sensor = sensor
		self.identifier = identifier

		# seed for the random number generator
		self.random_seed = random_seed

		# The amount of manipulated datapoints [0.0-1.0]
		# it is not the final amount
		# as there is a higher probability for duplicated cooridnates
		if type(amount) is not float: raise TypeError(f"amount not type float but type {type(amount)}") 
		if amount < 0 or amount > 1.0: raise ValueError(f"amount out of range. Value: {amount} - allowed [0;1.0]")
		self.amount = amount

		# The standart deviation around the location of center
		# in percentage of width and height of the image
		self.dispersion = dispersion
		if type(dispersion) is not float: raise TypeError(f"dispersion not type float but type {type(dispersion)}") 
		if dispersion < 0 or dispersion > 1.0: raise ValueError(f"dispersion out of range. Value: {dispersion} - allowed [0;1.0]")
		
		# location of the center
		if type(location_x) is not float: raise TypeError(f"location_x not type int but type {type(location_x)}")
		if type(location_y) is not float: raise TypeError(f"location_y not type int but type {type(location_y)}")
		if location_x < 0.0 or location_x > 1.0: raise ValueError(f"location_x out of range. Value: {location_x} - allowed [0.0;1.0]. Look in class description for more information")
		if location_y < 0.0 or location_y > 1.0: raise ValueError(f"location_y out of range. Value: {location_y} - allowed [0.0;1.0]. Look in class description for more information") 
		self.location_x = location_x
		self.location_y = location_y

		# for LiDAR the distance of the generated noise to the vehicle in the x dimension (Straight look from the vehicle)
		self.distance = distance

		# the intensity how much the affected pixles are changed [-1.0 - 1.0]
		# default white (1) -> this means each of the affected pixels will have the value 255 for all 3 colors
		if type(intensity) is not float: raise TypeError(f"intensity not type float but type {type(intensity)}") 
		if intensity < -1.0 or intensity > 1.0: raise ValueError(f"intensity out of range. Value: {intensity} - allowed [-1.0;1.0]")
		self.intensity = intensity

		# array with tuples (x,y) for each pixle/coordinate to fuzz.
		# There are no different values for the three RGB parameters
		# --> len(array(mask)) = len(array(image))/3 * amount (for images)
		# [[x,y],[1,1],[123,312],[456,321],...]
		self._mask = np.empty([1,2])
		self._maskIndices = np.empty([1,])
		# width of frame the noise is generated in
		if width and type(width) is not int: raise TypeError(f"width not type int but type {type(width)}") 
		self._width = width
		# height of frame the noise is generated in
		if height and type(height) is not int: raise TypeError(f"height not type int but type {type(height)}") 
		self._height = height
		# steps between lidar points. For pixles it cannot differ to 1
		if type(steps) is not float: raise TypeError(f"steps not type float but type {type(steps)}") 
		if steps < 1:  raise ValueError(f"steps below 1. Value: {steps} - ")
		self.steps = steps
	
	# allow custom getter methods for the width property of the object
	@property
	def width(self):
		return self._width
	
	# allow custom setter methods for the width property of the object
	@width.setter
	def width(self, width):
		self._width = width
	
	# allow custom getter methods for the height property of the object
	@property
	def height(self):
		return self._height
	
	# allow custom setter methods for the height property of the object
	@height.setter
	def height(self, height):
		self._height = height

	# allow custom getter methods for the mask property of the object
	@property
	def mask(self):
		# return mask immediatly, as it has already been created
		if len(self._mask) > 1: return self._mask
		# create mask as it has not been created yet
		self.__create_sensorFuzzingMaskArray()
		return self._mask
	
	# allow custom setter methods for the mask property of the object
	@mask.setter
	def mask(self, mask):
		self._mask = mask
	
	# allow custom getter methods for the mask property of the object
	@property
	def maskIndices(self):
		# return mask immediatly, as it has already been created
		if len(self._maskIndices) > 1: return self._maskIndices
		# create mask as it has not been created yet
		self.__create_sensorFuzzingMaskArray()
		self.__createMaskIndices()
		return self._maskIndices
	
	# allow custom setter methods for the mask property of the object
	@maskIndices.setter
	def mask(self, maskIndices):
		self._maskIndices = maskIndices

	def __createMaskIndices(self):
		'''
		precalculates the indices of the masks for faster application on the image array
		'''
		list_indices = []
		for pixel in self._mask:
				list_indices.append(self.__get_array_index(x=pixel[0], y=pixel[1]))
		self._maskIndices = np.array(list_indices)

	def __get_array_index(self, x: int, y: int) -> int:
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
		width=self.width
		height=self.height
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
		return (width * y + x)

	def __create_sensorFuzzingMaskArray(self):
		'''
		Arguments:
			number_data_points: int - number of lidar points\n
		Returns:
			SensorFuzzingMask
		---
		Creates the sensor fuzzing mask for the lidar sensor
		'''
		# calculate the location of the center of the mean relative to the width and height
		loc_x = int(self.width * self.location_x)
		loc_y = int(self.height * self.location_y)
		
		# calculate the dispersion based on the standart deviation relative to the width and height
		standard_deviation_x = int(self.width * self.dispersion)
		standard_deviation_y = int(self.height * self.dispersion)
		
		# calculate the amount of pixles to fuzz based on the given percentage
		amount = int(self.height * self.width * self.amount)

		# generate array of x values
		x_values = self.__generate_coordinate_parts(loc_x, standard_deviation_x, amount, upper_bound=self.width)
		# generate array of y values
		y_values = self.__generate_coordinate_parts(loc_y, standard_deviation_y, amount, upper_bound=self.height)

		# combine x and y values to coordinates. coordinates look like: (x_values[i];y_values[i])
		mask = np.column_stack((x_values, y_values))
		self.mask = mask
		self._mask = mask

	def __generate_coordinate_parts(self, location: int, standard_deviation: int, amount: int, upper_bound: int) -> np.array:
		'''
		---
		Returns an array of for one part of a coordinate in a rectangle frame
		based on a normal distribution with the provided parameters
		'''
		# generate array of values
		values = np.random.normal(loc=location, scale=standard_deviation, size=amount).astype(int)

		# generate array of values to replace outliers and shrink array to allowed values
		replace_values = np.random.normal(loc=location, scale=standard_deviation, size=amount).astype(int)
		replace_values = replace_values[replace_values < upper_bound]
		replace_values = replace_values[replace_values > 0]

		## Replace all values for coordinate in array with valid (0 =< x < upper_bound) integers
		# replace values greater than upper_bound with values between 0 and upper_bound
		for value_index in np.nonzero(values >= upper_bound)[0]:
			values[value_index] = np.random.choice(replace_values)
		# replace values smaller than 0 with values between 0 and image_width
		for value_index in np.nonzero(values < 0)[0]:
			values[value_index] = np.random.choice(replace_values)  
		    
		return values

	def fingerprint(self, other) -> list:
		'''
		Get a list with the relevant parameters to allow comparing two SensorFuzzingParameter classes
		'''
		fingerprint = [self.sensor.sensor_name,
						self.sensor.sensor_type,
						self.sensor.ros2_topic,
						self.identifier,
						self.random_seed,
						self.sensor,
						self.identifier,
						self.random_seed,
						self.amount,
						self.dispersion,
						self.location,
						self.distance,
						self.intensity,
						self.width,
						self.height,
						self.steps
					   ]
		return fingerprint
	
	def __eq__(self, other):
		return self.fingerprint == other.fingerprint
