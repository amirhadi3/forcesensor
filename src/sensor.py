#! /usr/bin/env python

import rospy
import numpy as np
from forcesensor.msg import SensorOutput
from forcesensor.srv import SensorOutputRequest
from forcesensor.msg import FlagMsg
from forcesensor.msg import ByteMsg
import CRC
import time

from serial_port_wrapper import Serial_Device

class Sensor:

	def __init__(self, baud=5000000, num_sensors=1):

		self.num_sensors = num_sensors

		##Variables to store most recent sensor response and whether it has been read yet
		self.response = [] #Array of two objects, one for sensor 0 and one for sensor 1
		self.response_seen = [] #Array of 2 bools, sensor 0 and sensor 1
		if num_sensors==1:
			self.response.append(0)
			self.response_seen.append(True)
		else:
			self.response.append(0)
			self.response.append(0)
			self.response_seen.append(True)
			self.response_seen.append(True)

		# #Create subscribers to read single responses from the sensor over the corresponding topics
		# self.byte_response_pub = rospy.Subscriber('byte_response', ByteMsg, self.__callback)
		# self.packet_response_pub = rospy.Subscriber('packet_response', SensorOutput, self.__callback)

		#Write commands from the user
		self.user_cmds = rospy.Publisher('user_commands', ByteMsg, queue_size=num_sensors)
		#Start or stop continuous data transfer
		self.run_flag = rospy.Publisher('continuous_data_flag', FlagMsg, queue_size=1)

	def __del__(self):
		"""
		When closing the force sensor object, tell all sensors to stop continuous data transfer
		and close all the ports
		"""
		stop_transmission = FlagMsg()
		stop_transmission.data_flag = False

		for i in range(self.num_sensors):
			stop_transmission.sensor_num = i
			self.run_flag.publish(stop_transmission)

	def start_data_transfer(self, data_rate=1500, sensor_num=0):
		"""
		Tell the sensor to start sending data continuously
		This data is published to the continuous_data topic and can be accessed there
		:param sensor_num the index of the sensor to send the command to.
		"""
		flag = FlagMsg()
		flag.data_flag = True
		flag.sensor_num = sensor_num
		flag.data_rate = data_rate
		self.run_flag.publish(flag)

	def stop_data_transfer(self, sensor_num=0):
		"""
		Tell the sensor to stop sending continuous data
		:param sensor_num the index of the sensor to send the command to.
		"""
		flag = FlagMsg()
		flag.data_flag = False
		flag.sensor_num = sensor_num
		flag.data_rate = 0
		self.run_flag.publish(flag)

	def measure(self, sensor_num=0):
		"""
		Request one measurement from the sensor
		:return a parsed data packet in the form of a SensorOutput object
			None if the packet somehow fails to arrive
		:param sensor_num the index of the sensor to send the command to.
			Can only do one at a time
		"""
		rospy.wait_for_service('ft_read')
		read = rospy.ServiceProxy('ft_read', SensorOutputRequest)
		try:
			data = read(sensor_num)
			rospy.loginfo(data)
		except rospy.ServiceException as exc:
			rospy.logwarn('Service did not process measurement request: ' + str(exc))
			return None

		if data.sensor_num == -2:
			rospy.logwarn('Measurement of one packet failed: returned empty packet')
	 		return None
	 	else:
	 		return SensorOutput(data.differential, data.sum, data.imu, data.report_id, np.int32(data.checksum), data.sensor_num)
		
	def reset_device(self, sensor_num=-1):
		"""
		Reset the device by sending corresponding byte to sensor
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors
		"""
		self.send_byte([0xF0], sensor_num)

	def reset_imu(self, sensor_num=-1):
		"""
		Reset the IMU by sending corresponding byte to sensor
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors
		"""
		self.send_byte([0xFA], sensor_num)

	def reset_dac(self, sensor_num=-1):
		"""
		Reset the DAC by sending corresponding byte to sensor
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors
		"""
		self.send_byte([0xFB], sensor_num)

	def reset_transducer(self, transducer_num, sensor_num=-1):
		"""
		Reset a transducer (ADS1257-A thru F)
		:param transducer_num the index of the transducer to reset, indexed 1-6
			Input 7 to reset all transducers
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors
		"""
		if transducer_num < 1 or transducer_num > 7:
			rospy.logwarn("Invalid Transducer Number")
			return
		self.send_byte([0xF0 + transducer_num], sensor_num)

	def deactivate_transducer(self, transducer_num, sensor_num=-1):
		"""
		Deactivate a transducer
		:param transducer_num the index of the transducer to deactivate, indexed 1-6
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors
		"""
		if transducer_num < 1 or transducer_num > 6:
			rospy.logwarn("Invalid Transducer Number")
			return
		self.send_byte([0xE0 + transducer_num], sensor_num)

	def config_imu(self, mode, delay, sensor_num=-1):
		"""
		Configure the IMU output
		:param mode the type of output to set the IMU to output
			Mode can be given as the string or int from the list below:
				1	Accelerometer
				2	Gyroscope
				4	Linear Acceleration
				5	Rotation Vector
				8	Game Rotation Vector
			The strings are not case sensitive and only the first 3 letters are considered
			so 'acc', 'accel', 'Acc', 'acceleration', 'AcCeletometer' are all valid strings for mode 1
		:param delay the interval between reports in ms
		"""
		byte = []

		if type(mode) == str:
			mode = mode.lower()
			
			if mode.startswith('acc'):
				byte = [0x21, delay]
			elif mode.startswith('gyr'):
				byte = [0x22, delay]
			elif mode.startswith('lin'):
				byte = [0x24, delay]
			elif mode.startswith('rot'):
				byte = [0x25, delay]
			elif mode.startswith('gam'):
				byte = [0x28, delay]
			else:
				rospy.logwarn('Invalid IMU Configuration Mode')
				return
		elif type(mode) == int:
			byte = [0x20 + mode, delay]
		else:
			return

		self.send_byte(byte, sensor_num=sensor_num)

	def set_imu_accelerometer(self, delay, sensor_num=-1):
		self.config_imu(1, delay, sensor_num)

	def set_imu_gyroscope(self, delay, sensor_num=-1):
		self.config_imu(2, delay, sensor_num)

	def set_imu_lin_accel(self, delay, sensor_num=-1):
		self.config_imu(4, delay, sensor_num)

	def set_imu_rotation(self, delay, sensor_num=-1):
		self.config_imu(5, delay, sensor_num)

	def set_imu_game_rot(self, delay, sensor_num=-1):
		self.config_iu(8, delay, sensor_num)

	def config_adc(self, adc_num, category, setting, sensor_num=-1):
		"""
		Configure the ADS1257-x device(s). These are ADCs with built in PGAs.
		:param adc_num specifies which of the six ADCs to control
			Indexing is 1-6. -1 targets every ADC at once
		:param category what configuration category to change. 
			The following are the available categories:
				0	'root' 		configure the root registers
				1	'drate'		configure the data rate
				2	'pga'		configure the PGA gain
				3	'pos'		configure the positive channel
				4	'neg'		configure the negative channel
			These names are not case sensitive, and the category can be selected 
			either by passing the string or the number from the above list 
		:param setting what to change the selected category's setting to
			The following are the available settings, organized by their categories
				0-Root Register Configuration:
					This setting should be passed as a binary tuple or string. The first element or character
					enables (1) or disables (0) ACAL. The second enables (1) or disables (0) IN-BUFF. 
						For example, (1,0) or '10' would both enable ACAL and disable IN_BUFF
				1-Data Rate Configuration
					The following rates in SPS are permitted. Any other number will be 
					rounded to the nearest value from this list:
						2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3
				2-PGA Gain Configuration
					The following PGA Gains are permitted. Any other number will be 
					rounded to the nearest value from this list:
						1, 2, 4, 8, 16, 32, 64
				3-Positive Channel Configuration
					Enter the int x to select AINx where x is 0, 1, 2, or 3
				4-Negative Channel Configuration
					Enter the int x to select AINx where x is 0, 1, 2, or 3
		:param sensor_num specifies which sensor this is done to. -1 indicates all sensors
		""" 
		if adc_num not in [-1,1,2,3,4,5,6]:
			rospy.logwarn('Invalid ADC number. Values between 1 and 6 are permitted, or -1 to select all')
			return

		#Set the first byte
		byte = [0x3 + adc_num] 
		
		#Select which category to work with
		cat_dict = {'root' : 0, 'drate' : 1, 'pga' : 2, 'pos' : 3, 'neg' : 4}
		if type(category) == str: #Convert string inputs
			category = cat_dict.get(category.lower(), value=-1)
		if category < 0 or category > 4: #Check validity
			rospy.logwarn('Invalid Category String')
			return

		if category == 0: #Configure Root Registers
			if (type(setting) != tuple and type(setting) != str) or len(setting) != 2 or setting[0] not in [0,1] or setting[1] not in [0,1]:
				rospy.logwarn('Invalid setting. Please enter a tuple or string with 2 binary elements to configure root registers')
				return

			config = int('000' + str(setting[0]) + str(setting[1]) + '000',2)

		elif category == 1: #Configure Data Rate
			rates = np.array([2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3])
			#Ensure we have an acceptable value
			if setting not in rates:
				diff = np.abs(rates - setting)
				setting = rates[np.argmin(diff)]
				rospy.logwarn('Data rate value not permitted. Rounding to ' + str(setting))

			drate = self.to_hex(np.where(rates == setting))
			config = int('1' + drate, 16)

		elif category == 2: #Configure PGA Gain
			gains = np.array([1, 2, 4, 8, 16, 32, 64])
			
			if setting not in gains:
				diff = np.abs(gains - setting)
				setting = gains[np.argmin(diff)]
				rospy.logwarn('Gain value not permitted. Rounding to ' + str(setting))

			gain = bin(np.where(gains == setting))[2:]
			while len(gain) < 5:
				gain = '0' + gain

			config = int('010' + gain, 2)

		elif category == 3: #Configure Positive Channel
			if setting not in [0,1,2,3]:
				rospy.logwarn('Please enter a channel between 0 and 4 (inclusive)')
				return
			config = int('3' + str(setting), 16)

		elif category == 4: #Configure Negative Channel
			if setting not in [0,1,2,3]:
				rospy.logwarn('Please enter a channel between 0 and 4 (inclusive)')
				return
			config = int('4' + str(setting), 16)

		byte.append(config)
		self.send_byte(byte, sensor_num=sensor_num)

	def set_adc_drate(self, adc_num, data_rate, sensor_num=-1):
		"""
		Convenience method that calls config_adc to set the data rate
		:param adc_num which ADC to do this to (1-6)
		:param data_rate the desired data rate. If this is not one allowed by config_adc,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		self.config_adc(adc_num, 1, data_rate, sensor_num)

	def set_adc_registers(self, adc_num, ACAL, IN_BUFF, sensor_num=-1):
			"""
			Convenience method that calls config_adc to configure the root registers
			:param adc_num which ADC to do this to (1-6)
			:param ACAL 1 to enable ACAL, 0 to disable it
			:param IN_BUFF 1 to enable IN_BUFF, 0 to disable it
			:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
			"""
			self.config_adc(adc_num, 0, (ACALL, IN_BUFF), sensor_num)

	def set_pga_gain(self, pga_num, gain, sensor_num=-1):
		"""
		Convenience method that calls config_adc to set the PGA's gain
		:param pga_num which PGA to do this to (1-6)
		:param gain the desired gain value. If this is not one allowed by config_adc,
			it will be rounded to the nearest allowed value
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		self.config_adc(pga_num, 2, gain, sensor_num)

	def set_adc_channel(self, adc_num, channel, positive=True, sensor_num=-1):
		"""
		Convenience method that calls config_adc to set the positive or negative channel number
		:param adc_num which ADC to do this to (1-6)
		:param positive
		:param channel the desired channel number (0-3)
		:param positive flag that states whether to change the positive or negative channel
			True changes the positive channel, false does the negative one
		:param sensor_num the index of the sensor to send the command to.
				The default value of -1 corresponds to sending the command to all the sensors	
		"""
		if pos:
			self.config_adc(adc_num, 3, channel, sensor_num)
		else:
			self.config_adc(adc_num, 4, channel, sensor_num)

	def config_dac(self, channel, voltage, sensor_num=-1):
		"""
		Configure the DAC by setting the output voltage of a specified channel or powering off a channel
		:param channel the channel to set the voltage of (int between 1 and 6, inclusive)
		:param voltage the voltage to set the output of the selected channel to
			input 0 to shut a channel off
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the bytes to all the sensors	
		"""
		
		if channel < 1 or channel > 6:
			rospy.logwarn('Invalid channel number. Input a number between 1 and 7, inclusive')
			return

		config = []
		if voltage == 0:
			config = [0x4, channel]
		else:
			volts_bin = bin(voltage)[2:]

			#Adjust the length to 12 bits
			while len(volts_bin) < 12:
				volts_bin = '0' + volts_bin

			#Make the last byte negative to indicate the CRC4 should be added to that byte, not in an additional byte
			config = [(4<<4) + channel, int(volts_bin[:8],2), -int(volts_bin[8:], 2)]
		self.send_byte(config, sensor_num=sensor_num)

	def to_hex(self, num):
		return "%x" % num

	def send_byte(self, byte, response_length=0, sensor_num=-1):
		"""
		Send a byte command to the serial port wrapper and tell it how
		many bytes to expect in response
		:param byte the byte command to send (int list)
		:param response_length the number of bytes expected to be returned by the sensor
			Default is 0, no response from sensor 
		:param sensor_num the index of the sensor to send the command to.
			The default value of -1 corresponds to sending the byte to all the sensors	
		"""

		#Create message
		cmd = ByteMsg()
		cmd.num_bytes = response_length
		cmd.sensor_num = sensor_num
		cmd.byte_data = byte

		rospy.logwarn('Sending byte ' + str(byte) + ' to sensor ' + str(sensor_num) + ', expecting ' + str(response_length) + ' bytes in return.')
		#Publish message
		self.user_cmds.publish(cmd)

if __name__ == "__main__":
	rospy.init_node('sensor_obj')
	sense = Sensor(num_sensors=int(rospy.get_param("num_sensors")))