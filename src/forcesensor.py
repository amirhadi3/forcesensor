#! /usr/bin/env python

import rospy
import numpy as np
from forcesensor.msg import SensorOutput
from forcesensor.msg import UserCommand
from forcesensor.msg import ByteMsg
from forcesensor.msg import FlagMsg

from serial_port_wrapper import Serial_Device

class Sensor:

	def __init__(self, baud=5e6, num_sensors=1, portnum1=0, portnum2=1):

		rospy.init_node('sensor_obj' + str(sensor_num))
		self.num_sensors = num_sensors
		
		if num_sensors == 1:
			self.sensor = [Serial_Device(port=portnum1)]
		else:
			self.sensor = [Serial_Device(port=portnum1), Serial_Device(port=portnum2,sensor_num=1)]

		##Variables to store most recent sensor response and whether it has been read yet
		self.response = [] #Array of two objects, one for sensor 0 and one for sensor 1
		self.response_seen = [] #Array of 2 bools, sensor 0 and sensor 1

		#Create subscribers to read single responses from the sensor over the corresponding topics
		self.byte_response_pub = rospy.Subscriber('byte_response', ByteMsg, self.responses.callback)
		self.packet_response_pub = rospy.Subscriber('packet_response', SensorOutput, self.responses.callback)

		#Write commands from the user
		self.user_cmds = rospy.Publisher('user_commands', UserCommand, queue_size=1)
		#Start or stop continuous data transfer
		self.run_flag = rospy.Publisher('continuous_data_flag', FlagMsg, changeFlag)

	def __del__(self):
		"""
		When closing the force sensor object, tell all sensors to stop continuous data transfer
		and close all the ports
		"""
		stop_transmission = FlagMsg()
		stop_transmisison.data_flag = False

		for i in range(len(self.sensor)):
			stop_transmission.sensor_num = i
			self.run_flag.publish(stop_transmission)

			del self.sensor[i]

	def callback(self, msg):
		"""
		Callback for subscribers that receive response bytes from sensor
		Saves the message in instance variable 'response' and updates
		'response_seen' flag to indicate this is new data that has not yet been read
		"""
		sensor = msg.sensor_num

		if type(msg) == ByteMsg:
			#Take care of returned message by converting it to bytes, returning the bytes
			#Convert int to binary string
			binary = bin(msg.byte_data)[2:]
			#Create mutable byte array of correct size
			response_bytes = bytearray(msg.num_bytes)
			#Set each byte by slicing the binary string
			for i in range(msg.num_bytes):
				response_bytes[i] = int(binary[i*8:(i+1)*8],2)

			#Convert the bytearray to bytes
			msg = bytes(response_bytes)

		self.response[sensor] = msg
		self.response_seen[sensor] = False

	def get_response(self, sensor_num=0):
		"""
		Get the most recent response that was received from the sensor.
		Also sets the response_seen flag to True to indicate that this data
		is no longer new.
		:return response the data received from the sensor
			None if no data has been received yet
		:return has_seen indicates whether the data has been read before or not
			i.e. if this is new data or we have not received the new data yet
		"""
		has_seen = self.response_seen[sensor_num]
		self.response_seen[sensor_num] = True
		return self.response[sensor_num], has_seen

	def start_data_transfer(self, sensor_num=0):
		"""
		Tell the sensor to start sending data continuously
		This data is published to the continuous_data topic and can be accessed there
		"""
		flag = FlagMsg()
		flag.data_flag = True
		flag.sensor_num = sensor_num
		run_flag.publish(flag)

	def stop_data_transfer(self, sensor_num=0):
		"""
		Tell the sensor to stop sending continuous data
		"""
		flag = FlagMsg()
		flag.data_flag = False
		flag.sensor_num = sensor_num
		run_flag.publish(flag)

	def measure(self, sensor_num=0):
		"""
		Request one measurement from the sensor
		:return a parsed data packet in the form of a SensorOutput object
			None if the packet somehow fails to arrive
		"""
		self.send_byte(0x12, 53, sensor_num)

		data, seen = get_response(sensor_num)
		i = 0
		while seen:
			data, seen = get_response(sensor_num)
			if i > 1000 or data == -1:
				rospy.logwarn('Measurement of one packet failed')
				return None
			i += 1

		return data

	def reset_device(self, sensor_num=0):
		"""
		Reset the device by sending corresponding byte to sensor
		"""
		self.send_byte(0xF0, sensor_num)

	def reset_imu(self, sensor_num=0):
		"""
		Reset the IMU by sending corresponding byte to sensor
		"""
		self.send_byte(0xFA, sensor_num)

	def reset_dac(self, sensor_num=0):
		"""
		Reset the DAC by sending corresponding byte to sensor
		"""
		self.send_byte(0xFB, sensor_num=0, sensor_num)

	def reset_transducer(self, transducer_num, sensor_num=0):
		"""
		Reset a transducer
		:param transducer_num the index of the transducer to reset, indexed 1-6
		"""
		self.send_byte(0xF0 + transducer_num, sensor_num)

	def send_byte(self, byte, response_length=0, sensor_num=0):
		"""
		Send a byte command to the serial port wrapper and tell it how
		many bytes to expect in response
		:param byte the byte command to send (an int or bytes object)
		:param response_length the number of bytes expected to be returned by the sensor
			Default is 0, no response from sensor 
		"""
		#Convert to int if not already an int
		if type(byte) == bytes:
			byte = int(byte.hex,16)

		#Create message
		cmd = UserCommand()
		cmd.byte_command = byte
		cmd.expected_response_length = response_length
		cmd.sensor_num = sensor_num

		#Publish message
		self.user_cmds.publish(cmd)

	def pause_micro(self, us):
		"""
		Sleep for 'us' microseconds. This is blocking
		"""
		for i in range(us*10):
			a=1