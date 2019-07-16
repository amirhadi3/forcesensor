#! /usr/bin/env python

import rospy
import ftd2xx
import time
from forcesensor.msg import SensorOutput
from forcesensor.msg import ByteMsg
from forcesensor.srv import SensorOutputRequest
from forcesensor.msg import FlagMsg
import CRC
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import numpy as np

#Currently everything has to be sent as a string in Python 2.7 so a byte like
#0xF0 is sent as 'F0', which is 2 bytes for the serial communication. So any number of
#bytes being read in/sent out should be multiplied by 2 
BYTES_PER_DIGIT_IN = 2
BYTES_PER_DIGIT_OUT = 2

class Serial_Device:

	def __init__(self):

		self.crc4_table = CRC.calculate_CRC4_table()
		self.crc32_table = CRC.calculate_CRC32_table()

		self.garbage_response = {
			'differential':[0,0,0,0,0,0],
			'sum': [0,0,0,0,0,0],
			'imu': [0,0,0,0,0],
			'report_id': '',
			'checksum': -2,
			'sensor_num': -2
			}

		self.INIT_BYTE = 0xAA
		self.baud = 5000000
		self.conti_read_flag = False
		self.report_ids = {
			1 : 'Accelerometer',
			2 : 'Gyroscope',
			4 : 'Linear Acceleration',
			5 : 'Rotation Vector',
			8 : 'Game Rotation Vector'
			}

		###Initialize serial port
		#Connect sensor serial port given by portNum (Usually 0 or 1)
		# self.port = ftd2xx.open(port_num)
		try:
			self.port = ftd2xx.open(0)
			self.port_num = 0
			self.sensor_num = 0
			rospy.logwarn('Opened port 0')
		except ftd2xx.ftd2xx.DeviceError:
			self.port = ftd2xx.open(1)
			self.port_num = 1
			self.sensor_num = 1
			rospy.logwarn('Opened port 1')

		#Set latency timer to 1ms (lowest possible value. Ideally would be lower but limited by USB frame size)
		self.port.setLatencyTimer(1)
		#Set read an write timeouts 
		self.port.setTimeouts(16,16)
		#Set USB package size to smallest possible value to avoid waiting for package until timeout for package to fill
		self.port.setUSBParameters(64)
		#Set baud rate to rate specified above
		self.port.setBaudRate(self.baud)
		self.reset() #reset transmit and receive buffers

		###Set up ROS
		rospy.init_node('port' + str(self.sensor_num))
		self.rate = rospy.Rate(1500)

		#queue_size limits the number of queued messages if a subscriber is reading too slowly
		#Create publisher to write continuous data to topic using custom message type
		self.force_data_pub = rospy.Publisher('force_data', SensorOutput, queue_size=10)
		#Listen to user commands and interrupt action to carry out the command
		self.user_cmds = rospy.Subscriber('user_commands', ByteMsg, self.send_byte)
		#Listen to user commands to dstart or stop continuous data transfer
		self.run_flag = rospy.Subscriber('continuous_data_flag', FlagMsg, self.changeFlag)
		#Service for reading and parsing one measurement
		self.measure_srv = rospy.Service('ft_read', SensorOutputRequest, self.one_measurement)

		##Run loop - do this forever in the background
		while not rospy.is_shutdown():
			self.rate.sleep()
			start = time.time()
			count = 0
			while self.conti_read_flag:
				#Wait for initialization bit and matching CRC-4
				if self.wait_for_packet(print_garbage=False):
					data = self.readBytes(51)
					if data == None:
						continue #Ignore faults
					#Check CRC-32 of data packet
					if self.check_crc(data[47:], data[:47]):
						#Parse and publish if correct, otherwise ignore
						parsed = self.parse(data)
						try:
							self.force_data_pub.publish(parsed)
							#rospy.loginfo('Publishing')
							count += 1
							if count == 1500:
								stop = time.time()
								rospy.logwarn(stop-start)
						except rospy.ROSSerializationException:
							continue
				self.rate.sleep()
			
		self.port.write('110E') #11 is the byte command, 0E is the matching CRC4
		self.port.purge()
		self.port.close()
		

	def __del__(self):
		self.port.close()
		rospy.delete_param('has_sensor')

	def one_measurement(self, msg):
		if msg.to_sensor_num != self.sensor_num:
			rospy.logwarn('Wrong sensor number')
			return self.garbage_response
		if self.conti_read_flag:
			rospy.logwarn('Cannot read package while continuously transmitting data')
			return self.garbage_response

		#Include the CRC-4
		toWrite = self.toStr([18], with_crc4=True)
		self.port.write(toWrite)
		self.port.purge(1)

		#Try a few times to read in a response
		count = 0
		while count < 5:
			#Wait for init byte or timeout
			if self.wait_for_packet():
				dat = self.found_init_byte(toWrite)
				if dat != None:
					#Convert SensorOutput to SensorOutputRequest
					try:
						return  {'differential':dat.differential, 'sum':dat.sum, 'imu':dat.imu, 'report_id':dat.report_id, 'checksum':np.int32(dat.checksum), 'sensor_num':np.int32(dat.sensor_num)}
					except OverflowError:
						return self.one_measurement()
			else: #If timed out before start byte found, try again

				self.port.write(toWrite)
				self.port.purge(1)
			count += 1
		if count == 100: #Return empty thing 
			return self.garbage_response

	def found_init_byte(self, toWrite):
		data = self.readBytes(51)

		#Check CRC-32 of data packet
		if data != None and self.check_crc(data[47:], data[:47]):
			#Parse and publish if correct
			parsed = self.parse(data)
			return parsed
		else:
			#If CRC was wrong, return none
			return None

	def send_byte(self, msg):
		"""
		Callback function for the user command topic. This is called whenever the user issues a command
		other than start/stop continuous data transfer. The message contains three ints. First, the command byte
		that is sent to the sensor, second the length of the expected response of the sensor, and third the intended 
		sensor to carry out the instruction. If sensor_num matches this sensor's number or is -1, this function
		attempts to read in this many bytes and publishes them to the byte_response topic. For the case of the byte
		command being 0x12, we expect a data packet to be sent, so it will be checked for CRC correctness, parsed,
		and written to the packet_response topic.
		The sensor_num of -1 indicates that the same should be done on every connected sensor
		"""
		#Check if this sensor is the target of the command
		if msg.sensor_num != self.sensor_num and msg.sensor_num != -1:
			return

		#Include the CRC-4
		toWrite = self.toStr(msg.byte_data, with_crc4=True)

		self.port.write(toWrite)
		self.port.purge(1)
		
		#Read 11 bytes or timeout
		flag = self.wait_for_packet(11)
		if flag:
			data = self.found_init_byte(toWrite)
			if data != None:
				try:
					self.force_data_pub.publish(data)
				except rospy.ROSSerializationException:
					return
					
	def reset(self, timeout=100):
		"""
		Reset all input and output buffers until there are no more bytes waiting.
		If the buffer keeps filling up with new data after the reset, this will try
		100 times to clear the buffers before it gives up because something is continuosly
		writing to the buffer
		"""
		i = 0
		while self.port.getQueueStatus() > 0 and i < 100:
			self.port.resetDevice()
			i += 1

		if i == timeout:
			print('Failed to reset. Ensure nothing is actively sending data')


	def check_crc(self, crc, p, n=32, polynomial=0x04C11DB7):
	    """
	    Check CRC Checksum with 4 or 32 bits
	    :param crc the n bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
	    :param polynomial the bit string of the CRC polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32. For n=4, 0x03 is used
	    :return True if the checksum matches, False otherwise
	    """
	    if type(p) == int:
	    	p = self.to_hex(p)
	    	p = self.toBytes(p)
	    if type(crc) != int:
	    	crc = self.to_int(crc)

	    if n == 4:
	    	checksum = CRC.crc4(p, polynomial, self.crc4_table)
	    elif n == 32:
	    	checksum = CRC.crc32(p, polynomial, self.crc32_table)

	    return checksum == crc

	def wait_for_packet(self, timeout=100, print_garbage=True):
		data = []
		for i in range(timeout):
			#Check for initialization byte
			dat = self.readBytes(1) 
			if dat == [self.INIT_BYTE]:
				#Print all the garbage that was read in
				if print_garbage and data != []:
					string = ''
					for b in data:
						byte = str(b)
						if len(byte) != 2:
							byte = '0' + byte
						string += byte
					rospy.loginfo(string)

				#Read next byte
				byte = self.readBytes(1)
				if byte != None and byte != []:
					byte = byte[0]
				else:
					continue

				#Store 4 LSBs of the byte (the checksum)
				crc = byte & 0x0F

				#Combine initialization byte and counter by shifting 4 bits and adding
				p = (self.INIT_BYTE << 4) + ((byte & 0xF0) >> 4) #4 MSBs of byte are the counter

				#Test checksum
				return self.check_crc(crc,p,4,0x3)
			else:
				if dat != [] and dat != None:
					data.append(dat[0])

		#Print all the garbage that was read in
		if print_garbage and data != []:
			string = ''
			for b in data:
				byte = str(b)
				if len(byte) != 2:
					byte = '0' + byte
				string += byte
			rospy.loginfo(string)
		return False

	def parse(self, byte_data):
		"""
		Parse the data packet of 51 bytes using the SH-2 structure
		:param byte_data 51-byte-long bytes object
		:return sensor_output object which contains the parsed data in a useful form
		"""

		#Create sensor output object
		data_out = SensorOutput()

		#Convert all 6 values in differentials and sums to ints
		data_out.differential = [self.to_int(byte_data[i:i+3]) for i in range(0,18,3)]
		data_out.sum = [self.to_int(byte_data[i:i+3]) for i in range(18,36,3)]
		
		#Save report ID
		try:
			rid = byte_data[36]
			data_out.report_id = self.report_ids[rid]
		except KeyError:
			rospy.logwarn('Invalid report ID key')
			data_out.report_id = 'None'
		
		#Configure Q Point based on report ID
		if rid == 1 or rid == 4:
			qpoint = 8
		elif rid == 2:
			qpoint = 9
		elif rid == 8:
			qpoint = 14

		#Parse IMU data
		#LSB is first byte that is returned
		if rid != 5:
			data_out.imu = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), qpoint) for i in range(37,47,2)]
		else:
			quat = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), 14) for i in range(37,43,2)]
			quat.append(self.q_to_float(self.to_int(byte_data[43:45], lsb_first=True), 12))
			quat.append(self.q_to_float(self.to_int(byte_data[45:47], lsb_first=True), 12))
			data_out.imu = quat

		#LSB comes last
		data_out.checksum = self.to_int(byte_data[47:])

		#Note which sensor this came from
		data_out.sensor_num = self.sensor_num

		return data_out

	def to_hex(self, num):
		return "%x" % num

	def q_to_float(self, qInt, qPoint):
		"""
		Convert the Q-Point Number in qInt with its decimal point at position qPoint
		(counting from the LSB, starting at 0) into a floating point decimal number
		"""
		return float(qInt) * 2**(-qPoint)

	def changeFlag(self, msg):
		"""
		Change the flag to notify the main loop about whether it should measure
		Also send the command byte to the sensor that tells it to start or stop
		transmission of continuous data
		"""
		if msg.sensor_num != self.sensor_num and msg.sensor_num != -1:
			return

		self.conti_read_flag = msg.data_flag

		if msg.data_flag == True:
			#Split up the data rate into a byte and another 4 bits
			data_rate = self.to_hex(msg.data_rate)
			while len(data_rate) < 3:
				data_rate = '0' + data_rate

			byte1 = int(data_rate[:2],16)
			byte2 = int(data_rate[2:3],16)

			#Prompt the start of continuous data transmission
			self.port.write(self.toStr([16, byte1, -byte2], with_crc4=True))
			self.port.purge(1)
		else:
			#Prompt the end of continuous data transmission
			self.port.write(self.toStr([17], with_crc4=True))
			self.port.purge(1)
			self.reset()

   	def to_int(self, byte, lsb_first=False):
		"""
		Helper method to convert a list of bytes where the least significant byte is 
		first or last into an int
		:param byte the list of bytes to convert to an int
		:param lsb_first True if the least significant byte of the number is listed first, False otherwise
			Default is false, so Most Significant Byte is assumed to be first
		"""
		num = 0
		sz = len(byte)

		if lsb_first: #LSB is first
			for i in range(sz):
				num += (byte[i] << i*8)
		else: #LSB is last
			for i in range(sz):
				num += (byte[sz - 1 - i] << i*8)

		return num

	def toBytes(self, string):
		"""
		Split a hexadecimal string of any length into a list of bytes
		:param string the hexadecimal string to convert
		"""

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def toStr(self, byte_list, with_crc4=False):
		"""
		Do the opposite of toBytes. Convert a list of bytes (array of 8-bit ints in decimal)
		to a string in hex. If the last byte of the list is negative, the CRC4 is placed in the 4 LSBs of that byte
		rather than as an additional byte at the end
		:param byte_list the list of bytes to convert to a hexadecimal string
		:param with_crc4 True if a CRC4 checksum for the number given by byte_list should be appended to the string
			Default is False, so no CRC4 is computed or added
		"""
		string = ''

		#If the last element is negative, make sure the CRC4 is desired
		if byte_list[-1] < 0:
			if with_crc4==False:
				#If not, the last element should not be negative 
				byte_list[-1] *= -1
			else:
				#If yes, find the CRC4 first, then shift the last byte 4 bits and add the CRC4
				crc = CRC.crc4(byte_list, table=self.crc4_table)
				byte_list[-1] *= -1
				byte_list[-1] <<= 4
				byte_list[-1] += crc
				with_crc4 = False #Make sure we don't add another CRC4

		for i in range(len(byte_list)):
				string += self.to_hex(byte_list[i])

		if with_crc4:
			string += '0' + self.to_hex(CRC.crc4(byte_list, table=self.crc4_table))

		return string

	def readBytes(self, num_bytes):
		try:
			read_in = self.port.read(num_bytes * BYTES_PER_DIGIT_IN)
			retval = self.toBytes(read_in)
			return retval
		except ValueError:
			return None

if __name__ == "__main__":
	port = Serial_Device()
		




