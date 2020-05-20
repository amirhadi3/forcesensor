#! /usr/bin/env python

import rospy
import pylibftdi as ftdi
import time
from forcesensor.msg import SensorOutput
from forcesensor.srv import ByteSrv
from forcesensor.srv import SensorOutputRequest
from forcesensor.msg import FlagMsg
from diagnostic_msgs.msg import KeyValue
import crc
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import numpy as np
import struct
import sys
import codecs
import re


BYTES_PER_DIGIT_IN = 1
ERR_MAX_NUM = 100
INIT_BYTE = 0xAA

class Serial_Device:

	def __init__(self, path, baud=3000000, portNum=1, rate=1500):
		self.num_errors = 0

		self.crc8_table = crc.calculate_CRC8_table()
		self.crc32_table = crc.calculate_CRC32_table()

		self.garbage_response = {'differential':[0,0,0,0,0,0], 
								'differential_raw':[0,0,0,0,0,0],
								'sum':[0,0,0,0,0,0],
								'sum_raw':[0,0,0,0,0,0],
								'imu':[0,0,0,0],
								'quaternion':[0,0,0,0],
								'saturated':[0,0,0,0,0,0],
								'temperature':0,
								'report_id':'NA'}

		self.baud = baud
		self.conti_read_flag = False
		self.report_ids = {
			0x01 : 'Accelerometer',
			0x02 : 'Gyroscope',
			0x04 : 'Linear Acceleration',
			0x05 : 'Rotation Vector',
			0x08 : 'Game Rotation Vector'
			}

		#Find calibration matrix
		self.calMatrix = np.eye(6)
		self.filename = path + '/src/calibration.cal'
		self.get_cal_matrix()

		###Initialize serial port
		try:
			self.port = ftdi.Device('USB-COM485 Plus2', interface_select=portNum)
		except ftdi.FtdiError:
			rospy.logerr('Failed to open port. Quitting...')
			sys.exit(0)
		self.port.ftdi_fn.ftdi_set_latency_timer(1)
		self.port.ftdi_fn.ftdi_set_line_property(8,1,0)
		self.port.baudrate =self.baud
		self.portNum = portNum
		rospy.loginfo('Opened port ' + str(self.portNum))
		
		self.reset() #reset transmit and receive buffers

		self.ab = {
			2.5:(0x5DC000,2.7304),
			5:(0x5DC000,2.7304),
			10:(0x5DC000,2.7304),
			15:(0x3E8000,1.8202),
			25:(0x4B0000,2.1843),
			30:(0x3E8000,1.8202),
			50:(0x4B0000,2.1843),
			60:(0x3E8000,1.8202),
			100:(0x4B0000,2.1843),
			500:(0x3C0000,1.7474),
			1000:(0x3C0000,1.7474),
			2000:(0x3C0000,1.7474),
			3750:(0x400000,1.8639),
			7500:(0x400000,1.8639),
			15000:(0x400000,1.8639),
			30000:(0x400000,1.8639)
		}

		self.inBuf = [False]*6
		self.adsGain = [1]*6
		self.adsRate = [30e3]*6
		self.vref = 2.5
		self.OFC = [1]*6
		self.FSC = [1]*6

		###Set up ROS
		self.sampleRate = rate
		self.rate = rospy.Rate(self.sampleRate)

		#queue_size limits the number of queued messages if a subscriber is reading too slowly
		#Create publisher to write continuous data to topic using custom message type
		self.force_data_pub = rospy.Publisher('force_data' + str(portNum), SensorOutput, queue_size=1)
		#Listen to user commands and interrupt action to carry out the command
		self.user_cmds = rospy.Service('user_command' + str(portNum), ByteSrv, self.send_byte)
		#Listen to user commands to dstart or stop continuous data transfer
		self.run_flag = rospy.Subscriber('continuous_data_flag' + str(portNum), FlagMsg, self.changeFlag)
		#Service for reading and parsing one measurement
		self.measure_srv = rospy.Service('poll' + str(portNum), SensorOutputRequest, self.poll)
		#Publisher for IMU Calibration
		self.imu_cal_pub = rospy.Publisher('imu_calibration' + str(portNum), KeyValue, queue_size=1)

		#Let all the topics start properly
		rospy.sleep(1)
		self.times = []
		count = 0
		##Run loop - do this forever in the background
		while not rospy.is_shutdown():
			self.rate.sleep()
			
			if self.conti_read_flag == 1: #Continuous data transfer
				#Wait for initialization bit and matching crc-4
				if self.wait_for_packet(verbose=False):
					parsed = self.found_init_byte()
					try:
						if parsed != None:
							self.force_data_pub.publish(parsed)
							count += 1
						else:
							rospy.logwarn('Error parsing data')
					except rospy.ROSSerializationException:
						rospy.logwarn("Error publishing. Serialization Exception caught. Continuing...")

			elif self.conti_read_flag == 0 and count != 0: #Data transmission has just been stopped
				rospy.loginfo('Stopping Continuous Data Transfer...')
				self.port.write(b'\x11\x00\x00\xC9') #11 is the byte command, C9 is the matching crc8
				self.purge()
				rospy.sleep(0.25)
				timeout = 0
				count = 0
				while self.readBytes(1) != []:
					self.port.write(b'\x11\x00\x00\xC9')
					self.purge()
					rospy.sleep(0.25)
					timeout += 1
					if timeout == 100:
						rospy.logwarn('Failed to stop transmission. Please try again')
						break

			elif self.conti_read_flag == 2: #Calibrate IMU
				self.calibrate_imu
		self.port.close()
		self.measure_srv.shutdown('Finished')
		self.user_cmds.shutdown('Finished')
		

	def __del__(self):
		self.port.close()

	def get_cal_matrix(self):
			"""
			Read the .cal file into a calibration matrix
			"""
			try:
				root = xml.parse(str(self.filename)).getroot()
				axes = root.findall('Calibration/Axis')
				self.calMatrix = np.array([[float(axes[i].attrib['values'].split()[j]) for j in range(6)] for i in range(6)])
			except:
				self.calMatrix = np.eye(6)

	def poll(self, msg):
		if self.conti_read_flag:
			rospy.logwarn('Cannot read package while continuously transmitting data')
			return self.garbage_response

		self.adsGain = msg.adsGain
		self.vref = msg.vref
		self.adsRate = msg.adsRate
		self.inBuf = msg.inBuf
		self.OFC = msg.OFC
		self.FSC = msg.FSC
		for i in range(6):
			self.a = self.ab[self.adsRate[i]][0]
			self.b = self.ab[self.adsRate[i]][1]

		#Send command to sensor
		self.port.write(b'\x12\x00\x00\x74')

		#Wait for init byte or timeout
		if self.wait_for_packet(verbose=False):
			dat = self.found_init_byte()
			if dat != None:
				#Convert SensorOutput to SensorOutputRequest
				try:
					return  {'wrench':dat.wrench,
							'differential':dat.differential, 
							'differential_raw':dat.differential_raw, 
							'sum':dat.sum, 
							'sum_raw':dat.sum_raw, 
							'imu':dat.imu, 
							'quaternion':dat.quaternion, 
							'saturated':dat.saturated, 
							'temperature':dat.temperature, 
							'report_id':dat.report_id}

				except OverflowError:
					rospy.logwarn("Failed to retrieve one measurement.")
					return self.failed_package()
			else:
				return self.failed_package()
		else: #If timed out before start byte found, try again
			return self.failed_package()

	def failed_package(self):
		rospy.logwarn("Failed to retrieve one measurement.")
		return self.garbage_response

	def found_init_byte(self):
		data = self.readBytes(53)

		#Check crc-32 of data packet
		if data != None and self.check_crc(data[49:], data[:49]):
			#Parse and publish if correct
			parsed = self.parse(data)
			return parsed
		else:
			#If crc was wrong, return none
			rospy.logwarn('CRC32 Checksum failed')
			return None

	def calibrate_imu(self):
		"""
		Parse the responses given by the sensor during calibration
		:return Status, statusType where status is the accuracy of the game rotation vector/magnetic field output
			and statusType is which mode is being reported (mag or grvec)
			Accuracy is given as:
				0		Unreliable
				1		Accuracy Low
				2		Accuracy Medium
				3		Accuracy High 
		"""
		dat = self.readLine(startChar='G', startChar2='M')

		datStr = ''
		for num in dat:
			datStr += self.toHex(num, padded=True)

		datStr = datStr.decode('hex')

		idx = datStr.find(':')

		ret = KeyValue()
		try:
			status = dat[idx+1]
			ret.key = datStr[:idx]
			ret.value = str(status-48)
		except:
			ret.key = ''
			ret.value = '-1'

		self.imu_cal_pub.publish(ret)

	def readLine(self, startChar=None, startChar2=None, endChar='\n', timeout=100):
		"""
		Attempt to read one line from serial, starting with startChar, and ending with endChar
		:param startChar the character to expect as the first character of the line.
			None by default allows any character to be the first character
		:param endChar the character at which to stop reading
			By default the new line character, '\n'
		:param timeout how many bytes to read, looking for the startChar and then endChar before giving up
		"""
		if startChar != None:
			startByte = int(codecs.encode(startChar, 'hex'),16)
			startByte2 = int(codecs.encode(startChar2, 'hex'),16)
		if endChar != None:
			endByte = int(codecs.encode(endChar, 'hex'),16)

		#Look for the start byte
		d = self.readBytes(1)
		count = 0
		while d != [startByte] and d != [startByte2] and count < timeout:
			d = self.readBytes(1)
			count += 1

		#Exit if it couldn't be found
		if count == 1000:
			return []

		#Look for the end byte
		line = []
		count = 0
		while d != [endByte] and count < timeout:
			if d != []:
				line.append(d[0])
			d = self.readBytes(1)
			count += 1

		#Return what came between start and end (inclusive)
		return line

	def send_byte(self, msg):
		"""
		Callback function for the user command topic. This is called whenever the user issues a command
		other than start/stop continuous data transfer. The message contains two ints. First, the command byte
		that is sent to the sensor, and second the length of the expected response of the sensor. This function
		attempts to read in this many bytes and publishes them to the byte_response topic. For the case of the byte
		command being 0x12, we expect a data packet to be sent, so it will be checked for crc correctness, parsed,
		and written to the packet_response topic.
		"""

		#Include the crc-8
		toWrite = self.toStr(msg.byte_data, with_crc8=True)
		try:
			self.port.write(toWrite)
			rospy.loginfo('Relayed ' + codecs.encode(toWrite, 'hex') + ' to sensor')
		except ftdi.FtdiError:
			rospy.logwarn('Failed to send command to sensor. Connection error.')
			return

		rospy.sleep(0.1)
		resp = self.readPacket()
		h = ''
		for num in resp:
			h += self.toHex(num, padded=True)
		h = h.decode('hex')
		match = re.findall('\w+-\w+-[FS]',h)

		if match != []:
			h = ''
			count = 0
			for string in match:
				h += string + '. '
				count += 1
				if count % 4 == 0:
					h += '\n'
			if h[-1] != '\n':
				h += '\n'

		rospy.loginfo(h)
		return {'response':resp}

	def readPacket(self, timeout=200):
		"""
		Read arbitrary package and return as list of integer bytes
		"""
		count = 0
		d = self.readBytes(1)
		data = []
		#Wait for start of package
		while d == []:
			d = self.readBytes(1)
			count = count + 1
			if count == timeout:
				rospy.loginfo('No response received')
				return data

		count = 0
		failed = 0
		while failed < 20:
			if d == []:
				failed += 1
			else:
				data.append(d[0])
				count = count + 1
				if count > timeout:
					break
			d = self.readBytes(1)

		return data
					
	def reset(self, timeout=100):
		"""
		Reset all input and output buffers until there are no more bytes waiting.
		If the buffer keeps filling up with new data after the reset, this will try
		100 times to clear the buffers before it gives up because something is continuosly
		writing to the buffer
		"""
		i = 0
		while self.readBytes(1) != [] and i < 100:
			self.purge()
			i += 1

		if i == timeout:
			rospy.logwarn('Failed to reset. Ensure nothing is actively sending data')

	def purge_rx(self,verbose=True):
		if self.port.ftdi_fn.ftdi_usb_purge_rx_buffer() == 0:
			if verbose:
				rospy.loginfo("Receive buffer cleared")
			return 0
		else:
			rospy.logwarn("Error clearing receive buffer")
			return -1
	def purge_tx(self, verbose = True):
		if self.port.ftdi_fn.ftdi_usb_purge_tx_buffer() == 0:
			if verbose:
				rospy.loginfo("Transmit buffer cleared")
			return 0
		else:
			rospy.logwarn("Error clearing transmit buffer")
			return -1

	def purge(self, verbose=True):
		return self.purge_tx(verbose) + self.purge_rx(verbose)

	def check_crc(self, crcval, p, n=32):
	    """
	    Check crc Checksum with 8 or 32 bits
	    :param crc the n bit checksum as a list of bytes (ints) or an int
	    :param p the list of bytes to compare to the checksum. (This is bytes 0 to 46 in the 53 byte sensor packet)
	    :param polynomial the bit string of the crc polynomial to use
	    	Default is 0x04C11DB7 which is what we use for n=32. For n=8, 0x07 is used
	    :return True if the checksum matches, False otherwise
	    """
	    if type(p) == int:
	    	p = self.toBytesList(self.toHex(p))
	    if type(crcval) != int:
	    	crcval = self.to_int(crcval)

	    if n == 8:
	    	checksum = crc.crc8(p, table=self.crc8_table)
	    elif n == 32:
	    	checksum = crc.crc32(p, table=self.crc32_table)

	    return checksum == crcval

	def wait_for_packet(self, timeout=200, verbose=True):
		"""
		Scan the incoming data for the initialization byte. If one is found, test the checksum in the next byte. 
		If the checksum matches, return True. The program should then read in the next 53 bytes as a packet.
		:param timeout how many bytes to attempt before stopping and giving up, returning False
		:param verbose Set to true to print out all the extra bytes that were read in before receiving a star byte or timing out
		:return True if the program can read in the next 53 bytes as a valid data packet
			False if no start byte + checksum combination is found within timeout tries
		"""
		data = ''
		for i in range(timeout):
			#Check for initialization byte
			dat = self.readBytes(1) 
			if dat == [INIT_BYTE]:
				#Print all the garbage that was read in
				if verbose and data != '':
					rospy.loginfo(data)

				#Read counter and CRC8
				byte = self.readBytes(2)
				if byte != None and byte != []:
					counter = byte[0]
					crcval = byte[1]
				else:
					continue

				#Combine initialization byte and counter
				p = [INIT_BYTE, counter]

				#Test checksum
				return self.check_crc(crcval,p,8)
			else:
				if dat != [] and dat != None:
					data += self.toHex(dat[0],padded=True)

		#Print all the garbage that was read in
		if verbose and data != '':
			rospy.loginfo(data.decode('hex'))
		
		return False

	def parse(self, byte_data):
		"""
		Parse the data packet of 53 bytes using the SH-2 structure
		:param byte_data 53-byte-long bytes object
		:return sensor_output object which contains the parsed data in a useful form
		"""

		#Create sensor output object
		data_out = SensorOutput()

		data_out.saturated = [0]*6
		a = []
		b = []
		for i in range(6):
			a.append(self.ab[self.adsRate[i]][0])
			b.append(self.ab[self.adsRate[i]][1])

		#Convert all 6 values in differentials and sums to ints
		data_out.differential_raw = [np.int32(self.to_int(byte_data[i:i+3])) for i in range(0,18,3)]
		data_out.differential = [self.volts_ads(data_out.differential_raw[i], self.adsGain[i], self.vref, a[i], b[i], self.OFC[i], self.FSC[i]) for i in range(6)]

		data_out.sum_raw = [np.int32(self.to_int(byte_data[i:i+3])) for i in range(18,36,3)]
		data_out.sum = [self.volts_adc(data_out.sum_raw[i],self.vref) for i in range(6)]

		for i in range(6):
			data_out.saturated[i] = self.checkSaturation(data_out.differential[i],data_out.sum[i],self.inBuf[i],self.adsGain[i])
		
		wrench = [0,0,0,0,0,0] if 0 in data_out.sum else [data_out.differential[i] / data_out.sum[i] for i in range(6)]
		wrench = np.matmul(self.calMatrix, wrench) 
		data_out.wrench = [wrench[i] for i in range(6)]

		try:
			rid = byte_data[36]
			data_out.report_id = self.report_ids[rid]
		except KeyError:
			rospy.logwarn('Invalid report ID')
			data_out.report_id = 'NA'

		#Configure Q Point based on report ID
		qpoint=0
		if rid == 1 or rid == 4:
			qpoint = 8
		elif rid == 2:
			qpoint = 9
		elif rid == 8:
			qpoint = 14

		#Parse IMU data
		#LSB is first byte that is returned
		if rid != 5:
			imu = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), qpoint) for i in range(37,47,2)]
			data_out.imu = imu[:4]
		else:
			quat = [self.q_to_float(self.to_int(byte_data[i:i+2], lsb_first=True), 14) for i in range(37,43,2)]
			quat.append(self.q_to_float(self.to_int(byte_data[43:45], lsb_first=True), 12))
			quat.append(self.q_to_float(self.to_int(byte_data[45:47], lsb_first=True), 12))
			data_out.imu = quat[:4]

		if rid == 5 or rid == 8:
			#Normalise if need be
			mag = 0
			for num in data_out.imu:
				mag += num**2
			mag = np.sqrt(mag)
			if mag > 1.1:
				data_out.imu = [data_out.imu[i] / mag for i in range(4)]

			data_out.quaternion = data_out.imu
		else:
			data_out.quaternion = None

		#Parse Temperature Data
		temp = self.to_int(byte_data[47:49])
		if byte_data[47] >= 2048: #The number begins with 1 and is thus negative
			temp -= 2**12 #2's complement
		data_out.temperature = temp * 0.0625

		return data_out

	def checkSaturation(self, Vdiff, Vsum, buff, gain):
		"""
		Based on table 7.3 (pg. 6) of:
		http://www.ti.com/lit/ds/symlink/ads1257.pdf
		"""
		#Check absolute input voltage
		Vbb = 4.775
		Vainn = -Vsum + Vbb - 0.5*Vdiff
		Vainp = -Vsum + Vbb + 0.5*Vdiff

		if buff:
			minV = 0
			maxV = 3
		else:
			minV = -0.1
			maxV = 5.1

		if Vainn > maxV or Vainn < minV or Vainp > maxV or Vainp < minV:
			return 1
		#Check differential voltage
		if abs(Vdiff) > 5 / gain:
			return 1
		#Check sum voltage
		if abs(Vsum) > 2.5:
			return 1
		return 0

	def volts_ads(self, v, G, Vref, a, b, OFC, FSC):
		"""
		Find Vin from Vout according to the conversion given in Equation 4 (Pg. 34) of 
		http://www.ti.com/lit/ds/symlink/ads1257.pdf
		"""
		if v >= 2**23:
			v -= 2**24

		return np.float32((v/(b*FSC) + OFC/a) * (2*Vref/G))

	def volts_adc(self, v, Vref):
		"""
		Find Vin from Vout according to the conversion given in Equation 2 (Pg. 9) of 
		https://www.intel.com/content/dam/www/programmable/us/en/pdfs/literature/hb/max-10/ug_m10_adc.pdf
		"""
		return np.float32(v * (Vref/2**12))

	def toHex(self, num, padded=False):
		"""
		Convert to hex without the "0x" at the start or the random "L" at the end
		"""
		h = "%x" % num
		if padded:
			if len(h) % 2 != 0:
				h = h + '0'
			return h
		else:
			return h

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

		self.adsGain = msg.adsGain
		self.vref = msg.vref
		self.adsRate = msg.adsRate
		self.inBuf = msg.inBuf
		self.OFC = msg.OFC
		self.FSC = msg.FSC
		for i in range(6):
			self.a = self.ab[self.adsRate[i]][0]
			self.b = self.ab[self.adsRate[i]][1]

		self.num_errors = 0 #Reset error count

		self.conti_read_flag = msg.data_flag

		if msg.data_flag == 1:
			#Split up the data rate into two rate bytes
			self.rate = rospy.Rate(msg.data_rate) #Update the ROS rate
			self.sampleRate = msg.data_rate

			data_rate = self.toHex(msg.data_rate)
			while len(data_rate) < 4:
				data_rate = '0' + data_rate

			byte1 = int(data_rate[:2],16)
			byte2 = int(data_rate[2:],16)

			#Prompt the start of continuous data transmission
			self.port.write(self.toStr([0x10, byte1, byte2], with_crc8=True))
		elif msg.data_flag == 0:
			#Prompt the end of continuous data transmission
			self.port.write(self.toStr([0x11, 0x00, 0x00], with_crc8=True))
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

	def toBytesList(self, string):
		"""
		Split a hexadecimal string of any length into a list of bytes
		:param string the hexadecimal string to convert
		"""
		string = codecs.encode(string, 'hex')

		#Watch out for leading zeros
		if len(string) % 2 != 0:
			string = '0' + string

		#Split into list and return
		return [int(string[i:i+2],16) for i in range(0,len(string),2)]

	def toStr(self, byte_list, with_crc8=False, format=0):
		"""
		Do the opposite of toBytesList. Convert a list of bytes (array of 8-bit ints in decimal)
		to a byte string in hex.
		:param byte_list the list of bytes (as decimal ints) to convert to a byte string
		:param with_crc8 1 if a crc8 checksum for the number given by byte_list should be appended to the string
			Default is False, so no crc8 is computed or added
		:param format can take the value 0 (bytes) or 1 (string))
			If format = 0, numbers will undergo the following conversion: 15 -> 0x0f -> b'\x0f' which is a bytes
			object useful for sending to the sensor. This works in Python 2.7 and 3
			If format = 1, numbers will be converted directly to a hex string: 15 -> '0f', which is actually 2 bytes b'\x30\x66'.
			These are the ASCII values of the characters. This is not useable for sending to a sensor.
		"""
		string = b''

		if format == 0:
			string = self.toBytes(byte_list)
		else:
			for i in range(len(byte_list)):
					string += self.toHex(byte_list[i])

		if with_crc8 == True:
			if format == 0:
				string += self.toBytes(crc.crc8(byte_list, table=self.crc8_table))
			else:
				string += self.toHex(crc.crc8(byte_list, table=self.crc8_table))

		return string

	def toBytes(self,byteList):
		"""
		The only reliable Python 2 and 3- compatible int-to-bytes conversion I could find 
		"""
		byte = ''

		if type(byteList) == int:
			byteList = [byteList]

		for num in byteList:
			byte += struct.pack("B", num)
		return byte

	def readBytes(self, num_bytes):
		try:
			read_in = self.port.read(num_bytes * BYTES_PER_DIGIT_IN)
		except ftdi._base.FtdiError:
			self.num_errors += 1
			rospy.logwarn('Connection problem. Threw "usb bulk read failed" error')
			if self.num_errors > ERR_MAX_NUM:
				rospy.logerr('Persistent connection problem. Closing...')
				rospy.shutdown_signal()
			return
		try:
			retval = self.toBytesList(read_in)
			return retval
		except ValueError:
			self.num_errors += 1
			rospy.logwarn('Connection problem. Read failed')
			if self.num_errors > ERR_MAX_NUM:
				rospy.logerr('Persistent connection problem. Closing...')
				rospy.shutdown_signal()
			return None

if __name__ == "__main__":
	portNum = int(rospy.get_param('portNum'))
	baudrate = int(rospy.get_param('baudrate'))
	rate = int(rospy.get_param('rate'))
	path = rospy.get_param('path')

	rospy.init_node('port' + str(portNum))

	port = Serial_Device(path=path, baud=baudrate, portNum=portNum, rate=rate)