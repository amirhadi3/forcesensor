#! /usr/bin/env python

import rospy
import ftd2xx
from optiforce.msg import SensorOutput
from optiforce.msg import UserCommand
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class Serial_Device:
	def __init__(self, port=0, sensor_num=1):
		##Start serial port wrapper to take care of communication to sensor
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, ["C:/Users/dgblack.stu/Documents/catkin_ws/src/optiforce/launch/serial_port_wrapper.launch"])
		launch.start()

		self.port_num = port
		self.baud = 5e6
		self.init_byte = 0xFF
		self.conti_read_flag = False
		self.sensor_num = sensor_num
		self.report_ids = {
			0x01: 'Accelerometer',
			0x02: 'Gyroscope',
			0x04: 'Linear Acceleration',
			0x05: 'Rotation Vector',
			0x08: 'Game Rotation Vector'
			}

		###Set up ROS
		rospy.init_node('serial_port_wrapper' + str(self.sensor_num))
		self.rate = rospy.Rate(1500)

		#queue_size limits the number of queued messages if a subscriber is reading too slowly
		#Create publisher to write continuous data to topic using custom message type
		self.continuous_data_pub = rospy.Publisher('continuous_data', SensorOutput, queue_size=10)
		#Create publishers to write single responses to topics for sensor object to read
		self.byte_response_pub = rospy.Publisher('byte_response', Int32, queue_size=1)
		self.packet_response_pub = rospy.Publisher('packet_response', SensorOutput, queue_size=1)
		#Listen to user commands and interrupt action to carry out the command
		self.user_cmds = rospy.Subscriber('user_commands', UserCommand, self.send_byte)
		#Listen to user commands to dstart or stop continuous data transfer
		self.run_flag = rospy.Subscriber('continuous_data_flag' + str(self.sensor_num), Bool, self.changeFlag)

		###Initialize serial port
		#Connect sensor serial port given by portNum (Usually 0,1, or 2)
		self.port = ftd2xx.open(self.port_num) 
		#Set latency timer to 1ms (lowest possible value. Ideally would be lower but limited by USB frame size)
		self.port.setLatencyTimer(1)
		#Set read an write timeouts 
		self.port.setTimeouts(16,16)
		#Set USB package size to smallest possible value to avoid waiting for package until timeout for package to fill
		self.port.setUSBParameters(64)
		#Set baud rate to rate specified above
		self.port.setBaudRate(self.baud)
		reset(self.port) #reset transmit and receive buffers

		##Run loop - do this forever in the background
		while not rospy.is_shutdown():
			while CONTINUOUS_READ_FLAG:
				#Wait for initialization bit and matching CRC-4
				if wait_for_packet(port):
					data = port.read(51)
					#Check CRC-32 of data packet
					if check_crc(to_int(data[47:]), data[:47]):
						#Parse and publish if correct, otherwise ignore
						parsed = parse(data)
						continuous_data_pub.publish(parsed)
				rate.sleep()
			rate.sleep()

		port.write(b'\x11')
		port.purge()
		port.close()

	def send_byte(msg):
		"""
		Callback function for the user command topic. This is called whenever the user issues a command
		other than start/stop continuous data transfer. The message contains two ints. First, the command byte
		that is sent to the sensor, and second, the length of the expected response of the sensor. This function
		attempts to read in this many bytes and publishes them to the byte_response topic. For the case of the byte
		command being 0x12, we expect a data packet to be sent, so it will be checked for CRC correctness, parsed,
		and written to the packet_response topic 	
		"""

		port.write(bytes([msg.command_byte]))
		port.purge(1)
		#If we expect a response, read in the expected length
		if msg.expected_response_length > 0:
			#Special case where we read one packet
			if msg.command_byte == 12:
				count = 0
				while count < 100:
					#Wait for init byte or timeout
					if wait_for_packet(port):
						data = port.read(51)
						#Check CRC-32 of data packet
						if check_crc(to_int(data[47:]), data[:47]):
							#Parse and publish if correct
							parsed = parse(data)
							packet_response_pub.publish(parsed)
							break
						else:
							#If CRC was wrong, try again
							port.write(bytes([msg]))
							port.purge(1)
					else: #If timed out before start byte found try again
						port.write(bytes([msg]))
						port.purge(1)
					count += 1
				if count == 100:
					byte_response_pub.publish(-1)
			else:
				#Usually just read response byte
				data = port.read(msg.expected_response_length)
				byte_response_pub.publish(int(data.hex(),16))

	def reset(port):
			"""
			Reset all input and output buffers until there are no more bytes waiting.
			If the buffer keeps filling up with new data after the reset, this will try
			100 times to clear the buffers before it gives up because something is continuosly
			writing to the buffer
			"""
			i = 0
			while port.getQueueStatus() > 0 and i < 100:
				port.resetDevice()
				i += 1

			if i == 100:
				print('Failed to reset. Ensure nothing is actively sending data')

	def check_crc(crc, p, n=32, polynomial=0xedb88320):
		    """
		    Check CRC Checksum with arbitrary number of bits
		    :param crc the n bit checksum in hex or int - note that the sensor returns LSB first, 
		        so can't just use sensor output here
		    :param p the packet to compare to the checksum. (This is bytes 0 to 46 in the 51 byte sensor packet)
		    :param polynomial the bit string of the CRC polynomial to use
		    :return True if the checksum matches, False otherwise
		    """
		    
		    #Convert p to correct type
		    if type(p) == bytes:
		   		p = int(p.hex(),16)

		    #Construct the number: <4 bit counter><init byte><4 bit crc>
		    p = (p << n) + crc #Append the crc to the end of the number
		    pBin = bin(p)[2:] #Store the binary representation of the number
		    length = len(pBin)
		    poly = polynomial << length - len(bin(polynomial))+2 #Shift the polynomial to align with most significant bit
		    minVal = 2**n #When p gets smaller than this, the dividend is zero
		    i = 0 #Start aligned with most significant bit

		    while p >= minVal: #Terminate when the dividend is equal to zero and only the checksum portion remains
		        while pBin[i] == '0' and i <= length-n: #Shift the divisor until it is aligned with the most significant 1
		            i = i + 1
		            poly = poly >> 1
		        p = p ^ poly #XOR the number with the divisor
		        pBin = bin(p)[2:] #Update the bit string for checking
		        #Make sure leading zeros weren't removed by Python
		        while len(pBin) < length:
		            pBin = '0' + pBin
		    return p == 0

	def wait_for_packet(port, timeout=100):
		for i in range(timeout):
			#Check for initialization byte
			if port.read(1) == bytes([INIT_BYTE]):
				#Read next byte
				byte = port.read(1)

				#Store 4 LSBs of the byte (the checksum)
				crc = byte[0] & 0x0F

				#Combine initialization byte and counter by shifting 4 bits and adding
				p = (INIT_BYTE << 4) + ((byte[0] & 0xF0) >> 4) #4 MSBs of byte are the counter

				#Test checksum
				return check_crc(crc,p,4,0x9)
		return False

	def parse(byte_data):
		"""
		Parse the data packet of 51 bytes using the SH-2 structure
		:param byte_data 51-byte-long bytes object
		:return sensor_output object which contains the parsed data in a useful form
		"""

		#Create sensor output object
		data_out = SensorOutput()

		#Convert all 6 values in differentials and sums to ints
		data_out.differential = [to_int(byte_data[i:i+3]) for i in {0, 3, 6, 9, 12, 15}]
		data_out.sum = [to_int(byte_data[i:i+3]) for i in {18, 21, 24, 27, 30, 33}]
		
		data_out.report_id = REPORT_IDS[byte_data[37]]
		data_out.sequence_num = byte_data[38]
		status = '{0:08b}'.format(byte_data[39])
		data_out.accuracy = int(status[-2:],2) #0=Unreliable, 1=Low, 2=Medium, 3=High
		data_out.delay = (byte_data[40] + int(status[0:-2] + '0000000',2)) * 100e-6 #delay in seconds

		#LSB is first byte that is returned
		x = to_int(byte_data[41:43])
		y = to_int(byte_data[43:45])
		z = to_int(byte_data[45:47])
		data_out.imu = {x,y,z}

		#Again, LSB comes first
		data_out.checksum = to_int(byte_data[47:])

		#Note which sensor this came from
		data_out.sensor_num = self.sensor_num

		return data_out

	def to_int(byte):
		"""
		Helper method to convert a bytes object where the least significant byte is first into an int
		"""
		num = 0
		for i in range(len(byte)):
			num += (byte[i] << i*8)
		return num

	def changeFlag(msg):
		"""
		Change the flag to notify the main loop about whether it should measure
		Also send the command byte to the sensor that tells it to start or stop
		transmission of continuous data
		"""
		self.conti_read_flag = msg

		if msg == True:
			#Prompt the start of continuous data transmission
			port.write(b'\x10')
			port.purge()
		else:
			#Prompt the end of continuous data transmission
			port.write(b'\x11')
			port.purge()

	




