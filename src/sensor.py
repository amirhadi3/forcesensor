#! /usr/bin/env python
import crc
import rospy
import numpy as np
from forcesensor.msg import SensorOutput
from forcesensor.srv import SensorOutputRequest
from forcesensor.srv import ByteSrv
from forcesensor.msg import FlagMsg
import time
import re

class Sensor:

	def __init__(self, sensorNum):

		self.sensorNum = sensorNum
		self.crc8_table = crc.calculate_CRC8_table()
		self.crc32_table = crc.calculate_CRC32_table()

		#Write commands from the user
		# self.user_cmds = rospy.Publisher('user_commands' + str(sensorNum), ByteMsg, queue_size=1)
		#Start or stop continuous data transfer
		self.run_flag = rospy.Publisher('continuous_data_flag' + str(sensorNum), FlagMsg, queue_size=1)
		rospy.wait_for_service('poll' + str(sensorNum))
		self.pollProxy = rospy.ServiceProxy('poll' + str(sensorNum), SensorOutputRequest, persistent=True)
		rospy.wait_for_service('user_command' + str(sensorNum))
		self.commandProxy = rospy.ServiceProxy('user_command' + str(sensorNum), ByteSrv, persistent=True)

		self.inBuf = [False]*6
		self.adsGain = [1]*6
		self.adsRate = [30e3]*6
		self.vref = 2.5
		self.OFC = [1]*6
		self.FSC = [1]*6
		for i in range(6):
			o,f = self.ads_report_registers(i)
			self.OFC[i] = o
			self.FSC[i] = f

	def start_data_transmission(self, data_rate=1500):
		"""
		Tell the sensor to start sending data continuously
		This data is published to the continuous_data topic and can be accessed there
		"""
		flag = FlagMsg()
		flag.data_flag = 1
		flag.data_rate = data_rate
		flag.adsGain = self.adsGain
		flag.vref = self.vref
		flag.adsRate = self.adsRate
		flag.inBuf = self.inBuf
		flag.FSC = self.FSC
		flag.OFC = self.OFC

		rospy.loginfo('Starting data transfer at ' + str(data_rate) + 'Hz')
		self.run_flag.publish(flag)
		return 'Starting Data Transfer'

	def stop_data_transmission(self):
		"""
		Tell the sensor to stop sending continuous data
		"""
		flag = FlagMsg()
		flag.data_flag = 0
		flag.data_rate = 0
		flag.adsGain = self.adsGain
		flag.vref = self.vref
		flag.adsRate = self.adsRate
		flag.inBuf = self.inBuf
		flag.FSC = self.FSC
		flag.OFC = self.OFC
		self.run_flag.publish(flag)
		return 'Stopped Data Transfer'

	def poll(self):
		"""
		Request one measurement from the sensor
		:return a parsed data packet in the form of a SensorOutput object
			None if the packet somehow fails to arrive
		"""
		data = self.get_service(self.pollProxy, self.adsGain, self.vref, self.adsRate, self.inBuf, self.OFC, self.FSC)

		if data == None or data.report_id == 'NA':
			rospy.logwarn('Measurement of one packet failed: returned empty packet')
	 		return None
	 	else:
	 		return SensorOutput(data.wrench, data.differential, data.differential_raw, data.sum, data.sum_raw, data.imu, data.quaternion, data.saturated, data.temperature, data.report_id)
		
	def reset_device(self):
		"""
		Reset the device by sending corresponding byte to sensor
		"""
		return self.send_byte([0xF0, 0x00, 0x00])

	def reset_imu(self):
		"""
		Reset the IMU by sending corresponding byte to sensor
		"""
		return self.send_byte([0xFA, 0x00, 0x00])

	def reset_dac(self):
		"""
		Reset the DAC by sending corresponding byte to sensor
		"""
		return self.send_byte([0xFB, 0x00, 0x00])

	def reset_ads(self, ads_num):
		"""
		Reset and activate one of the ADS1257 adss
		:param ads_num the index of the ads to reset (0-5)
		"""
		if ads_num <= 5 and ads_num >= 0: 
			resp = self.send_byte([0xF0 + ads_num + 1, 0x00, 0x00])
			time.sleep(0.1)
			o,f = self.ads_report_registers(ads_num)
			if 2**17 == o or 2**17 == f:
				rospy.logwarn('Updating calibration constants failed. Please Retry')
			else:
				self.OFC[ads_num] = o
				self.FSC[ads_num] = f
			return resp + 'ADS' + str(ads_num+1) + '-RECAL-S. '
		elif ads_num == -1:
			resp = ''
			for i in range(6):
				resp+=self.reset_ads(i)
			return resp
		else: 
			rospy.logwarn('Invalid index')

	def deactivate_ads(self, ads_num):
		"""
		Deactivate a transducer
		:param transducer_num the index of the transducer to deactivate, indexed 1-6
		"""
		if ads_num == -1:
			resp = ''
			for i in range(6):
				resp += self.send_byte([0xE0 + i+1, 0x00, 0x00])
			return resp
		if ads_num < 5 and ads_num >= 0: 
			return self.send_byte([0xE0 + ads_num+1, 0x00, 0x00])
		else: 
			rospy.logwarn('Invalid index')

	def config_imu(self, mode, delay):
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

		interv = self.toHex(delay)
		while len(interv) < 4:
			interv = '0' + interv

		delay1 = int(interv[:2],16)
		delay2 = int(interv[2:],16)

		if type(mode) == str:
			mode = mode.lower()
			
			if mode.startswith('acc'):
				msg = 0x21
			elif mode.startswith('gyr'):
				msg = 0x22
			elif mode.startswith('lin'):
				msg = 0x24
			elif mode.startswith('rot'):
				msg = 0x25
			elif mode.startswith('gam'):
				msg = 0x26
			else:
				rospy.logwarn('Invalid IMU Configuration Mode')
				return
			byte = [msg, delay1, delay2]
		elif type(mode) == int:
			byte = [0x20 + mode, delay1, delay2]
		else:
			return

		return self.send_byte(byte)

	def set_imu_accelerometer(self, delay):
		return self.config_imu(1, delay)

	def set_imu_gyroscope(self, delay):
		return self.config_imu(2, delay)

	def set_imu_lin_accel(self, delay):
		return self.config_imu(4, delay)

	def set_imu_rotation(self, delay):
		return self.config_imu(5, delay)

	def set_imu_game_rot(self, delay):
		return self.config_imu(8, delay)

	def imu_start_calibration(self):
		flag = FlagMsg()
		flag.data_flag = 2
		flag.data_rate = 1500
		flag.adsGain = self.adsGain
		flag.vref = self.vref
		flag.adsRate = self.adsRate
		flag.inBuf = self.inBuf
		flag.FSC = self.FSC
		flag.OFC = self.OFC
		self.run_flag.publish(flag)
		return self.send_byte([0x2F, 0x01, 0x00])

	def imu_cancel_calibration(self):
		"""
		Stop the calibration without saving it
		"""
		flag = FlagMsg()
		flag.data_flag = 0
		flag.data_rate = 1500
		flag.adsGain = self.adsGain
		flag.vref = self.vref
		flag.adsRate = self.adsRate
		flag.inBuf = self.inBuf
		flag.FSC = self.FSC
		flag.OFC = self.OFC
		self.run_flag.publish(flag)
		return self.send_byte([0x2F, 0x03, 0x00])

	def imu_finish_calibration(self):
		"""
		Complete the IMU calibration, saving the Dynamic Calibration Data (DCD0 to the IMU)
		"""
		flag = FlagMsg()
		flag.data_flag = 0
		flag.data_rate = 1500
		flag.adsGain = self.adsGain
		flag.vref = self.vref
		flag.adsRate = self.adsRate
		flag.inBuf = self.inBuf
		flag.FSC = self.FSC
		flag.OFC = self.OFC
		self.run_flag.publish(flag)
		return self.send_byte([0x2F, 0x02, 0x00])

	def config_ads(self, ads_num, category, setting):
		"""
		Configure the ADS1257-x device(s). These are adss with built in PGAs.
		:param ads_num specifies which of the six adss to control
			Indexing is 0-5. -1 targets every ads at once
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
				6- Issues self-calibration 
		""" 
		if ads_num not in [-1,0,1,2,3,4,5]:
			rospy.logwarn('Invalid ads number. Values between 1 and 6 are permitted, or -1 to select all')
			return
		if ads_num == -1:
			resp = ''
			for i in range(6):
				resp += self.config_ads(i,category,setting)
			return resp
		else:
			#Set the first byte
			byte = [0x30 + ads_num+1] 
			resp = []
			#Select which category to work with
			cat_dict = {'root' : 0, 'drate' : 1, 'pga' : 2, 'pos' : 3, 'neg' : 4}
			if type(category) == str: #Convert string inputs
				category = cat_dict.get(category.lower(), value=-1)
			if category < 0 or category > 6: #Check validity
				rospy.logwarn('Invalid Category String')
				return

			if category == 0: #Configure Root Registers
				if (type(setting) != tuple and type(setting) != str) or len(setting) != 2 or setting[0] not in [0,1] or setting[1] not in [0,1]:
					rospy.logwarn('Invalid setting. Please enter a tuple or string with 2 binary elements to configure root registers')
					return
				self.inBuf[ads_num] = setting[1]==1
				config = int(str(setting[0]) + str(setting[1]) + '000000',2)
				byte.append(0x00)
				byte.append(config)

			elif category == 1: #Configure Data Rate
				rates = np.array([2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1e3, 2e3, 3.75e3, 7.5e3, 15e3, 30e3])
				#Ensure we have an acceptable value
				if setting not in rates:
					diff = np.abs(rates - setting)
					setting = rates[np.argmin(diff)]
					rospy.logwarn('Data rate value not permitted. Rounding to ' + str(setting))

				self.adsRate[ads_num] = setting

				config = np.where(rates == setting)[0][0]
				byte.append(0x01)
				byte.append(config)

			elif category == 2: #Configure PGA Gain
				gains = np.array([1, 2, 4, 8, 16, 32, 64])
				
				if setting not in gains:
					diff = np.abs(gains - setting)
					setting = gains[np.argmin(diff)]
					rospy.logwarn('Gain value not permitted. Rounding to ' + str(setting))

				self.adsGain[ads_num] = setting

				config = np.where(gains == setting)[0][0]
				byte.append(0x02)
				byte.append(config)

			elif category == 3: #Configure Positive Channel
				if setting not in [0,1,2,3]:
					rospy.logwarn('Please enter a channel between 0 and 3 (inclusive)')
					return
				byte.append(0x03)
				byte.append(setting)

			elif category == 4: #Configure Negative Channel
				if setting not in [0,1,2,3]:
					rospy.logwarn('Please enter a channel between 0 and 3 (inclusive)')
					return
				byte.append(0x04)
				byte.append(setting)

			elif category == 6: #self-calibrate
				byte.append(0x06)
				byte.append(0x00)

		resp = self.send_byte(byte)

		o,f = self.ads_report_registers(ads_num)
		if o==2**25 or f==2**25:
			rospy.logwarn('Updating calibration constants failed. Please Retry')
		else:
			self.OFC[ads_num] = o
			self.FSC[ads_num] = f

		return resp

	def set_ads_drate(self, ads_num, data_rate):
		"""
		Convenience method that calls config_ads to set the data rate
		:param ads_num which ads to do this to (1-6)
		:param data_rate the desired data rate. If this is not one allowed by config_ads,
			it will be rounded to the nearest allowed value
		"""
		return self.config_ads(ads_num, 1, data_rate)

	def set_ads_registers(self, ads_num, ACAL, IN_BUFF):
			"""
			Convenience method that calls config_ads to configure the root registers
			:param ads_num which ads to do this to (1-6)
			:param ACAL 1 to enable ACAL, 0 to disable it
			:param IN_BUFF 1 to enable IN_BUFF, 0 to disable it
			"""
			return self.config_ads(ads_num, 0, (ACAL, IN_BUFF))

	def set_pga_gain(self, pga_num, gain):
		"""
		Convenience method that calls config_ads to set the PGA's gain
		:param pga_num which PGA to do this to (1-6)
		:param gain the desired gain value. If this is not one allowed by config_ads,
			it will be rounded to the nearest allowed value
		"""
		return self.config_ads(pga_num, 2, gain)

	def set_ads_channel(self, ads_num, channel, positive=True):
		"""
		Convenience method that calls config_ads to set the positive or negative channel number
		:param ads_num which ads to do this to (1-6)
		:param positive
		:param channel the desired channel number (0-3)
		:param positive flag that states whether to change the positive or negative channel
			True changes the positive channel, false does the negative one
		"""
		if positive:
			return self.config_ads(ads_num, 3, channel)
		else:
			return self.config_ads(ads_num, 4, channel)

	def ads_self_calibrate(self, ads_num):
		"""
		Convenience method that calls config_ads to self-calibrate the selected ADS
		:param ads_num which ads to do this to (1-6)
		"""
		return self.config_ads(ads_num, 6, 0)

	def ads_report_registers(self, ads_num):
		"""
		Reads the calibration registers of the ADS and reports the OFC and FSC registers 
		:param ads_num which ads to do this to (1-6)
		:return OFC register value, FSC register value
			-1 for both if reading the response failed
		"""
		
		if ads_num == -1:
			for i in range(6):
				ofc, fsc = self.ads_report_registers(i)
			return ofc, fsc

		resp = self.send_byte([0x30+ads_num+1,0x05,0x00], returnType='list')
		ok = False
		startIndex = 0
		try:
			startIndex = resp.index(0xAA)+1
			ok = self.check_crc(resp[startIndex+6],resp[startIndex-1:startIndex+6],8)
		except ValueError:
			rospy.logwarn('Failed to report FSC and OSC registers')
			return 2**25, 2**25

		if ok:
			ofc = self.to_int(resp[startIndex:startIndex+3],lsb_first=True)
			#Check if ofc is negative and find 2's complement accordingly
			if ofc >= 2**23:
				ofc -= 2**24
			fsc = self.to_int(resp[startIndex+3:startIndex+6],lsb_first=True)
			return ofc, fsc
		else:
			rospy.logwarn('Failed to report FSC and OSC registers: Checksum failed')
			return 2**25, 2**25

	def config_dac(self, channel, voltage):
		"""
		Configure the DAC by setting the output voltage of a specified channel or powering off a channel
		:param channel the channel to set the voltage of (int between 1 and 6, inclusive)
		:param voltage the voltage to set the output of the selected channel to
			input 0 to shut a channel off
		"""
		
		if channel == -1:
			resp = ''
			for i in range(6):
				resp += self.config_dac(i, voltage)
			return resp
		else:
			voltage = voltage * 0xFFF / 50

			config = []

			if channel < 0 or channel > 5:
				print('Invalid channel number. Input a number between 1 and 7, inclusive')
				return

			channel += 1
			if voltage == 0:
				config = [0x47, channel<<4, 0x00]
			else:
				volts_hex = self.toHex(voltage)

				#Adjust the length to 12 bits
				while len(volts_hex) < 3:
					volts_hex = '0' + volts_hex

				config = [(4<<4) + channel, int(volts_hex[:1],16), int(volts_hex[1:], 16)]
			return self.send_byte(config)

	def turn_on_led(self, ledNum):
		self.config_dac(ledNum, 0x666)
	def turn_on_leds(self,ledNums):
		for num in ledNums:
			self.config_dac(num, 0x666)
	def turn_on_leds_all(self):
		for i in range(1,7):
			self.config_dac(i,0x666)
	def turn_off_led(self, ledNum):
		self.config_dac(ledNum, 0)
	def turn_off_leds(self,ledNums):
		for num in ledNums:
			self.config_dac(num, 0)
	def turn_off_leds_all(self):
		for i in range(1,7):
			self.config_dac(i,0)

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

	def send_byte(self, byte, returnType='string'):
		"""
		Send a byte command to the serial port wrapper and tell it how
		many bytes to expect in response
		:param byte the byte command to send (int list)
		"""
		#Initiate service and send bytes
		resp = self.get_service(self.commandProxy, byte).response

		#Parse response into nicer format
		h = ''
		regex = '\w+-\w+-[SF]'
		if resp != []:
			for num in resp:
				h += self.toHex(num, padded=True)
			h = h.decode('hex')

			#Try to match regex
			if regex != None:
				match = re.findall(regex,h)
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
		if returnType == 'string':
			return h
		else:
			return [int(i) for i in resp]

	def get_service(self, proxy, *args):
		try:
			return proxy(*args)
		except rospy.ServiceException as exc:
			rospy.logwarn('Service did not process request: ' + str(exc))
			return None

if __name__ == "__main__":
	rospy.init_node('sensor_obj')
	sense = Sensor()