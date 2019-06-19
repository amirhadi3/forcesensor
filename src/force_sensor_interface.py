#! /usr/bin/env python

import rospy
from forcesensor.msg import SensorOutput
from forcesensor.msg import UserCommand
from forcesensor.msg import ByteMsg
from serial_port_wrapper import Serial_Device
from forcesensor import Sensor
from PySide2.QtWidgets import (QApplication, QLabel, QPushButton, QVBoxLayout, QWidget, QRadioButton)
from PySide2.QtCore import Slot, Qt
from PySide2.QtGui import QPainter
from pyqtgraph.widgets.MatplotlibWidget import MatplotlibWidget

class  gui(QWidget):
	def __init__(self, sensor, app):
		QWidget.__init__(self)
		self.sensor = sensor
		self.transducer_num = 1
		self.num = 0
		self.measurement = None

		if sensor.num_sensors == 2
			self.device_num_selector = QRadioButton('Click for sensor 2, leave unchecked for sensor 1')
			self.device_num_selector.toggled.connect(self.device_num_selector_callback)

		self.start_data_transfer_button = QPushButton('Start Data Transfer')
		self.start_data_transfer_button.clicked.connect(self.start_data_transfer_wrapper)

		self.stop_data_transfer_button = QPushButton('Stop Data Transfer')
		self.stop_data_transfer_button.clicked.connect(self.stop_data_transfer_wrapper)

		self.measure_button = QPushButton('Request One Measurement')
		self.measure_button.clicked.connect(self.measure_wrapper)

		self.reset_button = QPushButton('Reset Device')
		self.reset_button.clicked.connect(self.reset_wrapper)

		self.reset_transducer_button = QPushButton('Reset Transducer')
		self.reset_transducer_button.clicked.connect(self.reset_transducer_wrapper)

		self.reset_imu_button = QPushButton('Reset IMU')
		self.reset_imu_button.clicked.connect(self.reset_imu_wrapper)

		self.reset_dac_button = QPushButton('Reset DAC')
		self.reset_imu_button.clicked.connect(self.reset_dac_wrapper)

		self.pltwidget1 = MatplotlibWidget()
		self.diffs1 = self.pltwidget.getFigure().add_subplot(311)
		self.sums1 = self.pltwidget.getFigure().add_subplot(312)
		self.imu1 = self.pltwidget.getFigure().add_subplot(313)

		self.pltwidget2 = MatplotlibWidget()
		self.diffs2 = self.pltwidget.getFigure().add_subplot(311)
		self.sums2 = self.pltwidget.getFigure().add_subplot(312)
		self.imu2 = self.pltwidget.getFigure().add_subplot(313)

		self.qp = QtGui.QPainter()

	def start_data_transfer_wrapper(self):
		self.sensor.start_data_transfer(self.num)
	def stop_data_transfer_wrapper(self):
		self.sensor.stop_data_transfer(self.num)
	def measure_wrapper(self):
		self.measurement = self.sensor.measure(self.num)
	def reset_wrapper(self):
		self.sensor.reset(self.num)
	def reset_transducer_wrapper(self):
		self.sensor.reset_transducer(self.transducer_num, self.num)
	def reset_imu_wrapper(self):
		self.sensor.reset_imu(self.num)
	def reset_dac_wrapper(self):
		self.sensor.reset_dac(self.num)
	def device_num_selector_callback(self):
		if self.device_num_selector.isChecked():
			self.num = 1
		else:
			self.num = 0
	def data_subscriber(self, msg):
		if msg.sensor_num == 0:
			self.diff_data1.append(msg.differentials)
			self.sum_data1.append(msg.sum)
			self.imu_data1.append(msg.imu)
	def drawRectangle(self, qp):

if __name__ == "__main__":
	rospy.init_node('force_sensor_gui')

	sensor = Sensor(num_sensors=2)
	app = QApplication([])
	GUI = gui(sensor, app)
