import rospy
from sensor import Sensor
import time
import numpy as np
import matplotlib.pyplot as plt
from forcesensor_linux.msg import SensorOutput

BAUD = 8000000
HZ = 4000
N_SAMPLES = 30000
POLL = False
SAVE = True

s = Sensor(baud=BAUD)

times = []

########################### Polling ##################################
if POLL:
	count = 0
	maxi = N_SAMPLES * 5
	i = 0
	while count < N_SAMPLES and i < maxi:
		start = time.time()
		d = s.poll()
		if d != None:
			times.append(time.time()-start)
			count += 1
			if count % 1000 == 0:
				print(count)
		else:
			print('Failed one package')
		i += 1
	if SAVE:
		np.savetxt('/home/david/catkin_ws/src/forcesensor_linux/poll_' + str(N_SAMPLES) + 'samples.txt', np.array(times))
	plt.hist(np.array(times))
	plt.show()

########################### Continuous ##################################
else:
	rospy.init_node("latency_tester")
	times = [time.time()]
	count = 0
	def callback(msg):
		global count
		global times
		times.append(time.time())
		if count == N_SAMPLES:
			end()
		count += 1
		if count % 1000 == 0:
			print(count)

	def end():
		t = []
		for i in range(1,len(times)):
			t.append(times[i] - times[i-1])
		if SAVE:
			np.savetxt('/home/david/catkin_ws/src/forcesensor_linux/conti_' + str(N_SAMPLES) + 'samples_' + str(HZ) + 'hz.txt', np.array(t))
		plt.hist(np.array(t))
		plt.show()

	s.start_data_transfer(data_rate=HZ)
	time.sleep(1)
	sub = rospy.Subscriber('force_data',SensorOutput,callback)
	rospy.spin()