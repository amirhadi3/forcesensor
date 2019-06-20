import roslaunch
import sys

if len(sys.argv) == 1:
	launch('force_sensor_2.launch')
elif len(sys.argv) == 2:
	if sys.argv[1] == '1':
		launch('force_sensor_1.launch')
	elif sys.argv[1] = '2':
		launch('force_sensor_2.launch')
	else:
		print('Invalid number of sensors. Enter 1 or 2.')
elif len(sys.argv) == 3 and sys.argv[2] == 'qt':
	if sys.argv[1] == '1':
		launch('force_sensor_1_qt.launch')
	elif sys.argv[1] = '2':
		launch('force_sensor_2_qt.launch')
	else:
		print('Invalid number of sensors. Enter 1 or 2.')
else:
	print('Invalid Arguments')


def launch(filename):
	file = "~/catkin_ws/src/forcesensor/launch" + filename
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch = roslaunch.parent.ROSLaunchParent(uuid, [file])
	launch.start()
