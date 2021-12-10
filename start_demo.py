#!/usr/bin/python
import roslib
import rospy
import subprocess
import os
import getpass
import time
import rospkg

def replace():
	rospack = rospkg.RosPack()
	rospack.list()
	path = rospack.get_path('aww_gazebo')
	file = "/".join(path.split("/")[:-1]) + "/config"
	subprocess.call('mv ~/.config/terminator/config ~/.config/terminator/config.bak', shell = True)
	subprocess.call('cp '+ file + ' ~/.config/terminator/config', shell = True)

def run():
	subprocess.call('killall -9 roscore &', shell = True) # start roscore
	time.sleep(1)
	subprocess.call('roscore &', shell = True) # start roscore
	time.sleep(3)
	subprocess.call('terminator -l PFC &', shell = True) # return the config file to normal

def restoreConfig():
	subprocess.call('mv ~/.config/terminator/config.bak ~/.config/terminator/config', shell = True) # return the config file to normal


if __name__ == '__main__':
	try:	
		subprocess.call('exit', shell = True)
		replace()
		time.sleep(1)
		run()
		time.sleep(1)
		restoreConfig()
		subprocess.call('exit', shell = True)

	except rospy.ROSInterruptException:
		rospy.loginfo("[Node]: Closed!")
	except KeyboardInterrupt:
		subprocess.call('[keyboard Interruption]', shell = True)