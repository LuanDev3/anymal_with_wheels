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
	try:
		print "Killing all ros instances..."
		killROS = subprocess.Popen(['pkill', '-9', '-f', 'ros']) # finishing ros
		killROS.communicate()
		print "Starting roscore..."
		FNULL = open(os.devnull, 'w')
		subprocess.call('roscore &', shell = True,  stdout=FNULL, stderr=subprocess.STDOUT)
		time.sleep(3)
		print "Starting demo..."
		subprocess.call('terminator -l PFC', shell = True)
	except KeyboardInterrupt:
		print "Finishing demo..."
		killROS = subprocess.Popen(['pkill', '-9', '-f', 'ros'])
		killROS.communicate()
		killTerminator = subprocess.Popen(['pkill', '-9', '-f', 'terminator'])
		killTerminator.communicate()

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