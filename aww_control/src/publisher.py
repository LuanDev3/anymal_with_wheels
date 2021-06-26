#!/usr/bin/env python2

from std_msgs.msg import Float64
import rospy

class SimplePublisher():
	def __init__(self):
		rospy.init_node('publisher', anonymous=False)
		self.pubJointPosition = []
		self.pubJointPosition.append(rospy.Publisher('/aww/LF_HAA_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/LF_HFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/LF_KFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RF_HAA_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RF_HFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RF_KFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/LH_HAA_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/LH_HFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/LH_KFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RH_HAA_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RH_HFE_position_controller/command', Float64, queue_size=10))
		self.pubJointPosition.append(rospy.Publisher('/aww/RH_KFE_position_controller/command', Float64, queue_size=10))
		self.rate = rospy.Rate(50)

	def run(self):
		values = [0.0, 1.18, -2.12, 0.0, 1.18, -2.12, 0.0, -1.18, 2.12, 0.0, -1.18, 2.12]
		while not rospy.is_shutdown():
			value = Float64()
			for i, pub in enumerate(self.pubJointPosition):
				value.data = values[i]
				pub.publish(value)
			self.rate.sleep()


if __name__ == '__main__':
	simplePublisher = SimplePublisher()
	simplePublisher.run()