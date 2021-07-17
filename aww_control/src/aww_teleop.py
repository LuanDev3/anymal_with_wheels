#!/usr/bin/env python2

import yaml
import rospy
import rospkg
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Twist


class AwwTeleop():
    def __init__(self):
        rospy.init_node('aww_teleop', anonymous=True)
        rospy.loginfo("AWW teleop initialized!")
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('aww_control')
        self.joy_mapper = rospy.get_param('~joy_config', self.path + '/config/joy_mapper.yaml')

        with open(self.joy_mapper) as file:
            mapper = yaml.safe_load(file)

        if mapper:
            self.linear_position_an = mapper['structure']['linear_analogic']
            self.linear_position_ds = mapper['structure']['linear_discrete']
            self.rotation_position_an = mapper['structure']['rotation_analogic']
            self.rotation_position_ds = mapper['structure']['rotation_discrete']
            self.boost_position = mapper['structure']['boost']
            self.linear_gain = mapper['gains']['linear']
            self.rotation_gain = mapper['gains']['rotation']
            self.boost_linear_gain = mapper['gains']['linear_boost']
            self.boost_rotation_gain = mapper['gains']['rotation_boost']
            self.linear = 0
            self.rotation = 0
        else:
            raise AttributeError("The joy config file cannot be opened.")

        rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.pub = rospy.Publisher('/aww/cmd_vel', Twist, queue_size=1)

    def joyCallback(self, data):
        if (data.buttons[self.boost_position]):
            self.linear = (self.linear_gain + self.boost_linear_gain)* \
                (self.mutex(data.axes[self.linear_position_an], data.axes[self.linear_position_ds]))
            self.rotation = (self.rotation_gain + self.boost_rotation_gain)* \
                (self.mutex(data.axes[self.rotation_position_an], data.axes[self.rotation_position_ds])) 
        else:
            self.linear = (self.linear_gain)* \
                (self.mutex(data.axes[self.linear_position_an], data.axes[self.linear_position_ds]))
            self.rotation = (self.rotation_gain)* \
                (self.mutex(data.axes[self.rotation_position_an], data.axes[self.rotation_position_ds]))

    def mutex(self, input_an, input_ds):
        if input_an == 0.0:
            return input_ds
        elif input_ds == 0.0:
            return input_an
        else:
            return input_an


    def run(self):
        rate = rospy.Rate(30) # 30Hz
        while not rospy.is_shutdown():
            vel = Twist()
            vel.linear.x = self.linear
            vel.angular.z = self.rotation
            self.pub.publish(vel)
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    awwTeleop = AwwTeleop()
    awwTeleop.run()