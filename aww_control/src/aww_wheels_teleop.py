#!/usr/bin/env python2

import yaml
import rospy
import rospkg
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from utils import aww_change_posture


WHEELED = "wheeled"
WALKING = "walking"
FREE  = "free"

class Color:
    YELLOW = '\033[93m'
    END = '\033[0m'


def formatString(initialString, color=Color.YELLOW):
    return color + initialString + Color.END


class AwwTeleop():
    def __init__(self):
        rospy.init_node('aww_teleop', anonymous=True, log_level=getattr(rospy, 'DEBUG'))
        rospy.loginfo("AWW teleop initialized!")
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('aww_control')
        self.walkStatus = True
        self.allow_control = True
        self.joy_mapper = rospy.get_param('~joy_config', self.path + '/config/joy_mapper.yaml')
        self.awwChangePosture = aww_change_posture.AwwChangePosture()

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
        rospy.Subscriber("/aww/mode", String, self.handleRobotMode)
        rospy.Subscriber("/aww/leg_control_status", String, self.handleLegControlStatus)
        self.pub = rospy.Publisher('/aww/cmd_vel', Twist, queue_size=1)

    def handleRobotMode(self, data):
        if  data.data == WHEELED :
            rospy.loginfo("Receiving mode wheleed! wainting walk mode to finish!")
            while(self.walkStatus != FREE):
                pass
            rospy.loginfo("Now you will able to control the robot using the joystick!")
            self.awwChangePosture.goToPosition(aww_change_posture.HOME_POSITION)
            self.allow_control = True
        else:
            rospy.loginfo("Mode wheleed disabled! The joystick commands for control the robot are disabled...")
            self.allow_control = False

    def handleLegControlStatus(self, data):
        self.walkStatus = data.data

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
        self.awwChangePosture.goToPosition(aww_change_posture.HOME_POSITION)
        while not rospy.is_shutdown():
            vel = Twist()
            if self.allow_control:
                vel.linear.x = self.linear
                vel.angular.z = self.rotation
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            self.pub.publish(vel)
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    raw_input(formatString("[WHEEL] Welcome to aww wheel control and teleop node! Press Enter to start node:"))
    awwTeleop = AwwTeleop()
    awwTeleop.run()