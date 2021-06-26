#!/usr/bin/env python2

from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import rospkg
import rospy
import yaml
import time

class AwwDriverSim():

    ''' Class to simulate aww motors '''

    def __init__(self):
        rospy.init_node('aww_driver_sim', anonymous=False)
        rospy.loginfo("Initializing simulated driver")
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('aww_driver')
        self.joint_config = rospy.get_param('~joint_config', self.path + '/config/joint_parameters.yml')
        self.home = rospy.get_param('~home', self.path + '/config/home.yml')
        self.actualPosition = dict()
        self.positions = dict()
        self.limits = dict()
        self.wheeled_pos = dict()
        self.delta = dict()
        self.initPos = dict()
        self.iniTime = rospy.Time.now()
        with open(self.joint_config) as file:
            joint_dict = yaml.safe_load(file)
            for joint, content in joint_dict.items():

                # Set initial position to home
                pos = self.__homePosition(joint)
                self.actualPosition[joint] = pos
                self.positions[joint] = pos
                self.initPos[joint] = pos

                # Get min and max angles in radians
                self.limits[joint] = content.get('limits', {'effort': .0,
                                                            'lower': .0,
                                                            'upper': .0, 
                                                            'velocity': .0})

                # Get wheeled position
                self.wheeled_pos[joint] = content.get('wheeled_position', 0)


                # Debug info
                console_info = ("Joint %s \t started with min angle=%f, " % (joint, self.limits[joint]['lower']) +
                    "max angle=%f and initial pos=%f" % (self.limits[joint]['upper'], pos))
                rospy.loginfo(console_info)

        # Subscribers
        rospy.Subscriber('/aww/joint_commands', JointState, self.jointCallback)
        self.pubJointPosition = rospy.Publisher('/aww/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)


    def jointCallback(self, data):
        rospy.logdebug('Position command received')
        for i, name in enumerate(data.name):
            self.positions[name] = self.__clamp(name, data.position[i], self.limits[name]['lower'], self.limits[name]['upper'])
            self.delta[name] = (self.positions[name] - self.actualPosition[name])
            self.initPos[name] = self.actualPosition[name]
        self.iniTime = int(round(time.time() * 1000))

    def run(self):
        rospy.loginfo("Initializing control")
        while not rospy.is_shutdown():
            self.state = JointState()
            for name in self.actualPosition:
                if (abs(self.positions[name] - self.actualPosition[name]) > 0.01):
                    #TODO
                    # I need to add a dynamic here! Somwthing like this:
                    # timeV = abs(int(round(time.time() * 1000)) - self.iniTime)
                    # self.actualPosition[name] = self.initPos[name] + (1/(1 + np.exp(-4*(timeV/self.t3[name] - 1))))*abs(self.positions[name] - self.initPos[name])
                    self.actualPosition[name] = self.positions[name]
                self.state.name.append(name)
                self.state.position.append(self.actualPosition[name])
            self.state.header.stamp = rospy.Time.now()
            self.pubJointPosition.publish(self.state)
            self.rate.sleep()

    # "Private" functions

    # Clap function
    def __clamp(self, name, n, minn, maxn):
        value = max(min(maxn, n), minn)
        if (n < minn) or (n > maxn):
            rospy.logwarn("joint %s received a position value out of bounds." % (name) +
                    " Clipping the value to %s." % round(value, 3))
        return value

    # Home position function
    def __homePosition(self, jointName):
        try:
            with open(self.home) as file:
                documents = yaml.safe_load(file)
                for item, doc in documents.items():
                    if item == jointName:
                        return doc
                rospy.logwarn("Home position for joint %s not defined." % (jointName) +
                       " Setting position to 0 rad.")
                return 0
        except IOError:
            rospy.logwarn("file %s don't exist, setting position to 0 rad." % (self.home))
            return 0

if __name__ == '__main__':
    awwDriverSim = AwwDriverSim()
    awwDriverSim.run()