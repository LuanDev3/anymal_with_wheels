#!/usr/bin/env python2

import rospy
import rospkg
import actionlib
import time
import tf
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


WHEELED = "wheeled"
LEGGED = "legged"

class Color:
    YELLOW = '\033[93m'
    END = '\033[0m'


def formatString(initialString, color=Color.YELLOW):
    return color + initialString + Color.END


class Navigation():
    def __init__(self):
        rospy.init_node('aww_navigate_example', anonymous=True, log_level=getattr(rospy, 'DEBUG'))

        rospy.loginfo("Initing node!")

        # Publish
        self.pub_mode = rospy.Publisher('/aww/mode', String, queue_size=1)

        # move base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # tf
        self.listener = tf.TransformListener()


    def send_goal_and_wait(self, x, y, z, w):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def publish_mode(self, mode):
        self.pub_mode.publish(mode)


    def navigate(self):
        rospy.loginfo("Starting navigation!")
        self.publish_mode(WHEELED)
        time.sleep(10)
        self.send_goal_and_wait(24.296, 1.62, 0.911, -0.410)
        self.publish_mode(LEGGED)
        time.sleep(5)
        overcame_obstacle = False
        while (not overcame_obstacle):
            time.sleep(2)
            (position, _) = self.listener.lookupTransform("map", "base_footprint", rospy.Time())
            if (position[0] < 22.6331253052 or position[1] < -0.5966):
                overcame_obstacle = True
        self.publish_mode(WHEELED)
        self.send_goal_and_wait(16.456, -3.515, 0.994, -0.107)

            
if __name__ == '__main__':
    raw_input(formatString("[WHEEL] Welcome to the navigation example! Click on start to proceed:"))
    nav = Navigation()
    nav.navigate()