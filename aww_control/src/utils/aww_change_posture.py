#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list


WHEELED = "wheeled"
LEGGED  = "legged"

HOME_POSITION = {
    "name": "home",
    "points": [0.0,  1.18, -2.12, 0.0, #front_left
               0.0, -1.18,  2.12, 0.0, #rear_left
               0.0,  1.18, -2.12, 0.0, #front_right
               0.0, -1.18,  2.12, 0.0] #rear_right
}

WALK_POSITION = {
    "name": "walk",
    "points": [0.0,  0.7, -1.2, 0.0, #front_left
               0.0, -0.7,  1.2, 0.0, #rear_left
               0.0,  0.7, -1.2, 0.0, #front_right
               0.0, -0.7,  1.2, 0.0] #rear_right
}


def itsCloseEnough(goal, actual, tolerance):
  """ Method imported from move_group.py provided by Moveit!.
    # Copyright (c) 2013, SRI International
    # All rights reserved.
    # Author: Acorn Pooley, Mike Lautman
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


def removeWheelsForArray(array, indexes=[15, 11, 7, 3]):
    newArray = array[:]
    for index in indexes:
        newArray.pop(index)
    return newArray


class AwwChangePosture(object):

    def __init__(self, allLegsMoveGroup=None):
        if not allLegsMoveGroup:
            joint_state_topic = ['joint_states:=/aww/joint_states']
            moveit_commander.roscpp_initialize(joint_state_topic)
            self.allLegsMoveGroup    = moveit_commander.MoveGroupCommander("all_legs", "aww/robot_description")
        else:
            self.allLegsMoveGroup = allLegsMoveGroup


    def goToPosition(self, pose):
        rospy.loginfo("Sending robot to %s position..." % pose.get("name"))
        allLegsJointGoal = pose.get("points")

        rospy.logdebug(" -- Sending Go action and waiting ...")
        self.allLegsMoveGroup.go(allLegsJointGoal, wait=True)
        rospy.logdebug(" -- Go action finished, stop for warranty")
        self.allLegsMoveGroup.stop()

        rospy.logdebug(" -- Verifying if the joints are close enough to the desired")
        currentJoints = self.allLegsMoveGroup.get_current_joint_values()  
        resolution = 0.05
        if itsCloseEnough(removeWheelsForArray(allLegsJointGoal), removeWheelsForArray(currentJoints), resolution):
            rospy.loginfo("The joints were moved to te position with a error less than %s" % (resolution))
        else:
            rospy.logwarn("The joints weren't moved to te position with a error less than %s" % (resolution))
