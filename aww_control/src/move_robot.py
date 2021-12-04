#!/usr/bin/env python

import sys
import copy
import math
import rospy
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import threading
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from utils import aww_ik


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

class Mode:

    def __init__(self, wheeled, legged):
        self.wheeled = wheeled
        self.legged = legged

    def __eq__(self, other):
        if isinstance(other, Mode):
            if self.wheeled == other.wheeled and self.legged == other.legged:
                return True
        return False

    def getMode(self):
        return WHEELED if self.wheeled else LEGGED

class Color:
    YELLOW = '\033[93m'
    END = '\033[0m'


def formatString(initialString, color=Color.YELLOW):
    return color + initialString + Color.END


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

def findNameForMovegroup(moveGroupName):
    legName = moveGroupName.replace("MoveGroup", "")
    return " l".join(legName.split("L")) if "L" in legName else " r".join(legName.split("R"))


class MoveGroupInteface(object):

    def __init__(self):
        rospy.loginfo("Starting move_groups node...")
        joint_state_topic = ['joint_states:=/aww/joint_states']
        super(MoveGroupInteface, self).__init__()
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move_group_interface', anonymous=True, log_level=getattr(rospy, 'DEBUG'))

        rospy.logdebug(" -- Getting robot comamder")
        self.robot = moveit_commander.RobotCommander("aww/robot_description")
        a = self.robot.get_link("LF_thigh_fixed")
        b = self.robot.get_link("LF_LEG")
        print a.pose()
        print b.pose()
        rospy.logdebug(" -- Getting robot scene")
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.logdebug("-- Starting aww inverse kinematic leg resolver")
        self.awwLegIkResolver = aww_ik.AwwLegIKResolver()

        rospy.logdebug(" -- Getting robot move groups")
        self.frontLeftMoveGroup  = moveit_commander.MoveGroupCommander("front_left_leg", "aww/robot_description")
        self.frontRightMoveGroup = moveit_commander.MoveGroupCommander("front_right_leg", "aww/robot_description")
        self.rearLeftMoveGroup   = moveit_commander.MoveGroupCommander("rear_left_leg", "aww/robot_description")
        self.rearRightMoveGroup  = moveit_commander.MoveGroupCommander("rear_right_leg", "aww/robot_description")
        self.allLegsMoveGroup    = moveit_commander.MoveGroupCommander("all_legs", "aww/robot_description")

        rospy.logdebug(" -- Setting robot publishers")
        self.frontLeftTrajectoryPublishers  = rospy.Publisher('/move_group/front_left_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.frontRightTrajectoryPublishers = rospy.Publisher('/move_group/front_right_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.rearLeftTrajectoryPublishers   = rospy.Publisher('/move_group/rear_left_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.rearfrontTrajectoryPublishers  = rospy.Publisher('/move_group/rear_right_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.allLegsTrajectoryPublishers    = rospy.Publisher('/move_group/all_legs/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        rospy.logdebug("-- Setting robot subscribes")
        rospy.Subscriber("/aww/mode", String, self.handleRobotMode)
        self.mode = Mode(wheeled=True, legged=False)
        self.lastMode = Mode(wheeled=True, legged=False)

        rospy.logdebug(" -- Getting move groups end effector")
        self.frontLeftEefLink  = self.frontLeftMoveGroup.get_end_effector_link()
        self.frontRisghEefLink = self.frontRightMoveGroup.get_end_effector_link()
        self.rearLeftEefLink   = self.rearLeftMoveGroup.get_end_effector_link()
        self.rearRightEefLink  = self.rearRightMoveGroup.get_end_effector_link()
        self.allLegsEefLink    = self.allLegsMoveGroup.get_end_effector_link()

        rospy.loginfo("The end effectors configurated are:")
        rospy.loginfo("front_left_leg : %s" % self.frontLeftEefLink)
        rospy.loginfo("front_right_leg: %s" % self.frontRisghEefLink)
        rospy.loginfo("rear_left_leg  : %s" % self.rearLeftEefLink)
        rospy.loginfo("rear_right_leg : %s" % self.rearRightEefLink)
        rospy.loginfo("all_legs : %s"  % "-")


    def handleRobotMode(self, data):
        self.mode.wheeled = True if data.data == WHEELED else False
        self.mode.legged = not(self.mode.wheeled)


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


    def planCartesianPath(self, moveGroup, type, hDisplacement = 0.3, vDisplacement = 0.1, numberOfPoints = 20, scale=1):
        def createParabolicPath(hDisplacement = 0.2, vDisplacement = 0.1, numberOfPoints=10):
            x = [float(num) for num in numpy.linspace(0.0, hDisplacement, numberOfPoints)]
            gain = -vDisplacement/((hDisplacement/2)**2)
            translation = -hDisplacement/2
            y = [gain * (element + translation) ** 2 + vDisplacement for element in x]
            return x, y

        def createLinearPath(hDisplacement = 0.2, vDisplacement = None, numberOfPoints=10):
            x = [float(num) for num in numpy.linspace(0.0, hDisplacement, numberOfPoints)]
            y = [0.0 for element in x]
            return x, y

        def createInverseLinearPath(hDisplacement = 0.2, vDisplacement = None, numberOfPoints=10):
            x, y = createLinearPath(hDisplacement=hDisplacement, vDisplacement=vDisplacement, numberOfPoints=numberOfPoints)
            return [-element for element in x], y

        createPath = {
            'PARABOLIC': createParabolicPath,
            'LINEAR': createLinearPath,
            'INV-LINEAR': createInverseLinearPath
        }


        rospy.loginfo("Creating plan for %s leg!" % findNameForMovegroup(moveGroup))
        moveGroup = getattr(self, moveGroup)
        rospy.logdebug(" -- Creating trajectory...")
        xValues, zValues = createPath[type](hDisplacement, vDisplacement, numberOfPoints)
        wpose = moveGroup.get_current_pose().pose
        xInitial = wpose.position.x
        zInitial = wpose.position.z

        waypoints = list()
        for xPoint, zPoint in zip(xValues, zValues):
            wpose.position.x = xInitial + xPoint 
            wpose.position.z = zInitial + zPoint
            waypoints.append(copy.deepcopy(wpose))

        rospy.logdebug(" -- Computing path...")
        (plan, fraction) = moveGroup.compute_cartesian_path(waypoints[1:], 0.001, 0, avoid_collisions=False)
        test = self.awwLegIkResolver.calculatePathInJointSpace(waypoints, 3)
        print test
        return plan, fraction

    def moveBodyToFront(self, vDisplacement = 0.1, numberOfPoints = 10, scale=1):
        rospy.loginfo("Moving base forward!")
        rospy.logdebug(" -- Getting the actual pose for all move groups...")
        fl_pose = self.frontLeftMoveGroup.get_current_pose().pose
        fr_pose = self.frontRightMoveGroup.get_current_pose().pose
        rl_pose = self.rearLeftMoveGroup.get_current_pose().pose
        rr_pose = self.rearRightMoveGroup.get_current_pose().pose

        # The path for each move group is to move backward the back and by vDisplacement
        rospy.logdebug(" -- Calculating the linear path ...")
        norm_factor = 10/vDisplacement
        points = [x/norm_factor for x in range(1, int(norm_factor*vDisplacement) + 1, int(vDisplacement/10*norm_factor))]
        fl_waypoints = list()
        fr_waypoints = list()
        rl_waypoints = list()
        rr_waypoints = list()
        for i in range(numberOfPoints):
            fl_pose.position.x = fl_pose.position.x - points[i];
            fr_pose.position.x = fr_pose.position.x - points[i];
            rl_pose.position.x = rl_pose.position.x - points[i];
            rr_pose.position.x = rr_pose.position.x - points[i];
            fl_waypoints.append(copy.deepcopy(fl_pose))
            fr_waypoints.append(copy.deepcopy(fr_pose))
            rl_waypoints.append(copy.deepcopy(rl_pose))
            rr_waypoints.append(copy.deepcopy(rr_pose))

        rospy.loginfo(" -- Computing the cartesian path for the legs")
        fl_plan, _ = self.frontLeftMoveGroup.compute_cartesian_path(fl_waypoints, 0.01, 0.0)
        fr_plan, _ = self.frontRightMoveGroup.compute_cartesian_path(fr_waypoints, 0.01, 0.0)
        rl_plan, _ = self.rearLeftMoveGroup.compute_cartesian_path(rl_waypoints, 0.01, 0.0)
        rr_plan, _ = self.rearRightMoveGroup.compute_cartesian_path(rr_waypoints, 0.01, 0.0)
        print fl_plan.joint_trajectory, type(fl_plan.joint_trajectory)


    def executePlan(self, plan, moveGroup, wait=True):
        rospy.loginfo("Executing plan for %s leg!" % findNameForMovegroup(moveGroup))
        moveGroup = getattr(self, moveGroup)
        moveGroup.execute(plan, wait=wait)
        rospy.loginfo("Plan executed!")



    def controlManager(self):
        if not(self.lastMode == self.mode):
            rospy.loginfo(formatString("Mode changed to: %s") %self.mode.getMode())
            self.lastMode.wheeled = self.mode.wheeled
            self.lastMode.legged  = self.mode.legged
            if self.mode.wheeled:
                self.goToPosition(HOME_POSITION)
                pass
            else:
                self.goToPosition(WALK_POSITION)
                plan, _ = self.planCartesianPath("frontRightMoveGroup", 'INV-LINEAR', hDisplacement=0.2)
                self.executePlan(plan, "frontRightMoveGroup")
                plan, _ = self.planCartesianPath("rearRightMoveGroup", 'LINEAR', hDisplacement=0.2)
                self.executePlan(plan, "rearRightMoveGroup")
                time.sleep(2)
                plan, _ = self.planCartesianPath("frontRightMoveGroup", 'PARABOLIC', hDisplacement=0.4, vDisplacement=0.1)
                self.executePlan(plan, "frontRightMoveGroup")
                #plan, _ = self.planCartesianPath("rearRightMoveGroup", 'PARABOLIC')
                #self.executePlan(plan, "rearRightMoveGroup")

def main():
  try:
    interface = MoveGroupInteface()
    interface.goToPosition(HOME_POSITION)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        interface.controlManager()
        rate.sleep()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()