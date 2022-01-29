#!/usr/bin/env python
import copy
import rospy
import numpy
import actionlib
import moveit_commander
import moveit_msgs.msg
import time
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from utils import aww_ik, aww_change_posture


WHEELED = "wheeled"
LEGGED  = "legged"
WALKING = "walking"
FREE  = "free"

moveGroupPrefixes = {
    "frontLeftMoveGroup" : "LF",
    "frontRightMoveGroup" : "RF",
    "rearLeftMoveGroup" :  "LH",
    "rearRightMoveGroup" : "RH"
}

itsCloseEnough = aww_change_posture.itsCloseEnough
removeWheelsForArray = aww_change_posture.removeWheelsForArray


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
    BLUE = '\033[34m'
    END = '\033[0m'


def formatString(initialString, color=Color.YELLOW):
    return color + initialString + Color.END


def findNameForMovegroup(moveGroupName):
    legName = moveGroupName.replace("MoveGroup", "")
    return " l".join(legName.split("L")) if "L" in legName else " r".join(legName.split("R"))


def createParabolicPath(hDisplacement = 0.2, vDisplacement = 0.1, numberOfPoints=10):
    x = [float(num) for num in numpy.linspace(0.0, hDisplacement, numberOfPoints)]
    gain = -vDisplacement/((hDisplacement/2)**2)
    translation = -hDisplacement/2
    y = [gain * (element + translation) ** 2 + vDisplacement for element in x]
    return x, y


def createInverseParabolicPath(hDisplacement = 0.2, vDisplacement = 0.1, numberOfPoints=10):
    x, y = createParabolicPath(hDisplacement=hDisplacement, vDisplacement=vDisplacement, numberOfPoints=numberOfPoints)
    return [-element for element in x], y


def createLinearPath(hDisplacement = 0.2, vDisplacement = None, numberOfPoints=10):
    x = [float(num) for num in numpy.linspace(0.0, hDisplacement, numberOfPoints)]
    y = [0.0 for element in x]
    return x, y


def createInverseLinearPath(hDisplacement = 0.2, vDisplacement = None, numberOfPoints=10):
    x, y = createLinearPath(hDisplacement=hDisplacement, vDisplacement=vDisplacement, numberOfPoints=numberOfPoints)
    return [-element for element in x], y


class MoveGroupInteface(object):

    def __init__(self):
        rospy.loginfo("Starting move_groups node...")
        joint_state_topic = ['joint_states:=/aww/joint_states']
        super(MoveGroupInteface, self).__init__()
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move_group_interface', anonymous=True, log_level=getattr(rospy, 'DEBUG'))

        rospy.logdebug(" -- Getting robot comamder")
        self.robot = moveit_commander.RobotCommander("aww/robot_description")

        rospy.logdebug(" -- Starting aww inverse kinematic leg resolver")
        self.awwLegIkResolver = aww_ik.AwwLegIKResolver()

        self.createPath = {
            'PARABOLIC': createParabolicPath,
            'INV-PARABOLIC': createInverseParabolicPath,
            'LINEAR': createLinearPath,
            'INV-LINEAR': createInverseLinearPath
        }

        rospy.logdebug(" -- Getting action clients")
        self.frontLeftLegAction  = actionlib.SimpleActionClient('/aww/front_left_leg_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.frontRightLegAction = actionlib.SimpleActionClient('/aww/front_right_leg_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.rearLeftLegAction   = actionlib.SimpleActionClient('/aww/rear_left_leg_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.rearRightLegAction  = actionlib.SimpleActionClient('/aww/rear_right_leg_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.frontLeftLegAction.wait_for_server()
        self.frontRightLegAction.wait_for_server()
        self.rearLeftLegAction.wait_for_server()
        self.rearRightLegAction.wait_for_server()

        rospy.logdebug(" -- Getting robot move groups")
        flag = True
        self.allLegsMoveGroup = None
        self.frontLeftMoveGroup = None
        self.frontRightMoveGroup = None
        self.rearLeftMoveGroup = None
        self.rearRightMoveGroup = None
        while(flag):
            try:
                self.allLegsMoveGroup    = self.allLegsMoveGroup    if self.allLegsMoveGroup else moveit_commander.MoveGroupCommander("all_legs", "aww/robot_description")
                self.frontLeftMoveGroup  = self.frontLeftMoveGroup  if self.frontLeftMoveGroup else moveit_commander.MoveGroupCommander("front_left_leg", "aww/robot_description")
                self.frontRightMoveGroup = self.frontRightMoveGroup if self.frontRightMoveGroup else moveit_commander.MoveGroupCommander("front_right_leg", "aww/robot_description")
                self.rearLeftMoveGroup   = self.rearLeftMoveGroup   if self.rearLeftMoveGroup else moveit_commander.MoveGroupCommander("rear_left_leg", "aww/robot_description")
                self.rearRightMoveGroup  = self.rearRightMoveGroup  if self.rearRightMoveGroup else moveit_commander.MoveGroupCommander("rear_right_leg", "aww/robot_description")
                flag = False
            except Exception:
                flag = True
        self.awwChangePosture    = aww_change_posture.AwwChangePosture(self.allLegsMoveGroup)

        rospy.logdebug(" -- Setting robot publisher")
        self.walkStatus  = rospy.Publisher('/aww/leg_control_status', String, queue_size=1)

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


    def normalizeTranslationEEPose(self, pose, refFrame):
        refLink = self.robot.get_link(refFrame)
        x = pose.position.x - refLink.pose().pose.position.x
        z = pose.position.z - refLink.pose().pose.position.z
        return x, z


    def planCartesianPath(self, moveGroup, type, hDisplacement = 0.3, vDisplacement = 0.1, numberOfPoints = 20, scale=1):

        rospy.loginfo("Creating plan for %s leg!" % findNameForMovegroup(moveGroup))
        prefix = moveGroupPrefixes[moveGroup]
        moveGroup = getattr(self, moveGroup)
        rospy.logdebug(" -- Creating trajectory...")
        xValues, zValues = self.createPath[type](hDisplacement, vDisplacement, numberOfPoints)
        wpose = moveGroup.get_current_pose().pose
        normPose = self.normalizeTranslationEEPose(wpose, "$_thigh_fixed".replace("$", prefix))
        wpose.position.x = normPose[0]
        wpose.position.z = normPose[1]
        xInitial = wpose.position.x
        zInitial = wpose.position.z

        waypoints = list()
        for xPoint, zPoint in zip(xValues, zValues):
            wpose.position.x = xInitial + xPoint 
            wpose.position.z = zInitial + zPoint
            waypoints.append(copy.deepcopy(wpose))

        rospy.logdebug(" -- Computing path...")
        static_joints = [self.robot.get_joint("$_HAA".replace("$", prefix)).value(), 
                         self.robot.get_joint("$_wheel".replace("$", prefix)).value()]
        path = self.awwLegIkResolver.calculatePathInJointSpace(waypoints, static_joints, prefix, 1)
        return path
        

    def executePlan(self, plan, moveGroup, wait=True):
        rospy.loginfo("Executing plan for %s leg!" % findNameForMovegroup(moveGroup))
        moveGroup = getattr(self, moveGroup)
        moveGroup.execute(plan, wait=wait)
        rospy.loginfo("Plan executed!")

    
    def discontinuousGateStartPosition(self, displacement = 0.25):
        rospy.loginfo(formatString("Moving robot to initial position to start gate!", Color.BLUE))
        self.awwLegIkResolver.solution = aww_ik.INTERNAL_ELBOW
        plan = self.planCartesianPath("frontRightMoveGroup", 'INV-PARABOLIC', displacement)
        self.executePlan(plan, "frontRightMoveGroup")
        self.awwLegIkResolver.solution = aww_ik.INTERNAL_KNEE
        plan = self.planCartesianPath("rearRightMoveGroup", 'PARABOLIC', displacement)
        self.executePlan(plan, "rearRightMoveGroup")
        time.sleep(0.5)


    def moveBodyToFront(self, displacement = 0.25):
        rospy.loginfo(formatString("Sending body to front!", Color.BLUE))
        self.awwLegIkResolver.solution = aww_ik.INTERNAL_ELBOW
        plan_1 = self.planCartesianPath("frontLeftMoveGroup", 'INV-LINEAR', displacement)
        plan_2 = self.planCartesianPath("frontRightMoveGroup", 'INV-LINEAR', displacement)
        self.awwLegIkResolver.solution = aww_ik.INTERNAL_KNEE
        plan_3 = self.planCartesianPath("rearLeftMoveGroup", 'INV-LINEAR', displacement)
        plan_4 = self.planCartesianPath("rearRightMoveGroup", 'INV-LINEAR', displacement)
        goal1 = FollowJointTrajectoryGoal()     
        goal1.trajectory = plan_1.joint_trajectory
        goal2 = FollowJointTrajectoryGoal()                  
        goal2.trajectory = plan_2.joint_trajectory
        goal3 = FollowJointTrajectoryGoal()                  
        goal3.trajectory = plan_3.joint_trajectory
        goal4 = FollowJointTrajectoryGoal()                  
        goal4.trajectory = plan_4.joint_trajectory
        self.frontLeftLegAction.send_goal(goal1)
        self.frontRightLegAction.send_goal(goal2)
        self.rearLeftLegAction.send_goal(goal3)
        self.rearRightLegAction.send_goal(goal4)
        self.frontLeftLegAction.wait_for_result()
        self.frontRightLegAction.wait_for_result()
        self.rearLeftLegAction.wait_for_result()
        self.rearRightLegAction.wait_for_result()
        time.sleep(0.5)


    def moveLeg(self, group, type, displacement=(0.5, 0.1)):
        rospy.loginfo(formatString("Moving leg : %s!", Color.BLUE) %findNameForMovegroup(group))
        if "front" in group:
            self.awwLegIkResolver.solution = aww_ik.INTERNAL_ELBOW
        else:
            self.awwLegIkResolver.solution = aww_ik.INTERNAL_KNEE
        plan = self.planCartesianPath(group, type, hDisplacement=displacement[0], vDisplacement=displacement[1])
        self.executePlan(plan, group)
        time.sleep(0.5)


    def controlManager(self):
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if not(self.lastMode == self.mode):
                rospy.loginfo(formatString("Mode changed to: %s") %self.mode.getMode())
                self.lastMode.wheeled = self.mode.wheeled
                self.lastMode.legged  = self.mode.legged
                if self.mode.legged:
                    self.walkStatus.publish(WALKING)
                    self.awwChangePosture.goToPosition(aww_change_posture.WALK_POSITION)
                    self.discontinuousGateStartPosition()
                    while(True):
                        self.moveLeg("frontRightMoveGroup", 'PARABOLIC')
                        if self.mode.wheeled:
                            break
                        self.moveBodyToFront()
                        if self.mode.wheeled:
                            break
                        self.moveLeg("rearLeftMoveGroup", 'PARABOLIC')
                        if self.mode.wheeled:
                            break
                        self.moveLeg("frontLeftMoveGroup", 'PARABOLIC')
                        if self.mode.wheeled:
                            break
                        self.moveBodyToFront()
                        if self.mode.wheeled:
                            break
                        self.moveLeg("rearRightMoveGroup", 'PARABOLIC')
                        if self.mode.wheeled:
                            break
                    self.walkStatus.publish(FREE)
            rate.sleep()


def main():

    raw_input(formatString("[LEG] Welcome to aww leg control and teleop node! Press Enter to start node:"))
  
    try:
        interface = MoveGroupInteface()
        interface.awwChangePosture.goToPosition(aww_change_posture.HOME_POSITION)
        interface.controlManager()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()