#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

from __future__ import division

import sys
import copy
import math
import rospy
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import threading
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def itsCloseEnough(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
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


class MoveGroupInteface(object):

  def __init__(self):

    rospy.loginfo("Starting move_groups node...")
    super(MoveGroupInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True, log_level=getattr(rospy, 'DEBUG'))

    rospy.logdebug(" -- Getting robot comamder")
    self.robot = moveit_commander.RobotCommander()
    rospy.logdebug(" -- Getting robot scene")
    self.scene = moveit_commander.PlanningSceneInterface()

    rospy.logdebug(" -- Getting robot move groups")
    self.frontLeftMoveGroup  = moveit_commander.MoveGroupCommander("front_left_leg")
    self.frontRightMoveGroup = moveit_commander.MoveGroupCommander("front_right_leg")
    self.rearLeftMoveGroup   = moveit_commander.MoveGroupCommander("rear_left_leg")
    self.rearRightMoveGroup  = moveit_commander.MoveGroupCommander("rear_right_leg")
    self.allLegsMoveGroup    = moveit_commander.MoveGroupCommander("all_legs")

    rospy.logdebug(" -- Setting robot publishers")
    self.frontLeftTrajectoryPublishers  = rospy.Publisher('/move_group/front_left_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    self.frontRightTrajectoryPublishers = rospy.Publisher('/move_group/front_right_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    self.rearLeftTrajectoryPublishers   = rospy.Publisher('/move_group/rear_left_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    self.rearfrontTrajectoryPublishers  = rospy.Publisher('/move_group/rear_right_leg/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    self.allLegsTrajectoryPublishers    = rospy.Publisher('/move_group/all_legs/display_planned_path',  moveit_msgs.msg.DisplayTrajectory, queue_size=20)

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

  def goToInitialPosition(self):

    rospy.loginfo("Sending robot to initial position...")
    allLegsJointGoal = [0.0,  0.7, -0.94, 0.0, #front_left
                        0.0, -0.7,  0.94, 0.0, #rear_left
                        0.0,  0.7, -0.94, 0.0, #front_right
                        0.0, -0.7,  0.94, 0.0] #rear_right

    rospy.logdebug(" -- Sending Go action and waiting ...")
    self.allLegsMoveGroup.go(allLegsJointGoal, wait=True)
    rospy.logdebug(" -- Go action finished, stop for warranty")
    self.allLegsMoveGroup.stop()

    rospy.logdebug(" -- Verifying if the joints are close enough to the desired")
    currentJoints = self.allLegsMoveGroup.get_current_joint_values()
    resolution = 0.01
    if itsCloseEnough(allLegsJointGoal, currentJoints, resolution):
        rospy.loginfo("The joints were moved to te position with a error less tahn %s" % (resolution))
    else:
        rospy.logwarn("The joints weren't moved to te position with a error less tahn %s" % (resolution))


  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def planCartesianPath(self, moveGroup, hDisplacement = 0.3, vDisplacement = 0.1, numberOfPoints = 10, scale=1):
    def createParabolicPath(hDisplacement = 0.2, vDisplacement = 0.1, numberOfPoints=10):
        x = [float(num) for num in numpy.linspace(0.0, hDisplacement, numberOfPoints)]
        gain = -vDisplacement/((hDisplacement/2)**2)
        translation = -hDisplacement/2
        y = [gain * (element + translation) ** 2 + vDisplacement for element in x]
        return x, y

    moveGroup = getattr(self, moveGroup)
    xValues, zValues = createParabolicPath(hDisplacement, vDisplacement, numberOfPoints)
    wpose = moveGroup.get_current_pose().pose
    xInitial = wpose.position.x
    zInitial = wpose.position.z

    waypoints = list()
    for xPoint, zPoint in zip(xValues, zValues):
        wpose.position.x = xInitial + xPoint 
        wpose.position.z = zInitial + zPoint
        waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = moveGroup.compute_cartesian_path(waypoints, 0.01, 0.0)
    return plan, fraction


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def executePlan(self, plan, moveGroup):

    moveGroup = getattr(self, moveGroup)
    moveGroup.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


def main():
  try:
    interface = MoveGroupInteface()
    result = interface.goToInitialPosition()
    cartesian_plan, fraction = interface.planCartesianPath('frontRightMoveGroup')
    interface.executePlan(cartesian_plan, 'frontRightMoveGroup')
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()