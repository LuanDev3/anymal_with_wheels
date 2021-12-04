import math
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose

# In meters
HIP_OFFSET = 0.1003
THIGH_LENGTH = 0.285
KNEE_OFFSET = 0.102
LEG_LENGTH = 0.33797
VIRTUAL_LEG_LENGTH = math.sqrt(LEG_LENGTH**2 + KNEE_OFFSET**2)
PSI = math.atan(KNEE_OFFSET/LEG_LENGTH)

DOG_MODE = 'DOG_MODE'
HORSE_MODE = 'HORSE_MODE'

class ImpossibleTrajectory(Exception):
	def __init__(self, msg='The proposed trajectory is impossible to achieve!'):
		super(ImpossibleTrajectory, self).__init__(msg)

class AwwLegIKResolver:
	def __init__(self, solution=DOG_MODE):
		self.solution = solution if solution in (DOG_MODE, HORSE_MODE) else DOG_MODE

	def calculatePathInJointSpace(self, path_in_cartesian_space, time_to_complete_trajectory):
		robot_trajectory = RobotTrajectory()
		joint_trajectory = JointTrajectory()
		number_of_points = len(path_in_cartesian_space)

		rospy.loginfo("Creating path with %s points in joint space" % number_of_points)
		dt = time_to_complete_trajectory*(1.0/number_of_points)
		for i, point in enumerate(path_in_cartesian_space):
			final_point = JointTrajectoryPoint()
			p_x, p_z = point.position.x, point.position.z
			phi = self.getPhi(p_x, p_z)
			theta = self.getTheta(p_x, p_z, phi)
			final_point.positions = [1, theta, phi, 1]
			final_point.velocities = []
			final_point.accelerations = []
			final_point.effort = []
			final_point.time_from_start = rospy.Duration.from_sec((i+1)*dt)
			joint_trajectory.points.append(final_point)

		rospy.loginfo("Path created with %s points!" % len(joint_trajectory.points))
		joint_trajectory.joint_names = ["RF_HAA", "RF_HFE", "RF_KFE", "RF_wheel"]
		joint_trajectory.header.frame_id = "base"
		robot_trajectory.joint_trajectory = joint_trajectory
		return robot_trajectory


	def getPhi(self, px, pz):
		cos_eta = (px**2 + pz**2 - THIGH_LENGTH**2 - KNEE_OFFSET**2 - LEG_LENGTH**2)/(2*THIGH_LENGTH*VIRTUAL_LEG_LENGTH)
		if cos_eta > 1:
			# print "EITAAAAAA %f" % cos_eta
			# print "px %f" % px
			# print "pz %f" % pz
			# print "THIGH_LENGTH**2 %f" % (THIGH_LENGTH**2)
			# print "KNEE_OFFSET**2 %f" % (KNEE_OFFSET**2)
			# print "LEG_LENGTH**2 %f" % (LEG_LENGTH**2)
			# print "2THIGH_LENGTH*VIRTUAL_LEG_LENGTH %f" % (2*THIGH_LENGTH*VIRTUAL_LEG_LENGTH)
			# cos_eta = 1
			raise ImpossibleTrajectory
		eta_1 = math.atan2(cos_eta, math.sqrt(1 - cos_eta**2))
		eta_2 = math.atan2(cos_eta, math.sqrt(1 + cos_eta**2))
		phi_1 = eta_1 - PSI if eta_1 > 0 else eta_1 + PSI
		phi_2 = eta_2 - PSI if eta_2 > 0 else eta_2 + PSI
		if self.solution == DOG_MODE:
			return min(phi_1, phi_2)
		else:
			return max(phi_1, phi_2)

	def getTheta(self, px, pz, phi):
		external_angle = math.atan(pz/px) if px != 0 else math.radians(90)
		internal_angle = math.atan((VIRTUAL_LEG_LENGTH*math.sin(phi))/(THIGH_LENGTH + VIRTUAL_LEG_LENGTH*math.cos(phi)))
		theta = external_angle - internal_angle
		return theta
