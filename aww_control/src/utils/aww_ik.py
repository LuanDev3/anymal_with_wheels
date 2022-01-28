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

INTERNAL_ELBOW = 'INTERNAL_ELBOW'
INTERNAL_KNEE = 'INTERNAL_KNEE'

class ImpossibleTrajectory(Exception):
	def __init__(self, msg='The proposed trajectory is impossible to achieve!'):
		super(ImpossibleTrajectory, self).__init__(msg)

class AwwLegIKResolver:
	def __init__(self, solution=INTERNAL_ELBOW):
		self.solution = solution if solution in (INTERNAL_ELBOW, INTERNAL_KNEE) else INTERNAL_ELBOW

	def calculatePathInJointSpace(self, path_in_cartesian_space, static_joints, prefix, time_to_complete_trajectory):
		robot_trajectory = RobotTrajectory()
		joint_trajectory = JointTrajectory()
		number_of_points = len(path_in_cartesian_space)

		rospy.loginfo("Creating path with %s points in joint space" % number_of_points)
		dt = time_to_complete_trajectory*(1.0/number_of_points)
		for i, point in enumerate(path_in_cartesian_space):
			final_point = JointTrajectoryPoint()
			p_x, p_z = point.position.x, point.position.z
			phi, eta = self.getPhi(p_x, p_z)
			theta = self.getTheta(p_x, p_z, eta)
			final_point.positions = [static_joints[0], theta, phi, static_joints[1]]
			final_point.velocities = []
			final_point.accelerations = []
			final_point.effort = []
			final_point.time_from_start = rospy.Duration.from_sec((i+1)*dt)
			joint_trajectory.points.append(final_point)

		rospy.loginfo("Path created with %s points!" % len(joint_trajectory.points))
		joint_trajectory.joint_names = [joint.replace('$', prefix) for joint in ["$_HAA", "$_HFE", "$_KFE"]]
		joint_trajectory.header.frame_id = "base"
		robot_trajectory.joint_trajectory = joint_trajectory
		return robot_trajectory


	def getPhi(self, px, pz):
		print px, pz
		cos_eta = (px**2 + pz**2 - THIGH_LENGTH**2 - KNEE_OFFSET**2 - LEG_LENGTH**2)/(2*THIGH_LENGTH*VIRTUAL_LEG_LENGTH)
		if cos_eta > 1:
			raise ImpossibleTrajectory
		eta_1 = math.atan2(math.sqrt(1 - cos_eta**2), cos_eta)
		eta_2 = math.atan2(-math.sqrt(1 - cos_eta**2), cos_eta)
		phi_1 = eta_1 - PSI if eta_1 > 0 else eta_1 + PSI
		phi_2 = eta_2 - PSI if eta_2 > 0 else eta_2 + PSI
		if self.solution == INTERNAL_ELBOW:
			return min(phi_1, phi_2), min(eta_1, eta_2)
		else:
			return max(phi_1, phi_2), max(eta_1, eta_2)

	def getTheta(self, px, pz, eta):
		external_angle = -math.atan((VIRTUAL_LEG_LENGTH*math.sin(eta))/(THIGH_LENGTH + VIRTUAL_LEG_LENGTH*math.cos(eta)))
		internal_angle = math.atan(px/pz) if px != 0 else math.radians(90)
		theta = external_angle + internal_angle
		return theta
