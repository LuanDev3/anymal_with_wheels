import math
from moveit_msgs.msg import RobotTrajectory
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

class AwwLegIKResolver:
	def __init__(solution):
		self.solution = solution if solution in (DOG_MODE, HORSE_MODE) else DOG_MODE

	def calculatePathInJointSpace(path_in_cartesian_space):
		for point in path_in_cartesian_space:
			p_x, p_z = point.position.x, point.position.z
			phi = getPhi(p_x, p_z)
			theta = getTheta(p_x, p_z)

	def getPhi(px, pz):
		cos_eta = (px**2 + pz**2 - THIGH_LENGTH**2 - KNEE_OFFSET**2 - LEG_LENGTH**2)/(2*THIGH_LENGTH*VIRTUAL_LEG_LENGTH)
		eta_1 = math.atan2(cos_eta, math.sqrt(1 - cos_eta**2))
		eta_2 = math.atan2(cos_eta, math.sqrt(1 + cos_eta**2))
		phi_1 = eta_1 - PSI if eta_1 > 0 else eta_1 + PSI
		phi_2 = eta_2 - PSI if eta_2 > 0 else eta_2 + PSI
		if self.solution == DOG_MODE:
			return min(phi_1, phi_2)
		else:
			return max(phi_1, phi_2)

	def getTheta(px, pz):
		phi = getPhi(px, pz)
		external_angle = math.atan(pz/px) if px != 0 else math.radians(90)
		internal_angle = math.atan((VIRTUAL_LEG_LENGTH*math.sin(phi))/(THIGH_LENGTH + VIRTUAL_LEG_LENGTH*math.cos(phi)))
		theta = external_angle - internal_angle
		return theta
