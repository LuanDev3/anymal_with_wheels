#!/usr/bin/env python
import rospy
import tf

WHEEL_RADIUS = 0.098

if __name__ == "__main__":
	rospy.init_node("publish_base_footprint")
	br = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		try:
			(w1, _) = listener.lookupTransform("base", "LH_wheel_link", rospy.Time())
			(w2, _) = listener.lookupTransform("base", "LF_wheel_link", rospy.Time())
			(w3, _) = listener.lookupTransform("base", "RH_wheel_link", rospy.Time())
			(w4, _) = listener.lookupTransform("base", "RF_wheel_link", rospy.Time())
			z_vector = sorted([abs(w1[2]), abs(w2[2]), abs(w3[2]), abs(w4[2])])
			z_vector = [z_vector[0], z_vector[1]]
			height = float(sum(z_vector))/2
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			height = 0
		br.sendTransform((0.0, 0.0, -(WHEEL_RADIUS + height)),
						 (0.0, 0.0, 0.0, 1),
						 rospy.Time.now(),
						 "base_footprint",
						 "base")
		rate.sleep()