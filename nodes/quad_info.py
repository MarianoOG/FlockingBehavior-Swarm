#!/usr/bin/env python
import message_filters, rospy
from math import atan2
from geometry_msgs.msg import PoseStamped
from swarm.msg import QuadState, Quad

def callback(pose):
	global pub, quad_state, t

	# Calculate change in time:
	t_prev = t
	t = float(pose.header.stamp.secs + pose.header.stamp.nsecs/1000000000.0)
	delta = t - t_prev

	# Update position:
	pos_prev = Quad()
	pos_prev.x = quad_state.pos.x
	pos_prev.y = quad_state.pos.y
	pos_prev.z = quad_state.pos.z
	quad_state.pos.x = pose.pose.position.x
	quad_state.pos.y = pose.pose.position.y
	quad_state.pos.z = pose.pose.position.z
	
	# Update yaw angle:
	pos_prev.yaw = quad_state.pos.yaw
	ysqr = pose.pose.orientation.y * pose.pose.orientation.y
	q0 = -2.0 * (ysqr + pose.pose.orientation.z * pose.pose.orientation.z) + 1.0
	q1 = 2.0 * (pose.pose.orientation.x * pose.pose.orientation.y - pose.pose.orientation.w * pose.pose.orientation.z)
	quad_state.pos.yaw = - atan2(q1, q0)
	
	# Update velocities:
	quad_state.vel.x = (quad_state.pos.x - pos_prev.x) / delta
	quad_state.vel.y = (quad_state.pos.y - pos_prev.y) / delta
	quad_state.vel.z = (quad_state.pos.z - pos_prev.z) / delta
	quad_state.vel.yaw = (quad_state.pos.yaw - pos_prev.yaw) / delta

	try:
		quad_state.header.stamp = rospy.Time.now()
		pub.publish(quad_state)
		# rospy.loginfo("quad_state.pos = [%f, %f, %f - %f]", quad_state.pos.x, quad_state.pos.y, quad_state.pos.z, quad_state.pos.yaw)
	# rospy.loginfo("quad_state.vel = [%f, %f, %f - %f]", quad_state.vel.x, quad_state.vel.y, quad_state.vel.z, quad_state.vel.yaw)

	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	rospy.init_node('quad_info', anonymous=True)
	rospy.loginfo("Node %s started!", rospy.get_name())

	pub = rospy.Publisher('quad_state', QuadState, queue_size=10)
	quad_state = QuadState()
	quad_state.header.frame_id = 'world'
	quad_state.pos.x = 0; quad_state.pos.y = 0; quad_state.pos.z = 0; quad_state.pos.yaw = 0
	quad_state.vel.x = 0; quad_state.vel.y = 0; quad_state.vel.z = 0; quad_state.vel.yaw = 0
	
	t = 0.0

	try:
		rospy.Subscriber('ground_truth_to_tf/pose', PoseStamped, callback)
		rospy.loginfo("Node %s start spining!", rospy.get_name())
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		quad_state.pos.x = 0; quad_state.pos.y = 0; quad_state.pos.z = 0; quad_state.pos.yaw = 0
		quad_state.vel.x = 0; quad_state.vel.y = 0; quad_state.vel.z = 0; quad_state.vel.yaw = 0
		quad_state.header.stamp = rospy.Time.now()
		pub.publish(quad_state)
		rospy.loginfo("Node %s finished!", rospy.get_name())
