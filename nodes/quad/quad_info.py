#!/usr/bin/env python
import message_filters, rospy
from math import atan2
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Imu
from swarm.msg import QuadState

def info_callback(imu, pose):
	global quad_state, pos_prev, pub, t_prev

	# Calculate change in time:
	t = pose.header.stamp.secs + pose.header.stamp.nsecs/1000000000.0
	delta = t - t_prev
	t_prev = t

	# Update position:
	pos_prev.x = quad_state.pos.x
	pos_prev.y = quad_state.pos.y
	pos_prev.z = quad_state.pos.z
	quad_state.pos.x = pose.pose.position.x
	quad_state.pos.y = pose.pose.position.y
	quad_state.pos.z = pose.pose.position.z
	
	# Update yaw angle:
	ysqr = imu.orientation.y * imu.orientation.y
	q0 = -2.0 * (ysqr + imu.orientation.z * imu.orientation.z) + 1.0
	q1 = 2.0 * (imu.orientation.x * imu.orientation.y - imu.orientation.w * imu.orientation.z)
	quad_state.pos.yaw = - atan2(q1, q0)
	
	# Update velocities:
	quad_state.vel.x = (quad_state.pos.x - pos_prev.x) / delta
	quad_state.vel.y = (quad_state.pos.y - pos_prev.y) / delta
	quad_state.vel.z = (quad_state.pos.z - pos_prev.z) / delta
	quad_state.vel.yaw = imu.angular_velocity.z

	# Publish twist:
	try:
		quad_state.header.stamp = rospy.Time.now()
		pub.publish(quad_state)
	except rospy.ROSException:
		pass

	# Prints:
	rospy.loginfo("quad_state.pos = [%f, %f, %f - %f]", quad_state.pos.x, quad_state.pos.y, quad_state.pos.z, quad_state.pos.yaw)
	rospy.loginfo("quad_state.vel = [%f, %f, %f - %f]", quad_state.vel.x, quad_state.vel.y, quad_state.vel.z, quad_state.vel.yaw)

if __name__ == '__main__':
	rospy.init_node('quad_info', anonymous=True)
	imu_sub = message_filters.Subscriber('raw_imu', Imu)
	pose_sub = message_filters.Subscriber('ground_truth_to_tf/pose', PoseStamped)
	ts = message_filters.ApproximateTimeSynchronizer([imu_sub, pose_sub], 10, 0.015)
	
	quad_state = QuadState()
	quad_state.header.frame_id = 'world'
	pos_prev = Vector3()
	pub = rospy.Publisher('quad_state', QuadState, queue_size=10)
	quad_state.pos.x = 0; quad_state.pos.y = 0; quad_state.pos.z = 0; quad_state.pos.yaw = 0
	quad_state.vel.x = 0; quad_state.vel.y = 0; quad_state.vel.z = 0; quad_state.vel.yaw = 0
	t_prev = 0

	try:
		ts.registerCallback(info_callback)
		rospy.loginfo("Start spinning")
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		quad_state.pos.x = 0; quad_state.pos.y = 0; quad_state.pos.z = 0; quad_state.pos.yaw = 0
		quad_state.vel.x = 0; quad_state.vel.y = 0; quad_state.vel.z = 0; quad_state.vel.yaw = 0
		quad_state.header.stamp = rospy.Time.now()
		pub.publish(quad_state)
		rospy.loginfo("End of node")
