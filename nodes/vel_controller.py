#!/usr/bin/env python
import message_filters, rospy
from math import atan2, sin, cos, pi
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import Imu
from swarm.msg import QuadHoverPos

def info_callback(imu, pose, quad):
	global pos, pos_prev, pub, t_prev, twist

	# Calculate change in time:
	t = pose.header.stamp.secs + pose.header.stamp.nsecs/1000000000.0
	delta = t - t_prev
	t_prev = t

	# Update position: 
	pos_prev = pos
	pos = pose.pose.position
	
	# Update yaw angle:
	ysqr = imu.orientation.y * imu.orientation.y
	q0 = -2.0 * (ysqr + imu.orientation.z * imu.orientation.z) + 1.0
	q1 = 2.0 * (imu.orientation.x * imu.orientation.y - imu.orientation.w * imu.orientation.z)
	yaw = atan2(q1, q0)
	
	# Calculate linear velocity:
	vel = Vector3()
	vel.x = (pos.x - pos_prev.x) / delta
	vel.y = (pos.y - pos_prev.y) / delta
	vel.z = (pos.z - pos_prev.z) / delta
	
	# Constant variables:
	# Ku = [2.88,10.713,11.71]
	# Tu = [2.35,01.500,1.025]
	kp = [2.305,9.000,8.2]
	kd = [0.723,1.607,1.7]

	# Linear controll:
	sec_z = 0.1824
	if quad.position.z < sec_z: quad.position.z = sec_z
	e_x = pos.x - quad.position.x
	e_y = pos.y - quad.position.y
	twist.linear.x = - kd[0] * (vel.x * cos(yaw) - vel.y * sin(yaw)) - kp[0] * (e_x * cos(yaw) - e_y * sin(yaw))
	twist.linear.y = - kd[0] * (vel.x * sin(yaw) + vel.y * cos(yaw)) - kp[0] * (e_x * sin(yaw) + e_y * cos(yaw))
	twist.linear.z = - kd[1] * vel.z - kp[1] * (pos.z - quad.position.z)
	
	# Angular controll:
	e_yaw1 = - yaw - quad.yaw
	if quad.yaw > 0: e_yaw2 = - yaw - quad.yaw + 2 * pi
	else: e_yaw2 = - yaw - quad.yaw - 2 * pi
	if abs(e_yaw1) < abs(e_yaw2): e_yaw = e_yaw1
	else: e_yaw = e_yaw2
	twist.angular.z = - kd[2] * imu.angular_velocity.z - kp[2] * e_yaw

	# Prints:
	rospy.loginfo("quad = [%f, %f, %f - %f]", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

	# Publish twist:
	try:
		pub.publish(twist)
	except rospy.ROSException:
		pass

if __name__ == '__main__':
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	rospy.init_node('vel_controller', anonymous=True)
	imu_sub = message_filters.Subscriber('raw_imu', Imu)
	pose_sub = message_filters.Subscriber('ground_truth_to_tf/pose', PoseStamped)
	des_twist_sub = message_filters.Subscriber('des_pos', QuadHoverPos)
	ts = message_filters.ApproximateTimeSynchronizer([imu_sub, pose_sub, des_twist_sub], 2,0.01)
	pos = Vector3()
	pos_prev = Vector3()
	t_prev = 0
	twist = Twist()
	twist.angular.x = 0 
	twist.angular.y = 0

	try:
		ts.registerCallback(info_callback)
		rospy.loginfo("Start spinning")
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0; twist.angular.z = 0
		pub.publish(twist)
		rospy.loginfo("End of node")
