#!/usr/bin/env python
import message_filters, rospy
from math import sin, cos, pi
from geometry_msgs.msg import Twist
from swarm.msg import QuadStamped, QuadState

def des_callback(q_des):
	global des
	des = q_des
	# rospy.loginfo("des = [%f, %f, %f - %f]", des.x, des.y, des.z, des.yaw)

def info_callback(state):
	global pub, twist, des

	# Constant variables:
	kp = [2.004,9.000,8.2] # Ku = [2.88,10.713,11.71]
	kd = [0.668,1.607,1.7] # Tu = [2.32,01.500,1.025]

	# Size compensation in Z:
	min_z = 0.1824
	if des.z < min_z: des.z = min_z

	# Position Controller:
	e_x = state.pos.x - des.x
	e_y = state.pos.y - des.y
	twist.linear.x = - kd[0] * (state.vel.y * sin(state.pos.yaw) + state.vel.x * cos(state.pos.yaw)) - kp[0] * (e_y * sin(state.pos.yaw) + e_x * cos(state.pos.yaw))
	twist.linear.y = - kd[0] * (state.vel.y * cos(state.pos.yaw) - state.vel.x * sin(state.pos.yaw)) - kp[0] * (e_y * cos(state.pos.yaw) - e_x * sin(state.pos.yaw))
	twist.linear.z = - kd[1] * state.vel.z - kp[1] * (state.pos.z - des.z)
	
	# Yaw angle controller:
	e_yaw1 = state.pos.yaw - des.yaw
	if des.yaw > 0: e_yaw2 = e_yaw1 + 2 * pi
	else: e_yaw2 = e_yaw1 - 2 * pi
	if abs(e_yaw1) < abs(e_yaw2): e_yaw = e_yaw1
	else: e_yaw = e_yaw2
	twist.angular.z = - kd[2] * state.vel.yaw - kp[2] * e_yaw

	# Limit:
	if abs(twist.linear.x) < 0.05: twist.linear.x = 0.0
	if abs(twist.linear.y) < 0.05: twist.linear.y = 0.0
	if abs(twist.linear.z) < 0.05: twist.linear.z = 0.0
	if abs(twist.angular.z) < 0.05: twist.angular.z = 0.0

	# Publish twist:
	try:
		pub.publish(twist)
		# rospy.loginfo("quad_twist = [%f, %f, %f - %f]", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)
	except rospy.ROSException:
		pass

if __name__ == '__main__':
	rospy.init_node('pos_controller', anonymous=True)
	rospy.loginfo("Node %s started!", rospy.get_name())

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	des = QuadStamped()

	try:
		rospy.sleep(1)
		rospy.Subscriber('quad_state', QuadState, info_callback)
		rospy.Subscriber('des_pos', QuadStamped, des_callback)
		rospy.loginfo("Node %s start spining!", rospy.get_name())
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)
		rospy.loginfo("Node %s finished!", rospy.get_name())
