#!/usr/bin/env python
import message_filters, rospy
from math import sin, cos, pi
from geometry_msgs.msg import Twist
from swarm.msg import QuadStamped, QuadState

def info_callback(state, des):
	global pub, twist

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

	# Publish twist:
	try:
		pub.publish(twist)
	except rospy.ROSException:
		pass

	# Prints:
	rospy.loginfo("quad_twist = [%f, %f, %f - %f]", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

if __name__ == '__main__':
	rospy.init_node('pos_controller', anonymous=True)
	state_sub = message_filters.Subscriber('quad_state', QuadState)
	des_sub = message_filters.Subscriber('des_pos', QuadStamped)
	ts = message_filters.ApproximateTimeSynchronizer([state_sub, des_sub], 2,0.01)

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	twist = Twist()
	twist.angular.x = 0 
	twist.angular.y = 0

	try:
		rospy.sleep(1)
		ts.registerCallback(info_callback)
		rospy.loginfo("Start spinning")
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0; twist.angular.z = 0
		pub.publish(twist)
		rospy.loginfo("End of node")
