#!/usr/bin/env python
import rospy
from swarm.msg import QuadStamped, QuadState

def des_callback(q_des):
	global des
	des = q_des
	# rospy.loginfo("des = [%f, %f, %f - %f]", des.x, des.y, des.z, des.yaw)

def info_callback(state):
	global pub, quad, des

	quad = state
	quad.pos.x = des.x
	quad.pos.y = des.y
	quad.pos.z = des.z
	quad.pos.yaw = des.yaw

	# Publish:
	try:
		quad.header.stamp = rospy.Time.now()
		pub.publish(quad)

	except rospy.ROSException:
		pass

if __name__ == '__main__':
	rospy.init_node('mid_reynolds', anonymous=True)
	rospy.loginfo("Node %s started!", rospy.get_name())

	des = QuadStamped()
	pub = rospy.Publisher('next_generation', QuadState, queue_size=10)
	
	quad = QuadState()

	try:
		rospy.sleep(1)
		rospy.Subscriber('quad_state', QuadState, info_callback)
		rospy.Subscriber('mid_state', QuadStamped, des_callback)
		rospy.loginfo("Node %s start spining!", rospy.get_name())
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	finally:
		quad.header.stamp = rospy.Time.now()
		pub.publish(quad)
		rospy.loginfo("Node %s finished!", rospy.get_name())
