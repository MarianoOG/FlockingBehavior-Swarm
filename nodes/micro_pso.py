#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from swarm.msg import QuadState

def state_callback(state):
    global gBest, pBest, pub, pubBest, quad, t
    
    quad = state

    # Publish:
    try:    
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)
        # rospy.loginfo("%squad_pos = [%f, %f, %f - %f]", rospy.get_namespace(), quad.pos.x, quad.pos.y, quad.pos.z, quad.pos.yaw)
        # rospy.loginfo("quad_vel = [%f, %f, %f - %f]", quad.vel.x, quad.vel.y, quad.vel.z, quad.vel.yaw)
        t += 1
        if t==3:
           pubBest.publish(gBest)
           t = 0
           # rospy.loginfo("%s is sending", rospy.get_namespace())

    except rospy.ROSException:
        pass

def gBest_callback(val):
    global gBest
    gBest = val
    # rospy.loginfo("%s is recieving Gbest = %f", rospy.get_namespace(), gBest.data)

if __name__ == '__main__':
    rospy.init_node('micro_pso', anonymous=True)

    pub = rospy.Publisher('next_generation', QuadState, queue_size=10)
    pubBest = rospy.Publisher('/swarm_best', Float64, queue_size=10)
    gBest = Float64()
    gBest.data = 0.0
    pBest = 0.0

    quad = QuadState()
    quad.header.frame_id = 'world'

    xy = rospy.get_param(rospy.get_namespace())
    quad.pos.x = xy['x']; quad.pos.y = xy['y']; quad.pos.z = 0.0; quad.pos.yaw = 0.0
    quad.vel.x = 0.0; quad.vel.y = 0.0; quad.vel.z = 0.0; quad.vel.yaw = 0.0
    
    try:
        t = 0
        rospy.Subscriber('/swarm_best', Float64, gBest_callback)
        rospy.Subscriber('quad_state', QuadState, state_callback)
        rospy.loginfo("Start spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.pos.x = xy['x']; quad.pos.y = xy['y']; quad.pos.z = 0.0; quad.pos.yaw = 0.0
        quad.vel.x = 0.0; quad.vel.y = 0.0; quad.vel.z = 0.0; quad.vel.yaw = 0.0
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)
        rospy.loginfo("End of node")
