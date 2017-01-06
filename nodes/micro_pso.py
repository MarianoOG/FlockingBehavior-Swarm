#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from swarm.msg import QuadState, QuadFitness
from swarm.srv import Fitness

def gBest_callback(val):
    global gBest, pubBest
    
    # rospy.loginfo("%s is recieving Gbest = %f", rospy.get_namespace(), gBest.fitness)

    if gBest.fitness <= val.fitness:
        gBest = val
    else:
        pubBest.publish(gBest)

def pso_callback(val):
    global pso
    pso = val

def state_callback(state):
    global gBest, pBest, pub, pubBest, quad, t
    
    quad.pos.x = state.pos.x
    quad.pos.y = state.pos.y
    quad.pos.z = state.pos.z
    quad.vel.x = state.vel.x
    quad.vel.y = state.vel.y
    quad.vel.z = state.vel.z
    
    if state.header.stamp.secs >= 1:
        quad.pos.z = 1.0

    # Publish:
    try:
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)

        t += 1
        if t==3:
           pubBest.publish(gBest)
           t = 0
           # rospy.loginfo("%s is sending", rospy.get_namespace())

    except rospy.ROSException:
        pass

if __name__ == '__main__':
    rospy.init_node('micro_pso', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)

    pub = rospy.Publisher('next_generation', QuadState, queue_size=10)
    pubBest = rospy.Publisher('/swarm_best', QuadFitness, queue_size=10)
    gBest = QuadFitness()
    gBest.fitness = 0.0
    pBest = 0.0

    quad = QuadState()
    quad.header.frame_id = 'world'

    xy = rospy.get_param(rospy.get_namespace())
    quad.pos.x = xy['x']; quad.pos.y = xy['y']; quad.pos.z = 0.0; quad.pos.yaw = 0.0
    quad.vel.x = 0.0; quad.vel.y = 0.0; quad.vel.z = 0.0; quad.vel.yaw = 0.0
    
    try:
        t = 0    
        rospy.Subscriber('/swarm_best', QuadFitness, gBest_callback)
        rospy.Subscriber('quad_state', QuadState, state_callback)
        rospy.loginfo("Node %s start spining!", rospy.get_name())
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.pos.x = xy['x']; quad.pos.y = xy['y']; quad.pos.z = 0.0; quad.pos.yaw = 0.0
        quad.vel.x = 0.0; quad.vel.y = 0.0; quad.vel.z = 0.0; quad.vel.yaw = 0.0
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)
        rospy.loginfo("Node %s finished!", rospy.get_name())
