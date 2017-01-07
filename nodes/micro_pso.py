#!/usr/bin/env python
import rospy
from math import sqrt, sin
from random import random
from std_msgs.msg import Int32
from swarm.msg import QuadState, QuadFitness
from swarm.srv import Fitness

def fun(x, y, z, yaw):
    # z = z-2; y = y-2; x = x-2; yaw = yaw-1
    # r = sqrt(x*x+y*y+z*z+yaw*yaw) + 0.0001
    # f = sin(r)/r
    f = - (x - 1) * (x - 1) - (y - 1) * (y - 1) - abs(z - 1) - abs(yaw - 1) # max(1,2,1,1)
    return f

def fitness_function(quad):
    fitness = fun(quad.pos.x, quad.pos.y, quad.pos.z, quad.pos.yaw)
    return fitness

def gBest_callback(val):
    global pubBest, gBest
    
    # rospy.loginfo("%s is recieving Gbest = %f", rospy.get_namespace(), gBest.fitness)
    
    if gBest.fitness <= val.fitness:
        gBest = val
    else:
        try:
            pubBest.publish(gBest)
        except rospy.ROSException:
            pass

# def pso_callback(val):
#     global pso
#     pso = val

def state_callback(quad, ab):
    global gBest, pBest, pub, pubBest
    
    fitness = fitness_function(quad)

    # First value of gBest and pBest
    if gBest.quad.z == -1.0:
        gBest.quad = quad.pos
        gBest.fitness = fitness

    if pBest.quad.z == -1.0:
        pBest.quad = quad.pos
        pBest.fitness = fitness

    # Checking the local and global best state:
    if fitness > pBest.fitness:
        pBest.quad = quad.pos
        pBest.fitness = fitness

        if pBest.fitness > gBest.fitness:
            gBest = pBest
            try:
                pubBest.publish(gBest)
            except rospy.ROSException:
                pass

    # Actual velocity and random parameters
    vel = quad.vel
    r = [random(), random(), random()]

    # Local knoledge:
    vel.x += ab[0] * r[0] * (pBest.quad.x - quad.pos.x)
    vel.y += ab[0] * r[0] * (pBest.quad.y - quad.pos.y)
    vel.z += ab[0] * r[0] * (pBest.quad.z - quad.pos.z)
    vel.yaw += ab[0] * r[0] * (pBest.quad.yaw - quad.pos.yaw)

    # Global knoledge:
    vel.x += ab[1] * r[1] * (gBest.quad.x - quad.pos.x) + r[2]
    vel.y += ab[1] * r[1] * (gBest.quad.y - quad.pos.y) + r[2]
    vel.z += ab[1] * r[1] * (gBest.quad.z - quad.pos.z) + r[2]
    vel.yaw += ab[1] * r[1] * (gBest.quad.yaw - quad.pos.yaw) + r[2]

    # Normalize vector and add to position:
    c = 0.3
    d = sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z + vel.yaw * vel.yaw)
    quad.pos.x += c * vel.x / d
    quad.pos.y += c * vel.y / d
    quad.pos.z += c * vel.z / d
    quad.pos.yaw += vel.yaw / d

    # Publish:
    try:
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)

    except rospy.ROSException:
        pass

if __name__ == '__main__':
    rospy.init_node('micro_pso', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)

    # Publishers:
    pub = rospy.Publisher('next_generation', QuadState, queue_size=10)
    pubBest = rospy.Publisher('/swarm_best', QuadFitness, queue_size=10)
    
    # Initial state of variables:
    ab = [2.0, 2.0]
    gBest = QuadFitness(); gBest.quad.z = -1.0
    pBest = QuadFitness(); pBest.quad.z = -1.0
    # pso = ??
    
    try:
        rospy.Subscriber('/swarm_best', QuadFitness, gBest_callback)
        rospy.Subscriber('quad_state', QuadState, state_callback, ab)
        # rospy.Subscriber('/convergence', Int32, pso_callback)
        rospy.loginfo("Node %s start spining!", rospy.get_name())
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Node %s finished!", rospy.get_name())
