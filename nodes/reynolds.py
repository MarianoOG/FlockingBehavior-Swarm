#!/usr/bin/env python
import rospy, sys
from math import sqrt
from swarm.msg import QuadStamped, QuadState

def apply_reynolds_rules(quad_state, n, D, c_frict, r, a_pot, a_slip, nn):
    global quad
    
    # Restart variables:
    for i in range(n):
        a_pot[i].x = 0.0
        a_pot[i].y = 0.0
        a_pot[i].z = 0.0
        a_pot[i].yaw = 0.0

        a_slip[i].x = 0.0
        a_slip[i].y = 0.0
        a_slip[i].z = 0.0
        a_slip[i].yaw = 0.0

        nn[i] = 0.0

    dist = QuadStamped()
    for i in range(n):
        for j in range(i,n):
            if not(i==j):
                dist.x = quad_state[j].pos.x - quad_state[i].pos.x
                dist.y = quad_state[j].pos.y - quad_state[i].pos.y
                dist.z = quad_state[j].pos.z - quad_state[i].pos.z
                dist.yaw = quad_state[j].pos.yaw - quad_state[i].pos.yaw
                
                d = sqrt(dist.x*dist.x + dist.y*dist.y + dist.z*dist.z  + dist.yaw*dist.yaw)

                if d>0.0:
                    
                    if d<r[2]:
                        d2 = max(d-(r[2]-r[4]),r[3])
                        d2 = d2 * d2
                        nn[i] += 1.0
                        nn[j] += 1.0
                        a_slip[i].x += quad_state[j].vel.x - quad_state[i].vel.x / d2
                        a_slip[i].y += quad_state[j].vel.y - quad_state[i].vel.y / d2
                        a_slip[i].z += quad_state[j].vel.z - quad_state[i].vel.z / d2
                        a_slip[i].yaw += quad_state[j].vel.yaw - quad_state[i].vel.yaw / d2
                        a_slip[j].x -= a_slip[i].x
                        a_slip[j].y -= a_slip[i].y
                        a_slip[j].z -= a_slip[i].z
                        a_slip[j].yaw -= a_slip[i].yaw

                        if d<r[0]:
                            a_pot[i].x += min(r[1],r[0]-d)*dist.x / d
                            a_pot[i].y += min(r[1],r[0]-d)*dist.y / d
                            a_pot[i].z += min(r[1],r[0]-d)*dist.z / d
                            a_pot[i].yaw += min(r[1],r[0]-d)*dist.yaw / d
                            a_pot[j].x -= a_pot[i].x
                            a_pot[j].y -= a_pot[i].y
                            a_pot[j].z -= a_pot[i].z
                            a_pot[j].yaw -= a_pot[i].yaw

        m = max(nn[i],1)
        a_slip[i].x /= m
        a_slip[i].y /= m
        a_slip[i].z /= m
        a_slip[i].yaw /= m
        
        # Final movements:
        quad[i].x = quad_state[i].pos.x - D * a_pot[i].x + c_frict * a_slip[i].x
        quad[i].y = quad_state[i].pos.y - D * a_pot[i].y + c_frict * a_slip[i].y
        quad[i].z = quad_state[i].pos.z - D * a_pot[i].z + c_frict * a_slip[i].z
        quad[i].yaw = quad_state[i].pos.yaw - D * a_pot[i].yaw + c_frict * a_slip[i].yaw

        # rospy.loginfo("i: %i, quad [%f, %f, %f]", i, quad[i].x, quad[i].y, quad[i].z)

def quad_callback(other_quad, i):
    global quad_state

    quad_state[i].pos = other_quad.pos
    quad_state[i].vel = other_quad.vel

if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)

    # n number of quads in total:
    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])

    # Repulsion & alignment parameters:
    D = 0.65
    c_frict = 0.1
    r = [1.5, 1.0, 2.0, 1.1, 1.9]
    
    # Publisher, initial messages and initial variables:
    pub = []
    quad = []
    quad_state = []
    a_pot = []
    a_slip = []
    nn = []
    for i in range(n):
        pub.append(rospy.Publisher('/uav' + str(i) + '/des_pos', QuadStamped, queue_size=10))
        quad.append(QuadStamped())
        quad[i].header.frame_id = 'world'
        xy = rospy.get_param('/uav' + str(i))
        quad[i].x = xy['x']; quad[i].y = xy['y']; quad[i].z = 0.0; quad[i].yaw = 0.0
        quad_state.append(QuadState())
        a_pot.append(QuadStamped())
        a_slip.append(QuadStamped())
        nn.append(0.0)

    try:
        rospy.loginfo("Node %s start subscribing!", rospy.get_name())
        while not rospy.is_shutdown():
            for i in range(n):
                rospy.Subscriber('/uav' + str(i) + '/next_generation', QuadState, quad_callback, i)
            apply_reynolds_rules(quad_state, n, D, c_frict, r, a_pot, a_slip, nn)
            for i in range(n):
                quad[i].header.stamp = rospy.Time.now()
                pub[i].publish(quad[i])
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        for i in range(n):
            xy = rospy.get_param('/uav' + str(i))
            quad[i].x = xy['x']; quad[i].y = xy['y']; quad[i].z = 0.0; quad[i].yaw = 0.0
            quad[i].header.stamp = rospy.Time.now()
            pub[i].publish(quad[i])
        rospy.loginfo("Node %s finished!", rospy.get_name())
