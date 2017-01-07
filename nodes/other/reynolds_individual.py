#!/usr/bin/env python
import message_filters, rospy, sys
from math import sqrt
from geometry_msgs.msg import Vector3
from swarm.msg import QuadStamped, QuadState

def reynolds_callback(quad_state, quad_other):
    global other, pub, quad, n, current

    # rospy.loginfo("q = [%f %f %f - %f] %s", quad_state.pos.x, quad_state.pos.y, quad_state.pos.z, quad_state.pos.yaw, rospy.get_namespace())

    # Repulsion parameters:
    D = 0.65
    r0 = 1.5
    r1 = 1.0

    # Alignement parameters:
    c_frict = 0.1
    r2 = 2.0
    r3 = 1.1
    r4 = 1.9

    e = QuadState()
    e.pos.x = quad_other.pos.x - quad_state.pos.x
    e.pos.y = quad_other.pos.y - quad_state.pos.y
    e.pos.z = quad_other.pos.z - quad_state.pos.z
    e.vel.x = quad_other.vel.x - quad_state.vel.x
    e.vel.y = quad_other.vel.y - quad_state.vel.y
    e.vel.z = quad_other.vel.z - quad_state.vel.z
    
    d = sqrt(e.pos.x * e.pos.x + e.pos.y * e.pos.y + e.pos.z * e.pos.z)

    # Modification vectors for repulsion and alignement:
    quad.x = quad_state.pos.x
    quad.y = quad_state.pos.y
    quad.z = quad_state.pos.z

    if d<r0:
        m = min(r1,r0-d)
        quad.x -= D * m * e.pos.x / d
        quad.y -= D * m * e.pos.y / d
        quad.z -= D * m * e.pos.z / d

    if d<r2:
        d2 = max(d-(r2-r4),r3)
        d2 = d2 * d2
        quad.x += c_frict * (quad_other.vel.x - quad_state.vel.x / d2)
        quad.y += c_frict * (quad_other.vel.y - quad_state.vel.y / d2)
        quad.z += c_frict * (quad_other.vel.z - quad_state.vel.z / d2)

if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)
    
    # n number of quads in total:
    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])
    
    # Counter for other quads
    other = 0
    
    # Current quad and initial position:
    name = rospy.get_namespace()
    current = int(name[-2])
    xy = rospy.get_param(name)

    # Publisher and initial message:
    pub = rospy.Publisher('des_pos', QuadStamped, queue_size=10)
    quad = QuadStamped()
    quad.header.frame_id = 'world'
    quad.x = xy['x']; quad.y = xy['y']; quad.z = 0.0; quad.yaw = 0.0

    sub = []
    ts = []
    for i in range(n):
        sub.append(message_filters.Subscriber('/uav' + str(i) + '/next_generation', QuadState))
    for i in range(n):
        if not current==i:
            ts.append(message_filters.ApproximateTimeSynchronizer([sub[current],sub[i]], 10, 0.01))

    try:
        rospy.loginfo("Node %s start subscribing!", rospy.get_name())
        while not rospy.is_shutdown():
            for i in range(n-1):
                ts[i].registerCallback(reynolds_callback)
            quad.header.stamp = rospy.Time.now()
            pub.publish(quad)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.header.stamp = rospy.Time.now()
        quad.x = xy['x']; quad.y = xy['y']; quad.z = 0.0; quad.yaw = 0.0
        pub.publish(quad)
        rospy.loginfo("Node %s finished!", rospy.get_name())
