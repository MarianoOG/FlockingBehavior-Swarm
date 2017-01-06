#!/usr/bin/env python
import message_filters, rospy, sys
from math import sqrt
from geometry_msgs.msg import Vector3
from swarm.msg import QuadStamped, QuadState

def m_pso_callback(quad_state, args):
    global other, pub, quad

    n = args[0]
    current = args[1]

    a_pot = Vector3()
    a_slip = Vector3()
    
    if other==current: 
        other += 1
    if other>=n:
        if current==0:
            other = 1
        else:
            other = 0

    rospy.loginfo("%i -> %i", current, other)

    d = -1.0

    try:
        
        quad_other = rospy.wait_for_message('/uav' + str(other) + '/next_generation', QuadState)

        e = Vector3()
        e.x = quad_other.pos.x - quad_state.pos.x
        e.y = quad_other.pos.y - quad_state.pos.y
        e.z = quad_other.pos.z - quad_state.pos.z
        
        d = sqrt(e.x * e.x + e.y * e.y + e.z * e.z)

        rospy.loginfo("distance_%i_%i = %f", current, other, d)

        if quad.header.stamp.secs >= 1:
            quad.z = 1.0

        other += 1

    except rospy.ROSException:
        pass

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

    try:
        while not rospy.is_shutdown():
            rospy.Subscriber('next_generation', QuadState, m_pso_callback, (n, current))
            quad.header.stamp = rospy.Time.now()
            pub.publish(quad)
            rospy.loginfo("Node %s start spining!", rospy.get_name())
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.header.stamp = rospy.Time.now()
        quad.x = xy['x']; quad.y = xy['y']; quad.z = 0.0; quad.yaw = 0.0
        pub.publish(quad)
        rospy.loginfo("Node %s finished!", rospy.get_name())
