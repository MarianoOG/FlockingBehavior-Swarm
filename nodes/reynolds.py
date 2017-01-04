#!/usr/bin/env python
import message_filters, rospy, sys
from math import sqrt
from geometry_msgs.msg import Vector3
from swarm.msg import QuadStamped, QuadState
from swarm.srv import QuadDistance

def quad_distance_client(quad, other):
    rospy.wait_for_service('/quad_distance')
    try:
        quad_distance = rospy.ServiceProxy('/quad_distance', QuadDistance)
        resp = quad_distance(quad, other)
        return resp.d
    except rospy.ServiceException, e:
        rospy.loginfo("Service failed!")

def m_pso_callback(quad_state, args):
    global pub

    n = args[0]
    current = args[1]
    
    a_pot = Vector3()
    a_slip = Vector3()
    
    for i in range(n):
        if not i==current:
            d = quad_distance_client(quad_state, i)
            rospy.loginfo("distance_%i_%i = %f", current, i, d)

    if quad.header.stamp.secs >= 1:
        quad.z = 1.0

    # Publish:
    try:
        quad.header.stamp = rospy.Time.now()
        pub.publish(quad)
    except rospy.ROSException:
        pass

if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)

    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])

    name = rospy.get_namespace()
    current = int(name[-2])
    xy = rospy.get_param(name)

    pub = rospy.Publisher('des_pos', QuadStamped, queue_size=10)

    quad = QuadStamped()
    quad.header.frame_id = 'world'
    quad.x = xy['x']; quad.y = xy['y']; quad.z = 0.0; quad.yaw = 0.0
    
    try:
        rospy.Subscriber('next_generation', QuadState, m_pso_callback, (n, current))
        rospy.loginfo("Start spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.header.stamp = rospy.Time.now()
        quad.x = xy['x']; quad.y = xy['y']; quad.z = 0.0; quad.yaw = 0.0
        pub.publish(quad)
        rospy.loginfo("End of node")
