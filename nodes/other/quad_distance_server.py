#!/usr/bin/env python

from math import sqrt
from geometry_msgs.msg import Vector3
from swarm.msg import QuadState
from swarm.srv import QuadDistance
import rospy

def handle_quad_distance(req):
    d = -1.0
    quad = QuadState()

    try:
        quad = rospy.wait_for_message('/uav' + str(req.other) + '/next_generation', QuadState)
        
        e = Vector3()
        e.x = quad.pos.x - req.quad.pos.x
        e.y = quad.pos.y - req.quad.pos.y
        e.z = quad.pos.z - req.quad.pos.z
        
        d = sqrt(e.x * e.x + e.y * e.y + e.z * e.z)

    except rospy.ROSException:
        pass

    # rospy.loginfo("Distance is %f", d)
    return d

if __name__ == "__main__":
    rospy.init_node('quad_distance_server')
    
    s = rospy.Service('quad_distance', QuadDistance, handle_quad_distance)

    rospy.loginfo("Start spinning")
    rospy.spin()
