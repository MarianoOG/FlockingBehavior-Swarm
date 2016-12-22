#!/usr/bin/env python
import rospy
from math import sin, cos, pi
from swarm.msg import QuadStamped

if __name__ == '__main__':
    rospy.init_node('S4_go_up_and_stay', anonymous=True)
    rate = rospy.Rate(100)
    
    x = [0.0,0.0,-1.0,1.0]
    y = [-1.0,1.0,0.0,0.0]

    pub = []
    quad = []
    for i in range(4):
        pub.append(rospy.Publisher('/uav' + str(i) + '/des_pos', QuadStamped, queue_size=4))
        quad.append(QuadStamped())
        quad[i].header.frame_id = 'world'
        quad[i].x = x[i]
        quad[i].y = y[i]
        quad[i].z = 0
        quad[i].yaw = 0

    try:
        yaw = [0.0,-pi,-pi/2.0,pi/2.0]
        while not rospy.is_shutdown():
            for i in range(4):
                quad[i].header.stamp = rospy.Time.now()
                if quad[i].header.stamp.secs >= 2:
                    quad[i].z = 1.0
                    if quad[i].yaw == 0.0: quad[i].yaw = yaw[i]
                if quad[i].header.stamp.secs >= 3:
                    quad[i].yaw += pi / 200.0
                    if quad[i].yaw > pi: quad[i].yaw -= 2 * pi
                    quad[i].x = sin(quad[i].yaw)
                    quad[i].y = - cos(quad[i].yaw)
                pub[i].publish(quad[i])
                rospy.loginfo("%i = [%f, %f, %f - %f]", i, quad[i].x, quad[i].y, quad[i].z, quad[i].yaw)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        for i in range(4):
            quad[i].header.stamp = rospy.Time.now()
            quad[i].x = 0
            quad[i].y = 0
            quad[i].z = 0
            quad[i].yaw = 0
            pub[i].publish(quad[i])
        rospy.loginfo("End of node")
