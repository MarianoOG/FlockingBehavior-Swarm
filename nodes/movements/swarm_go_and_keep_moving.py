#!/usr/bin/env python
import rospy, sys
from math import pi
from random import random
from swarm.msg import QuadStamped

if __name__ == '__main__':
    rospy.init_node('swarm_go_and_keep_moving', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)
    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])

    pub = []
    quad = []
    for i in range(n):
        pub.append(rospy.Publisher('/uav' + str(i) + '/mid_state', QuadStamped, queue_size=n))
        quad.append(QuadStamped())
        quad[i].header.frame_id = 'world'
        xy = rospy.get_param('/uav' + str(i))
        quad[i].x = xy['x']
        quad[i].y = xy['y']
        quad[i].z = 0
        quad[i].yaw = 0

    try:
        t = 1
        rospy.loginfo("Node %s start sending!", rospy.get_name())
        while not rospy.is_shutdown():
            for i in range(n):
                quad[i].header.stamp = rospy.Time.now()
                if quad[i].header.stamp.secs >= t:
                    if i==0:
                        quad[i].x = 10.0 * random() - 5.0
                        quad[i].y = 10.0 * random() - 5.0
                        quad[i].z = 3 * random() + 0.5
                        quad[i].yaw = 2 * pi * random() - pi
                    else:
                        xy = rospy.get_param('/uav' + str(i))
                        quad[i].x = quad[0].x + xy['x']
                        quad[i].y = quad[0].y + xy['y']
                        quad[i].z = quad[0].z + random() / 5.0
                        quad[i].yaw = quad[0].yaw + random() / 5.0
                    if i==n-1:
                        t += 4
                        rospy.loginfo("%s: t = %i!", rospy.get_name(), t)
                pub[i].publish(quad[i])
                rospy.loginfo("%f = [%f, %f, %f - %f]", i, quad[i].x, quad[i].y, quad[i].z, quad[i].yaw)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        for i in range(n):
            xy = rospy.get_param('/uav' + str(i))
            quad[i].x = xy['x']
            quad[i].y = xy['y']
            quad[i].z = 0
            quad[i].yaw = 0
            quad[i].header.stamp = rospy.Time.now()
            pub[i].publish(quad[i])
        rospy.loginfo("Node %s finished!", rospy.get_name())
