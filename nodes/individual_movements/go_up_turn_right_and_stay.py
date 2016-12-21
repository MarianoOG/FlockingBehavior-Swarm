#!/usr/bin/env python
import rospy
from math import pi
from swarm.msg import QuadHoverPos

if __name__ == '__main__':
    pub = rospy.Publisher('des_pos', QuadHoverPos, queue_size=1)
    rospy.init_node('go_up_and_stay', anonymous=True)
    rate = rospy.Rate(100)
    
    quad = QuadHoverPos();
    quad.header.frame_id = 'world'
    quad.position.x = 0
    quad.position.y = 0
    quad.position.z = 0
    quad.yaw = 0

    try:
        while not rospy.is_shutdown():
            quad.header.stamp = rospy.Time.now()
            if quad.header.stamp.secs >= 2:
                quad.position.z = 1
            if quad.header.stamp.secs >= 3:
                quad.yaw = pi / 2.0
            if quad.header.stamp.secs >= 5:
                quad.position.x = 1
            pub.publish(quad)
            rospy.loginfo("[%f, %f, %f - %f]", quad.position.x, quad.position.y, quad.position.z, quad.yaw)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        quad.header.stamp = rospy.Time.now()
        quad.position.x = 0
        quad.position.z = 0
        quad.yaw = 0
        pub.publish(quad)
        rospy.loginfo("End of node")
