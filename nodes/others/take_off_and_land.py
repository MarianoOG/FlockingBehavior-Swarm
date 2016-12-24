#!/usr/bin/env python
import rospy
from math import pi, sin
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('take_off_and_land', anonymous=True)
    rate = rospy.Rate(100)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    twist = Twist()

    try:
        t = 0
        while not rospy.is_shutdown():
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)
            t = (t + 1) % 10
            z = sin(t*2*pi/10)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.loginfo("End of node")
