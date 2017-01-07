#!/usr/bin/env python
import rospy
from swarm.msg import QuadFitness

def best_callback(val):
    global best
    
    best = val

if __name__ == '__main__':
    rospy.init_node('best_fit', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)

    # Publishers:
    pub = rospy.Publisher('/last_best', QuadFitness, queue_size=10)
    best = QuadFitness()
    
    try:
        rospy.loginfo("Node %s start subscribing!", rospy.get_name())
        while not rospy.is_shutdown():
            rospy.Subscriber('/swarm_best', QuadFitness, best_callback)
            pub.publish(best)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Node %s finished!", rospy.get_name())
