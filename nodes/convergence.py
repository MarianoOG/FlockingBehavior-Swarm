#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from swarm.msg import QuadFitness

def fit_callback(val):
    global best
    
    best = val

if __name__ == '__main__':
    rospy.init_node('convergence', anonymous=True)
    rospy.loginfo("Node %s started!", rospy.get_name())
    rate = rospy.Rate(100)

    # Publishers:
    pub = rospy.Publisher('/swarm_state', Int32, queue_size=10)
    best = QuadFitness()
    state = -1
    
    try:
        rospy.loginfo("Node %s start subscribing!", rospy.get_name())
        while not rospy.is_shutdown():
            rospy.Subscriber('/last_best', QuadFitness, fit_callback)
            if rospy.Time.now().secs == 20:
                state = 1
                pub.publish(state)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Node %s finished!", rospy.get_name())
