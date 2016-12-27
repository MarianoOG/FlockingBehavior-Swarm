#!/usr/bin/env python
import message_filters, rospy, sys
from swarm.msg import QuadState

def multiple_callback(*args):
    global pub, quad, n
    
    # Publish:
    try:
        for i in range(n):
            quad[i] = args[i]
            quad[i].header.stamp = rospy.Time.now()
            pub[i].publish(quad[i])
    except rospy.ROSException:
        pass

if __name__ == '__main__':
    rospy.init_node('micro_pso', anonymous=True)
    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])

    sub = []
    pub = []
    quad = []
    for i in range(n):        
        sub.append(message_filters.Subscriber('/uav' + str(i) + '/quad_state', QuadState))
        pub.append(rospy.Publisher('/uav' + str(i) + '/next_generation', QuadState, queue_size=10))
        quad.append(QuadState())
        quad[i].header.frame_id = 'world'

    ts = message_filters.ApproximateTimeSynchronizer(sub, n, 0.01)

    try:
        rospy.sleep(1)
        ts.registerCallback(multiple_callback)
        rospy.loginfo("Start spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        for i in range(n):
            quad[i].header.stamp = rospy.Time.now()
            pub[i].publish(quad[i])
        rospy.loginfo("End of node")
