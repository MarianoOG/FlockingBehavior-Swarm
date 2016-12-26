#!/usr/bin/env python
import message_filters, rospy, sys
from swarm.msg import QuadStamped, QuadState

def multiple_callback(*args):
    global pub, quad, n
    
    # Publish:
    try:
        for i in range(n):
            quad[i].z = 1    
            pub[i].publish(quad[i])
    except rospy.ROSException:
        pass

if __name__ == '__main__':
    rospy.init_node('reynolds', anonymous=True)
    argv = sys.argv
    rospy.myargv(argv)
    n = int(argv[1])

    sub = []
    pub = []
    quad = []
    for i in range(n):        
        sub.append(message_filters.Subscriber('/uav' + str(i) + '/quad_state', QuadState))
        pub.append(rospy.Publisher('/uav' + str(i) + '/des_pos', QuadStamped, queue_size=n))
        quad.append(QuadStamped())
        quad[i].header.frame_id = 'world'
        xy = rospy.get_param('/uav' + str(i))
        quad[i].x = xy['x']
        quad[i].y = xy['y']
        quad[i].z = 0
        quad[i].yaw = 0

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
            xy = rospy.get_param('/uav' + str(i))
            quad[i].x = xy
            quad[i].y = xy['x']
            quad[i].z = xy['y']
            quad[i].yaw = 0
            pub[i].publish(quad[i])
        rospy.loginfo("End of node")
