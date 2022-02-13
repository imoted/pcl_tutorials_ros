#!/usr/bin/python

import rospy

if __name__ == '__main__':
    rospy.init_node('test_node_python')
    rate = rospy.Rate(0.5)
    test_pub = rospy.Publisher('/test_topic', String, queue_size=10)
    val = 0
    for i in range(10):
        test_pub.publish(str(val))
        rospy.loginfo('node is running')
        rate.sleep()
        val += 1
