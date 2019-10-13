#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class FilteringNodeTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('pcl_tutorials_ros', nodes, 'node does not exist')

if __name__ == '__main__':
    # Wait for node startup
    time.sleep(3)
    rospy.init_node('example_filtering_')
    rostest.rosrun('pcl_tutorials_ros', 'test_filtering_node', FilteringNodeTest)
