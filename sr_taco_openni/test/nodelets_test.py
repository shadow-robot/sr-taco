#!/usr/bin/env python
PKG = 'sr_taco_openni'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import unittest

from nodelet.srv import *

class TestClusterSegment(unittest.TestCase):
    def test_listed(self):
        """
        Test that the nodelets in the .test file have been loaded into the
        nodelet managet by listing them.
        """
        rospy.wait_for_service('/nodelet_manager/list',timeout=10)
        list = rospy.ServiceProxy('/nodelet_manager/list', NodeletList)
        resp = list()
        nodelets = resp.nodelets
        self.assertEqual(sorted(nodelets), sorted(['/taco_openni', '/cluster_segment']),
                         'Nodelet cluster_segment not is listed. Got: %s'%nodelets)
 
if __name__ == '__main__':
    import rostest
    name = 'cluster_segment_test'
    rospy.init_node(name)
    rostest.rosrun(PKG, name, TestClusterSegment)
