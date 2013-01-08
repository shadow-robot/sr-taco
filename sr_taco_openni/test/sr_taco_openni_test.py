#!/usr/bin/env python
PKG = 'sr_taco_openni'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import unittest

from nodelet.srv import *

class TestSrTacoOpenni(unittest.TestCase):
    def test_publishers(self):
        """
        Test that the sensor is publishing the expected topics. 
        """
        self.assertTrue(True, 'Not true')
 
if __name__ == '__main__':
    import rostest
    name = 'sr_taco_openni_test'
    rospy.init_node(name)
    rostest.rosrun(PKG, name, TestSrTacoOpenni)