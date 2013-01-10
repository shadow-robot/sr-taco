#!/usr/bin/env python
PKG = 'sr_taco_openni'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import unittest

from sensor_msgs.msg import PointCloud2, Image, CameraInfo 

class TestSrTacoOpenni(unittest.TestCase):
    def assertTopicPublishing(self, topic, topic_type, timeout = 3.0):
        """Test if a topic is publishing something of the correct type."""
        is_publishing = False;
        err = None
        try:
            rospy.wait_for_message(topic, topic_type, timeout = timeout)
        except Exception as err:
            pass
        else:
            is_publishing = True;
        self.assertTrue(is_publishing, topic + " is not publishing : " + str(err));
        return is_publishing

    def test_camera_info_publishers(self):
        self.assertTopicPublishing("/tacoSensor/unfoveated/depth/camera_info", CameraInfo)
        self.assertTopicPublishing("/tacoSensor/unfoveated/intensity/camera_info", CameraInfo)
        self.assertTopicPublishing("/tacoSensor/foveated/depth/camera_info", CameraInfo)
        self.assertTopicPublishing("/tacoSensor/foveated/intensity/camera_info", CameraInfo)
    
    def test_cloud_publishers(self):
        self.assertTopicPublishing("/tacoSensor/unfoveated/pointcloud2", PointCloud2)
        self.assertTopicPublishing("/tacoSensor/foveated/pointcloud2", PointCloud2)
        
    def test_image_publishers(self):
        self.assertTopicPublishing("/tacoSensor/unfoveated/depth/image", Image)
        self.assertTopicPublishing("/tacoSensor/unfoveated/intensity/image", Image)
        self.assertTopicPublishing("/tacoSensor/foveated/depth/image", Image)
        self.assertTopicPublishing("/tacoSensor/foveated/intensity/image", Image)
        # TODO: Test the images are the right size and depth.
            
    def test_saliency_publishers(self):
        self.assertTopicPublishing("/tacoSensor/saliency_map_spatial/image", Image)
 
if __name__ == '__main__':
    import rostest
    name = 'sr_taco_openni_test'
    rospy.init_node(name)
    rostest.rosrun(PKG, name, TestSrTacoOpenni)