/**
 * @brief Common constants etc for the sr_taco_openni package.
 * @author: mda
 * @date 10 Jan 2013
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

namespace sr_taco_openni
{
  /** Constant width  for our point cloud and images such as saliency.
   * Should match the camera we are faking the taco with, not the actual
   * taco device.
   * e.g. 640 works for a Kinect.
   */
  static const unsigned int taco_width = 640;

  /** Constant height for our point cloud and images such as saliency.
   * Should match the camera we are faking the taco with, not the actual
   * taco device.
   * e.g. 480 works for a Kinect.
   */
  static const unsigned int taco_height = 480;

  /** Define a common PCL PointType for use across the nodelets.
   */
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

}

