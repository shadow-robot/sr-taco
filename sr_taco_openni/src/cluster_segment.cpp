/*
 * cluster_segment.cpp
 *
 *  Created on: 14 Dec 2012
 *      Author: mda
 */

#include <pluginlib/class_list_macros.h>
#include <sr_taco_openni/attention_manager.h>
#include <ros/ros.h>

namespace sr_taco_openni
{
  /**
   * @brief AttentionManager that extracts clusters after removing planar surfaces.
   */
  class ClusterSegment : public AttentionManager
  {
    public:
      ClusterSegment(){}

    private:
      virtual void onInit()
      {
        NODELET_INFO("Starting ClusterSegment attention manager.");
      }

  };
}

PLUGINLIB_DECLARE_CLASS(sr_taco_openni,cluster_segment,sr_taco_openni::ClusterSegment,nodelet::Nodelet)
