/*
 * attention_manager.h
 *
 *  Created on: 14 Dec 2012
 *      Author: mda
 */

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "sr_taco_openni/common.h"
#include <sensor_msgs/image_encodings.h>

namespace sr_taco_openni
{
  /**
   * \brief Base class for attention managers, nodelets that generate saliency maps.
   */
  class AttentionManager : public nodelet::Nodelet
  {
    public:
      AttentionManager(){}

    protected:
      /**
       * @brief Construct and return a shared pointer to a new, blank SaliencyMap (sensor_msgs::Image).
       * @return A boost shared pointer to a SaliencyMap.
       *
       * The Map will have the correct encoding (MONO8) set along with the width, height and step,
       * with the data vector resized to match. The timestamp is set to now.
       *
       * Sub classes should use this in their callbacks to get a new pointer and add their map then
       * publish the pointer, not changing it after (i.e. don't use the pointer again). This is for
       * maximum nodelet goodness.
       */
      SaliencyMapPtr newSaliencyMap()
      {
        SaliencyMapPtr map = SaliencyMapPtr(new SaliencyMap());
        map->width = taco_width;
        map->height = taco_height;
        //we're using mono8 as the saliency map can contain labels (change if needed)
        map->encoding = sensor_msgs::image_encodings::MONO8;
        map->step = taco_width; // * 1 as using mono
        map->data.resize( taco_height * map->step );
        map->header.stamp = ros::Time::now();
        for(size_t i = 0; i < map->data.size(); ++i)
            map->data[i] = 0;
        return map;
      }
  };
}
