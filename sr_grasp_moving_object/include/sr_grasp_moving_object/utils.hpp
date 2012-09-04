/**
 * @file   analyse_moving_object.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep  4 09:44:09 2012
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  This node is used to analyse a moving object. It publishes important information
 * (speed / acceleration) and also publishes useful markers for displaying in rviz.
 *
 */

#ifndef _UTILS_GRASP_MOVING_OBJECT_HPP_
#define _UTILS_GRASP_MOVING_OBJECT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "tf/transform_datatypes.h"

namespace sr_utils
{
  static inline double compute_distance(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    tf::Vector3 pt1, pt2;
    pointMsgToTF(a, pt1);
    pointMsgToTF(b, pt2);

    double distance = pt1.distance(pt2);

    return distance;
  };
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
