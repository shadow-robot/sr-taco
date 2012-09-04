/**
 * @file   visual_servoing.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep  4 14:57:11 2012
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
 * @brief This node is used to servo the arm and hand on a moving object.
 * It will try to grasp the object if possible.
 *
 * The principle we'll use is the following:
 *  - the callback simply updates the target
 *  - a fast loop is trying to get closer to the target iteratively
 *
 */

#ifndef _VISUAL_SERVOING_HPP_
#define _VISUAL_SERVOING_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace sr_taco
{
  class VisualServoing
  {
  public:
    VisualServoing();
    ~VisualServoing();

  protected:
    ros::NodeHandle nh_tilde_;

    ///subscribes to the odometry messages coming from the object analyser
    ros::Subscriber odom_sub_;
    void new_odom_cb_(const nav_msgs::OdometryConstPtr& msg);

    ///A timer used to servo the arm
    ros::Timer timer_;
    ///Timer callback, will servo the arm to the current target
    void get_closer_(const ros::TimerEvent& event);

    nav_msgs::Odometry target_;

    ///Set to true once we've received a msg
    bool msg_received_;
  };
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
