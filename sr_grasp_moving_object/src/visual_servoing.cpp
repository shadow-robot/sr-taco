/**
 * @file   visual_servoing.cpp
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

#include <sr_grasp_moving_object/visual_servoing.hpp>
#include <sr_grasp_moving_object/utils.hpp>

namespace sr_taco
{
  VisualServoing::VisualServoing()
    : nh_tilde_("~"), msg_received_(false)
  {
    odom_sub_ = nh_tilde_.subscribe("/analyse_moving_object/odometry", 2, &VisualServoing::new_odom_cb_, this);
    timer_ = nh_tilde_.createTimer(ros::Rate(100.0), &VisualServoing::get_closer_, this);
  }

  VisualServoing::~VisualServoing()
  {
  }

  void VisualServoing::new_odom_cb_(const nav_msgs::OdometryConstPtr& msg)
  {
    msg_received_ = true;

    //update the target
    target_.pose = msg->pose;
    target_.twist = msg->twist;
    target_.child_frame_id = msg->child_frame_id;
    target_.header = msg->header;
  }

  ///Timer callback, will servo the arm to the current target
  void VisualServoing::get_closer_(const ros::TimerEvent& event)
  {
    //don't do anything if we haven't received the first target
    if(!msg_received_)
      return;

    //get theoretic grasping point position (palm + offsets)

    //compute a target that'll get us closer to the target

    //send it to the joints
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visual_servoing");

  sr_taco::VisualServoing visual_servo;
  ros::spin();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
