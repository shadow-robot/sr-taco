/**
 * @file   visual_servoing_moveit.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  7 09:38:03 2013
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
 * It will try to grasp the object if possible. We're using moveit.
 *
 * The principle we'll use is the following:
 *  - the callback simply updates the target
 *  - a fast loop is trying to get closer to the target iteratively
 *
 */

#ifndef _VISUAL_SERVOING_MOVEIT_HPP_
#define _VISUAL_SERVOING_MOVEIT_HPP_

#include <ros/ros.h>

#include <boost/smart_ptr.hpp>
#include <moveit/move_group_interface/move_group.h>

#include <sr_visual_servoing/VisualServoingFeedback.h>

namespace sr_taco
{
  class VisualServoing
  {
  public:
    VisualServoing();
    ~VisualServoing();

    ///Servo the arm to the current target, called periodically from Actionlib Server
    sr_visual_servoing::VisualServoingFeedback get_closer();

  protected:
    boost::shared_ptr<move_group_interface::MoveGroup> right_arm_;

    sr_visual_servoing::VisualServoingFeedback visual_servoing_feedback_;
  };
} //end namespace sr_taco

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
