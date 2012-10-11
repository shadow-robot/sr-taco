/**
 * @file   sr_grasp_moving_object.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 25 10:38:51 2012
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
 * @brief
 *
 */

#ifndef _SR_GRASP_MOVING_OBJECT_HPP_
#define _SR_GRASP_MOVING_OBJECT_HPP_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>

#include <sr_visual_servoing/VisualServoingAction.h>
#include <sr_visual_servoing/VisualServoingFeedback.h>
#include <actionlib/client/simple_action_client.h>

namespace sr_taco
{
  class SrGraspMovingObject
  {
  public:
    SrGraspMovingObject();
    virtual ~SrGraspMovingObject();

    bool grasp_object();
  protected:
    ros::NodeHandle nh_;

    boost::shared_ptr<actionlib::SimpleActionClient<sr_visual_servoing::VisualServoingAction> > servo_client_;

    void servo_feedback_(const sr_visual_servoing::VisualServoingFeedbackConstPtr& feedback);
    void servo_done_(const actionlib::SimpleClientGoalState& state,
                     const sr_visual_servoing::VisualServoingResultConstPtr& result);

    void close_grasp_(double percentage);

    std::vector<ros::Publisher> hand_publishers_;
    std::vector<double> opened_grasp_;
    std::vector<double> closed_grasp_;
    void init_grasp_and_pub_();
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
