/**
 * @file   sr_grasp_moving_object.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 25 10:42:26 2012
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

#include <sr_grasp_moving_object/sr_grasp_moving_object.hpp>

#include <std_msgs/Float64.h>
#include <algorithm>

namespace sr_taco
{
  SrGraspMovingObject::SrGraspMovingObject()
  {
    init_grasp_and_pub_();

    servo_client_.reset( new actionlib::SimpleActionClient<sr_visual_servoing::VisualServoingAction>("/visual_servo", true) );

    ROS_INFO("Waiting for actionlib server on /visual_servo...");
    servo_client_->waitForServer();
    ROS_INFO("OK, actionlib server found, ready to start grasping.");
  }

  SrGraspMovingObject::~SrGraspMovingObject()
  {
    //TODO: not sure why it's not stopping when killing the node
    servo_client_->cancelAllGoals();
  }

  bool SrGraspMovingObject::grasp_object()
  {
    sr_visual_servoing::VisualServoingGoal goal;
    servo_client_->sendGoal(goal,
                            boost::bind(&SrGraspMovingObject::servo_done_, this, _1, _2),
                            actionlib::SimpleActionClient<sr_visual_servoing::VisualServoingAction>::SimpleActiveCallback(),
                            boost::bind(&SrGraspMovingObject::servo_feedback_, this, _1));

    return true;
  }


  void SrGraspMovingObject::servo_feedback_(const sr_visual_servoing::VisualServoingFeedbackConstPtr& feedback)
  {
    //close the grasp more or less depending on the distance to the object
    // may need to use another function to close faster when we get
    // really close to the object (so that the grasp stays opened longer).
    if( feedback->distance < 0.15 )
    {
      close_grasp_( 1.0 - std::min((feedback->distance - 0.1) / 0.2, 1.0) );
      if( feedback->distance < 0.12 )
      {
        close_grasp_( 1.0 );
        ROS_INFO_STREAM( "The object should have been caught." );
        servo_client_->cancelAllGoals();
      }
    }
    else
    {
      //completely open the hand otherwise
      close_grasp_(0.0);
    }
  }

  void SrGraspMovingObject::servo_done_(const actionlib::SimpleClientGoalState& state,
                                        const sr_visual_servoing::VisualServoingResultConstPtr& result)
  {
    ROS_DEBUG("Stopped servoing");
  }

  void SrGraspMovingObject::close_grasp_(double percentage)
  {
    ROS_DEBUG_STREAM("Closing at " << percentage*100 << "%");

    double opened, closed;
    std_msgs::Float64 target;
    for(unsigned int i=0; i < hand_publishers_.size(); ++i)
    {
      opened = opened_grasp_[i];
      closed = closed_grasp_[i];

      target.data = opened + (closed - opened) * percentage;

      hand_publishers_[i].publish(target);
    }
  }

  void SrGraspMovingObject::init_grasp_and_pub_()
  {
    //TODO: replace this ugly function by using a Cpp version of the grasp library.
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_ffj0_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_ffj3_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_ffj4_position_controller/command", 1));

    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_mfj0_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_mfj3_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_mfj4_position_controller/command", 1));

    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_rfj0_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_rfj3_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_rfj4_position_controller/command", 1));

    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_lfj0_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_lfj3_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_lfj4_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_lfj5_position_controller/command", 1));

    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_thj1_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_thj2_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_thj3_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_thj4_position_controller/command", 1));
    hand_publishers_.push_back(nh_.advertise<std_msgs::Float64>("/sh_thj5_position_controller/command", 1));

    ///////
    // OPENED
    //FF
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.0);

    //MF
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.0);

    //RF
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.0);

    //LF
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.04);
    opened_grasp_.push_back(0.0);
    opened_grasp_.push_back(0.0);

    //TH
    opened_grasp_.push_back(0.03);
    opened_grasp_.push_back(0.03);
    opened_grasp_.push_back(-0.05);
    opened_grasp_.push_back(-0.02);
    opened_grasp_.push_back(-0.09);

    ///////
    // CLOSED
    //FF
    closed_grasp_.push_back(2.4);
    closed_grasp_.push_back(1.2);
    closed_grasp_.push_back(0.0);

    //MF
    closed_grasp_.push_back(2.4);
    closed_grasp_.push_back(1.2);
    closed_grasp_.push_back(0.0);

    //RF
    closed_grasp_.push_back(2.4);
    closed_grasp_.push_back(1.2);
    closed_grasp_.push_back(0.0);

    //LF
    closed_grasp_.push_back(2.4);
    closed_grasp_.push_back(1.2);
    closed_grasp_.push_back(0.0);
    closed_grasp_.push_back(0.0);

    //TH
    closed_grasp_.push_back(0.9);
    closed_grasp_.push_back(0.4);
    closed_grasp_.push_back(0.1);
    closed_grasp_.push_back(0.9);
    closed_grasp_.push_back(-0.1);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sr_grasp_moving_object");

  sr_taco::SrGraspMovingObject sr_grasp;

  sr_grasp.grasp_object();

  ros::spin();

  return 0;
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
