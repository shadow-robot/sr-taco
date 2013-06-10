/**
 * @file   visual_servoing_moveit.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  7 09:37:10 2013
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
 * @brief  Visual servoing using moveit
 *
 *
 */

#include <sr_visual_servoing/visual_servoing_moveit.hpp>

namespace sr_taco
{
  VisualServoing::VisualServoing()
  {
    right_arm_.reset(new move_group_interface::MoveGroup("right_arm"));
    //move to the start pose
    right_arm_->setNamedTarget("default");
    right_arm_->asyncMove();

    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    /* Get a shared pointer to the model */
    kinematic_model_ = robot_model_loader.getModel();
    ROS_ERROR("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));

    //@todo: add traj controllers to the hand
    // right_hand_.reset(new move_group_interface::MoveGroup("right_hand"));
    // //open the hand
    // right_hand_->setNamedTarget("open");
    // right_hand_->move();

    // the distance is set to -1 until we've started tracking the object
    visual_servoing_feedback_.distance = -1.0;

    //initialises subscribers
    odom_sub_ = nh_tilde_.subscribe("/analyse_moving_object/odometry", 2, &VisualServoing::new_odom_cb_, this);
  }

  VisualServoing::~VisualServoing()
  {}

  sr_visual_servoing::VisualServoingFeedback VisualServoing::get_closer()
  {
    if( object_msg_received_ )
    {
      //generate different solutions aroung the current position
      // and keep the one closest to object position + twist
      // (the object is moving toward this point)
      generate_best_solution_();

      update_feedback_();
    }
    return visual_servoing_feedback_;
  }

  void VisualServoing::generate_best_solution_()
  {
    //get current arm pose
    geometry_msgs::PoseStamped current_pose = right_arm_->getCurrentPose();

    current_pose.pose.position.x += 0.01;
    current_pose.pose.position.y += 0.01;
    current_pose.pose.position.z += 0.01;

    right_arm_->setPoseTarget(current_pose.pose);

    //plan the move
    right_arm_->setPositionTarget( tracked_object_.pose.pose.position.x,
                                   tracked_object_.pose.pose.position.y,
                                   tracked_object_.pose.pose.position.z );

    ROS_ERROR_STREAM("pose: " << right_arm_->getPoseTarget() );

    //move
    right_arm_->asyncMove();
  }

  void VisualServoing::new_odom_cb_(const nav_msgs::OdometryConstPtr& msg)
  {
    //update the target
    tracked_object_.pose = msg->pose;
    tracked_object_.twist = msg->twist;
    tracked_object_.child_frame_id = msg->child_frame_id;
    tracked_object_.header = msg->header;
    tracked_object_.header.frame_id = "shadowarm_base";
    object_msg_received_ = true;
  }


  void VisualServoing::update_feedback_()
  {
    //update the feedback with the tracked object pose;
    visual_servoing_feedback_.object_pose.position.x = tracked_object_.pose.pose.position.x;
    visual_servoing_feedback_.object_pose.position.y = tracked_object_.pose.pose.position.y;
    visual_servoing_feedback_.object_pose.position.z = tracked_object_.pose.pose.position.z;

    visual_servoing_feedback_.object_pose.orientation.x = tracked_object_.pose.pose.orientation.x;
    visual_servoing_feedback_.object_pose.orientation.y = tracked_object_.pose.pose.orientation.y;
    visual_servoing_feedback_.object_pose.orientation.z = tracked_object_.pose.pose.orientation.z;
    visual_servoing_feedback_.object_pose.orientation.w = tracked_object_.pose.pose.orientation.w;

    //@todo: udpate rest of the feedback
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
