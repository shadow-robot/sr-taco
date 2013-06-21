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
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>

namespace sr_taco
{
  VisualServoing::VisualServoing()
  {

    robot_publishers_["WRJ1"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_wrj1_mixed_position_velocity_controller/command", 1);
    robot_publishers_["WRJ2"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_wrj2_mixed_position_velocity_controller/command", 1);

    // for the arm
    robot_publishers_["ShoulderJRotate"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_sr_position_controller/command", 1);
    robot_publishers_["ShoulderJSwing"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_ss_position_controller/command", 1);
    robot_publishers_["ElbowJRotate"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_er_position_controller/command", 1);
    robot_publishers_["ElbowJSwing"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_es_position_controller/command", 1);


    right_arm_.reset(new move_group_interface::MoveGroup("right_arm"));

    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    /* Get a shared pointer to the model */
    kinematic_model_ = robot_model_loader.getModel();
    ROS_ERROR("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    joint_state_group_ = kinematic_state_->getJointStateGroup("right_arm");

    planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));

    //plan for the start pose
    joint_state_group_->setToDefaultState("default");
    joint_state_group_->getVariableValues(joints_target_);
    joint_names_ = joint_state_group_->getJointModelGroup()->getJointModelNames();

    //@todo: add traj controllers to the hand
    // right_hand_.reset(new move_group_interface::MoveGroup("right_hand"));
    // //open the hand
    // right_hand_->setNamedTarget("open");
    // right_hand_->move();

    // the distance is set to -1 until we've started tracking the object
    visual_servoing_feedback_.distance = -1.0;

    //initialises subscribers
    odom_sub_ = nh_tilde_.subscribe("/analyse_moving_object/odometry", 2, &VisualServoing::new_odom_cb_, this);
    right_arm_timer_ = nh_tilde_.createTimer(ros::Duration(0.5), &VisualServoing::move_arm_, this);
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
    //geometry_msgs::PoseStamped current_pose = right_arm_->getCurrentPose();

    tf::poseMsgToEigen(tracked_object_.pose.pose, end_effector_target_);
    bool found_ik = joint_state_group_->setFromIK(end_effector_target_, 10, 0.1);
    if( found_ik )
      ROS_INFO("IK Found");
  }

  void VisualServoing::move_arm_(const ros::TimerEvent&)
  {
    ROS_ERROR("move arm");
    //stop last step then move
    //right_arm_->stop();

    std_msgs::Float64 msg;
    for (size_t i=0; i < joints_target_.size(); ++i)
    {
      ROS_ERROR_STREAM(" joint state[" << joint_names_[i] <<"]: " << joints_target_[i]);

      msg.data = joints_target_[i];
      robot_publishers_[joint_names_[i]].publish( msg );
    }
  }

  void VisualServoing::new_odom_cb_(const nav_msgs::OdometryConstPtr& msg)
  {
    //update the target
    tracked_object_.pose = msg->pose;
    tracked_object_.twist = msg->twist;
    tracked_object_.child_frame_id = msg->child_frame_id;
    tracked_object_.header = msg->header;
    tracked_object_.header.frame_id = "shadowarm_base";

    //Fixed orientation of the wrist
    tracked_object_.pose.pose.orientation.w = 0.5599;
    tracked_object_.pose.pose.orientation.x = 0.4320;
    tracked_object_.pose.pose.orientation.y = 0.4320;
    tracked_object_.pose.pose.orientation.z = 0.5599;

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
