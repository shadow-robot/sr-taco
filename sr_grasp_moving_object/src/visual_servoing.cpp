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

#include <std_msgs/Float64.h>

//#include <omp.h>

namespace sr_taco
{
  //using 4 degrees increments
  const double VisualServoing::epsilon_ = 0.035;

  VisualServoing::VisualServoing()
    : nh_tilde_("~"), object_msg_received_(false),
      joint_states_msg_received_(false)
  {
    epsilons_.push_back(- epsilon_);
    epsilons_.push_back(0.0);
    epsilons_.push_back(epsilon_);

    init_robot_publishers_();

    //initialises the forward kinematics solver
    // and other kdl stuffs
    if( !kdl_parser::treeFromParam("/robot_description", kdl_arm_tree_) )
    {
      ROS_FATAL_STREAM("Failed to construct the kdl tree from the robot_description.");
      ROS_BREAK();
    }
    kdl_arm_tree_.getChain("shadowarm_trunk", "palm", kdl_arm_chain_);

    fksolver_.reset( new KDL::ChainFkSolverPos_recursive(kdl_arm_chain_) );
    unsigned int nj = kdl_arm_chain_.getNrOfJoints();
    kdl_joint_positions_ = KDL::JntArray(nj);

    //initialises subscribers and timer
    joint_states_sub_ = nh_tilde_.subscribe("/joint_states", 2, &VisualServoing::joint_states_cb_, this);

    odom_sub_ = nh_tilde_.subscribe("/analyse_moving_object/odometry", 2, &VisualServoing::new_odom_cb_, this);
    timer_ = nh_tilde_.createTimer(ros::Rate(100.0), &VisualServoing::get_closer_, this);
  }

  VisualServoing::~VisualServoing()
  {
  }

  void VisualServoing::new_odom_cb_(const nav_msgs::OdometryConstPtr& msg)
  {
    //update the target
    tracked_object_.pose = msg->pose;
    tracked_object_.twist = msg->twist;
    tracked_object_.child_frame_id = msg->child_frame_id;
    tracked_object_.header = msg->header;

    object_msg_received_ = true;
  }

  ///Timer callback, will servo the arm to the current tracked_object
  void VisualServoing::get_closer_(const ros::TimerEvent& event)
  {
    //don't do anything if we haven't received the first tracked_object
    // or the first joint_states msg.
    if(!object_msg_received_)
      return;
    if(!joint_states_msg_received_)
      return;

    //generate different solutions aroung the current position
    // and keep the one closest to object position + twist
    // (the object is moving toward this point)
    generate_best_solution_();

    //send it to the joints
    send_robot_targets_();
  }

  void VisualServoing::generate_best_solution_()
  {
    //TODO: generate the best solution using fk
    // combines current_pos, current_pos + epsilon and current_pos - epsilon
    // for all the joints from arm_base to palm.
    //TODO: use openmp for loop (https://computing.llnl.gov/tutorials/openMP/#DO)
    bool kinematic_status;
    geometry_msgs::Pose pose;
    double best_distance = -1.0;
    double distance = 0.0;

    double eps_sr, eps_ss, eps_es, eps_er, eps_wrj1, eps_wrj2;

    int indx = 0;
    for(unsigned int i=0; i < epsilons_.size(); ++i)
    {
      eps_sr = epsilons_[i];
      for(unsigned int j=0; j < epsilons_.size(); ++j)
      {
        eps_ss = epsilons_[j];
        for(unsigned int k=0; k < epsilons_.size(); ++k)
        {
          eps_er = epsilons_[k];
          for(unsigned int l=0; l < epsilons_.size(); ++l)
          {
            eps_es = epsilons_[l];
            for(unsigned int m=0; m < epsilons_.size(); ++m)
            {
              eps_wrj1 = epsilons_[m];
              for(unsigned int n=0; n < epsilons_.size(); ++n)
              {
                ++indx;

                eps_wrj2 = epsilons_[n];

                kdl_joint_positions_(0) = current_positions_["ShoulderJRotate"] + eps_sr;
                kdl_joint_positions_(1) = current_positions_["ShoulderJSwing"] + eps_ss;
                kdl_joint_positions_(2) = current_positions_["ElbowJSwing"] + eps_es;
                kdl_joint_positions_(3) = current_positions_["ElbowJRotate"] + eps_er;
                //joint[4] in the solution is ignored: it's the static link between arm and hand
                kdl_joint_positions_(4) = 0.0;
                kdl_joint_positions_(5) = current_positions_["WRJ2"] + eps_wrj2;
                kdl_joint_positions_(6) = current_positions_["WRJ1"] + eps_wrj1;

                kinematic_status = fksolver_->JntToCart(kdl_joint_positions_, kdl_cartesian_position_);

                if( kinematic_status < 0 )
                {
                  ROS_ERROR_STREAM("Error computing fk for the robot.");
                  return;
                }

                pose.position.x = kdl_cartesian_position_.p.x();
                pose.position.y = kdl_cartesian_position_.p.y();
                pose.position.z = kdl_cartesian_position_.p.z();

                //TODO, compute position + twist in object callback
                distance = sr_utils::compute_distance(pose.position, tracked_object_.pose.pose.position);

                if( best_distance == -1.0 )
                {
                  best_distance = distance;

                  //same order as target_names_ NOT A MISTAKE!!
                  robot_targets_[0] = current_positions_["ShoulderJRotate"] + eps_sr;
                  robot_targets_[1] = current_positions_["ShoulderJSwing"] + eps_ss;
                  robot_targets_[2] = current_positions_["ElbowJRotate"] + eps_er;
                  robot_targets_[3] = current_positions_["ElbowJSwing"] + eps_es;
                  robot_targets_[4] = current_positions_["WRJ1"] + eps_wrj1;
                  robot_targets_[5] = current_positions_["WRJ2"] + eps_wrj2;
                }
                else
                {
                  if( distance < best_distance )
                  {
                    best_distance = distance;

                    //same order as target_names_ NOT A MISTAKE!!
                    robot_targets_[0] = current_positions_["ShoulderJRotate"] + eps_sr;
                    robot_targets_[1] = current_positions_["ShoulderJSwing"] + eps_ss;
                    robot_targets_[2] = current_positions_["ElbowJRotate"] + eps_er;
                    robot_targets_[3] = current_positions_["ElbowJSwing"] + eps_es;
                    robot_targets_[4] = current_positions_["WRJ1"] + eps_wrj1;
                    robot_targets_[5] = current_positions_["WRJ2"] + eps_wrj2;
                  }
                }
              }//end wrj2
            } //end wrj1
          }//end es
        }//end er
      }//end ss
    }//end sr

    ROS_INFO_STREAM("computing: ["<< robot_targets_[0] << ", "
                    << robot_targets_[1] << ", "
                    << robot_targets_[2] << ", "
                    << robot_targets_[3] << ", "
                    << robot_targets_[4] << ", "
                    << robot_targets_[5] << ", "
                    << "]  -> " <<
                    " (best distance = " << best_distance << ") id = " << indx );
  }

  void VisualServoing::send_robot_targets_()
  {
    if( joint_states_msg_received_ )
    {
      ROS_ASSERT(target_names_.size() == robot_targets_.size());
      std_msgs::Float64 msg;
      for (unsigned int i = 0; i < target_names_.size(); ++i)
      {
        msg.data = robot_targets_[i];
        robot_publishers_[joint_names_[i]].publish( msg );
      }
    }
  }

  void VisualServoing::joint_states_cb_(const sensor_msgs::JointStateConstPtr& msg)
  {
    if( !joint_states_msg_received_ )
    {
      joint_names_ = msg->name;

      joint_states_msg_received_ = true;
    }

    ROS_ASSERT( msg->position.size() == joint_names_.size() );
    for(unsigned int i=0; i < joint_names_.size(); ++i)
    {
      current_positions_[ joint_names_[i] ] = msg->position[i];
    }
  }

  void VisualServoing::init_robot_publishers_()
  {
    //Those are the targets to which we'll publish for
    // moving the arm (we also include the wrist in the arm)
    target_names_.push_back("ShoulderJRotate");
    robot_targets_.push_back(0.0);
    target_names_.push_back("ShoulderJSwing");
    robot_targets_.push_back(0.0);
    target_names_.push_back("ElbowJRotate");
    robot_targets_.push_back(0.0);
    target_names_.push_back("EblowJSwing");
    robot_targets_.push_back(0.0);
    target_names_.push_back("WRJ1");
    robot_targets_.push_back(0.0);
    target_names_.push_back("WRJ2");
    robot_targets_.push_back(0.0);

    //initialises the map of publishers
    //TODO: really ugly, replace by a service call etc...
    //for the hand
    robot_publishers_["FFJ0"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_ffj0_mixed_position_velocity_controller/command", 1);
    robot_publishers_["FFJ3"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_ffj3_mixed_position_velocity_controller/command", 1);
    robot_publishers_["FFJ4"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_ffj4_mixed_position_velocity_controller/command", 1);

    robot_publishers_["MFJ0"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_mfj0_mixed_position_velocity_controller/command", 1);
    robot_publishers_["MFJ3"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_mfj3_mixed_position_velocity_controller/command", 1);
    robot_publishers_["MFJ4"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_mfj4_mixed_position_velocity_controller/command", 1);

    robot_publishers_["RFJ0"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_rfj0_mixed_position_velocity_controller/command", 1);
    robot_publishers_["RFJ3"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_rfj3_mixed_position_velocity_controller/command", 1);
    robot_publishers_["RFJ4"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_rfj4_mixed_position_velocity_controller/command", 1);

    robot_publishers_["LFJ0"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_lfj0_mixed_position_velocity_controller/command", 1);
    robot_publishers_["LFJ3"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_lfj3_mixed_position_velocity_controller/command", 1);
    robot_publishers_["LFJ4"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_lfj4_mixed_position_velocity_controller/command", 1);
    robot_publishers_["LFJ5"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_lfj5_mixed_position_velocity_controller/command", 1);

    robot_publishers_["THJ1"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_thj1_mixed_position_velocity_controller/command", 1);
    robot_publishers_["THJ2"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_thj2_mixed_position_velocity_controller/command", 1);
    robot_publishers_["THJ3"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_thj3_mixed_position_velocity_controller/command", 1);
    robot_publishers_["THJ4"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_thj4_mixed_position_velocity_controller/command", 1);
    robot_publishers_["THJ5"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_thj5_mixed_position_velocity_controller/command", 1);

    robot_publishers_["WRJ1"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_wrj1_mixed_position_velocity_controller/command", 1);
    robot_publishers_["WRJ2"] = nh_tilde_.advertise<std_msgs::Float64>("/sh_wrj2_mixed_position_velocity_controller/command", 1);

    // for the arm
    robot_publishers_["ShoulderJRotate"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_sr_position_controller/command", 1);
    robot_publishers_["ShoulderJSwing"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_ss_position_controller/command", 1);
    robot_publishers_["ElbowJRotate"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_er_position_controller/command", 1);
    robot_publishers_["ElbowJSwing"] = nh_tilde_.advertise<std_msgs::Float64>("/sa_es_position_controller/command", 1);
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
