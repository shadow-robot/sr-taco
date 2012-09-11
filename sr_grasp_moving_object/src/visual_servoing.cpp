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
#include <ros/package.h>

#include <std_msgs/Float64.h>

//#include <omp.h>

namespace sr_taco
{
  VisualServoing::VisualServoing()
    : nh_tilde_("~"), object_msg_received_(false),
      joint_states_msg_received_(false)
  {
    init_openrave_();

    init_robot_publishers_();

    //initialises subscribers and timer
    joint_states_sub_ = nh_tilde_.subscribe("/gazebo/joint_states", 2, &VisualServoing::joint_states_cb_, this);

    odom_sub_ = nh_tilde_.subscribe("/analyse_moving_object/odometry", 2, &VisualServoing::new_odom_cb_, this);
    timer_ = nh_tilde_.createTimer(ros::Rate(100.0), &VisualServoing::get_closer_, this);
  }

  VisualServoing::~VisualServoing()
  {
    OpenRAVE::RaveDestroy();
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
    // for all the joints from arm_base to palm.
    //TODO: use openmp for loop (https://computing.llnl.gov/tutorials/openMP/#DO)

    OpenRAVE::Transform trans;
    /*trans.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(OpenRAVE::RaveRandomFloat() - 0.5,
                                                                       OpenRAVE::RaveRandomFloat() - 0.5,
                                                                       OpenRAVE::RaveRandomFloat() - 0.5));
    trans.trans = OpenRAVE::Vector(OpenRAVE::RaveRandomFloat() - 0.5,
                                   OpenRAVE::RaveRandomFloat() - 0.5,
                                   OpenRAVE::RaveRandomFloat() - 0.5) * 2;
                                   */
    trans.rot.w = 0.317;
    trans.rot.x = 0.519;
    trans.rot.y = 0.215;
    trans.rot.z = 0.764;

    trans.trans.x = 0.608;
    trans.trans.y = -0.127;
    trans.trans.z = 0.3;

    std::vector<OpenRAVE::dReal> ik_solution;
    if( rave_manipulator_->FindIKSolution(OpenRAVE::IkParameterization(trans), ik_solution, OpenRAVE::IKFO_IgnoreEndEffectorCollisions) )
    {
      std::stringstream ss;
      for(size_t i = 0; i < ik_solution.size(); ++i)
      {
        ss << ik_solution[i] << " ";
      }
      ROS_WARN_STREAM("The solution is: " << ss.str());
    }
    else
    {
      ROS_ERROR_STREAM("No solution found for " << trans);
    }

    /*
    ROS_ERROR("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
              "Returns the ik solutions given the transformation of the end effector specified by\n"
              "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
              "There are %d free parameters that have to be specified.\n\n",
              IKFAST_NAMESPACE::GetNumFreeParameters());
     */

    /*
    ik_fast::IkSolutionList<IkReal> solutions;
    std::vector<ik_fast::IkReal> vfree(ik_fast::GetNumFreeParameters());
    ik_fast::IkReal eerot[9],eetrans[3];

    tf::Matrix3x3 test(tracked_object_.pose.pose.orientation);

    for (unsigned int i=0; i < 9; ++i)
      eerot[i] = test.m_el[i];

    eetrans[0] = tracked_object_.pose.pose.position.x;
    eetrans[1] = tracked_object_.pose.pose.position.y;
    eetrans[2] = tracked_object_.pose.pose.position.z;

    for(std::size_t i = 0; i < vfree.size(); ++i)
      vfree[i] = 0.0;

    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess )
    {
      ROS_WARN("Failed to get ik solution");
      return;
    }

    ROS_ERROR("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<ik_fast::IkReal> solvalues(ik_fast::GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
    {
      const ik_fast::IkSolutionBase<ik_fast::IkReal>& sol = solutions.GetSolution(i);
      ROS_ERROR("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
      std::vector<ik_fast::IkReal> vsolfree(sol.GetFree().size());
      sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
      for( std::size_t j = 0; j < solvalues.size(); ++j)
        ROS_ERROR("%.15f, ", solvalues[j]);
      ROS_ERROR("\n");
    }
    */
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

  void VisualServoing::init_openrave_()
  {
    OpenRAVE::RaveInitialize(true);
    rave_env_ = OpenRAVE::RaveCreateEnvironment();

    //lock the environment to prevent changes
    OpenRAVE::EnvironmentMutex::scoped_lock lock(rave_env_->GetMutex());
    //load the scene from the xml file
    // the file is in sr_grasp_moving_object/openrave/arm_and_hand_motor.xml
    std::string path = ros::package::getPath("sr_grasp_moving_object");
    path += "/openrave/arm_and_hand_motor.xml";
    rave_robot_ = rave_env_->ReadRobotXMLFile(path);
    if( !rave_robot_ )
    {
      rave_env_->Destroy();
      ROS_FATAL("Couldn't create the robot for OpenRAVE");
      ROS_BREAK();
    }

    rave_env_->Add(rave_robot_);
    rave_ikfast_ = OpenRAVE::RaveCreateModule(rave_env_, "ikfast");
    rave_env_->Add(rave_ikfast_, true, "");
    //rave_robot_->SetActiveManipulator("arm_and_hand_motor");

    std::stringstream ssin,ssout;
    ssin << "LoadIKFastSolver " << rave_robot_->GetName() << " " << OpenRAVE::IKP_Transform6D;
    rave_manipulator_ = rave_robot_->GetActiveManipulator();
    if( !rave_ikfast_->SendCommand(ssout, ssin) )
    {
      ROS_FATAL_STREAM( "failed to load the ik solver for " << rave_robot_->GetName() << ssout.str() );
      rave_env_->Destroy();
      ROS_BREAK();
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
    target_names_.push_back("ElbowJSwing");
    robot_targets_.push_back(0.0);
    target_names_.push_back("ElbowJRotate");
    robot_targets_.push_back(0.0);
    target_names_.push_back("WRJ2");
    robot_targets_.push_back(0.0);
    target_names_.push_back("WRJ1");
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
