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

#include <boost/smart_ptr.hpp>
#include <map>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace sr_taco
{
  class VisualServoing
  {
  public:
    VisualServoing();
    ~VisualServoing();

  protected:
    ros::NodeHandle nh_tilde_;

    ///subscribes to the joint_states, updating the vector of current_positions_
    ros::Subscriber joint_states_sub_;
    std::map<std::string, double> current_positions_;
    std::vector<std::string> joint_names_;
    void joint_states_cb_(const sensor_msgs::JointStateConstPtr& msg);

    ///subscribes to the odometry messages coming from the object analyser
    ros::Subscriber odom_sub_;
    void new_odom_cb_(const nav_msgs::OdometryConstPtr& msg);

    ///A timer used to servo the arm
    ros::Timer timer_;
    ///Timer callback, will servo the arm to the current target
    void get_closer_(const ros::TimerEvent& event);

    ///The increment we'll use for computing the best solution
    static const double epsilon_;
    ///The vector containing -epsilon, 0, +epsilon
    std::vector<double> epsilons_;

    /**
     * Generate different solutions aroung the current position
     *  and keep the one closest to object position + twist
     *  (the object is moving toward this point)
     *
     *  Updates the robot_targets_ vector.
     */
    void generate_best_solution_();

    KDL::Tree kdl_arm_tree_;
    KDL::Chain kdl_arm_chain_;
    KDL::JntArray kdl_joint_positions_;
    KDL::Frame kdl_cartesian_position_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

    /**
     * Send the current robot_targets_ to the robot.
     */
    void send_robot_targets_();
    /**
     * map containing the publishers for sending targets to the robot
     */
    std::map<std::string, ros::Publisher> robot_publishers_;
    void init_robot_publishers_();

    ///The latest object position and twist
    nav_msgs::Odometry tracked_object_;

    ///The targets we'll send to the robot
    std::vector<double> robot_targets_;
    ///The names of the joints we're sending targets to
    std::vector<std::string> target_names_;

    ///Set to true once we've received a msg
    bool object_msg_received_;
    bool joint_states_msg_received_;
  };
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
