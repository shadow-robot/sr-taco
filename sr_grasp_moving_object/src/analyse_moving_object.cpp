/**
 * @file   analyse_moving_object.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep  4 09:44:09 2012
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
 * @brief  This node is used to analyse a moving object. It publishes important information
 * (speed / acceleration) and also publishes useful markers for displaying in rviz.
 *
 */

#include <sr_grasp_moving_object/analyse_moving_object.hpp>


namespace sr_taco
{
  AnalyseMovingObject::AnalyseMovingObject()
    : is_first_(true)
  {}

  AnalyseMovingObject::~AnalyseMovingObject()
  {}

  AnalysedData AnalyseMovingObject::new_measurement(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    AnalysedData data;
    if( is_first_ )
    {
      //we ignore the first message for the twist as we don't have enough data
      is_first_ = false;
    }
    else
    {
      //compute twist from pose and last pose
      data.twist.linear.x = pose->pose.position.x - last_pose_.pose.position.x;
      data.twist.linear.x /= (pose->header.stamp - last_pose_.header.stamp).toSec();

      data.twist.linear.y = pose->pose.position.y - last_pose_.pose.position.y;
      data.twist.linear.y /= (pose->header.stamp - last_pose_.header.stamp).toSec();

      data.twist.linear.z = pose->pose.position.z - last_pose_.pose.position.z;
      data.twist.linear.z /= (pose->header.stamp - last_pose_.header.stamp).toSec();

      //TODO: compute angular twist
    }

    data.pose = pose->pose;

    last_pose_.header = pose->header;
    last_pose_.pose = pose->pose;
    return data;
  }

////////////////
// AnalyseMovingObjectNode
  AnalyseMovingObjectNode::AnalyseMovingObjectNode()
    : nh_tilde_("~")
  {
    odom_msg_.header.frame_id = "/base_link";
    odom_msg_.child_frame_id = "/tracked_object";
    odometry_pub_ = nh_tilde_.advertise<nav_msgs::Odometry>("odometry", 2);

    moving_object_sub_ = nh_tilde_.subscribe("/object/position", 2, &AnalyseMovingObjectNode::new_measurement_cb_, this);
  }

  AnalyseMovingObjectNode::~AnalyseMovingObjectNode()
  {}

  void AnalyseMovingObjectNode::new_measurement_cb_(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    AnalysedData data = analyser_.new_measurement(msg);

    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.pose.pose = data.pose;
    odom_msg_.twist.twist = data.twist;
    odometry_pub_.publish(odom_msg_);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "analyse_moving_object");

  sr_taco::AnalyseMovingObjectNode amon;
  ros::spin();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
