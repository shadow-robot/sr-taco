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
#include <sr_grasp_moving_object/utils.hpp>

namespace sr_taco
{
  AnalyseMovingObject::AnalyseMovingObject()
    : is_first_(true)
  {
    model_.reset( new PredictionModel() );

    update_timer_ = nh_.createTimer(ros::Duration(0.01), &AnalyseMovingObject::update_model_, this);
  }

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
      double dt = (pose->header.stamp - last_pose_.header.stamp).toSec();
      data.twist.linear.x = pose->pose.position.x - last_pose_.pose.position.x;
      data.twist.linear.x /= dt;

      data.twist.linear.y = pose->pose.position.y - last_pose_.pose.position.y;
      data.twist.linear.y /= dt;

      data.twist.linear.z = pose->pose.position.z - last_pose_.pose.position.z;
      data.twist.linear.z /= dt;

      //TODO: compute angular twist

      //compute the velocity
      data.velocity = sr_utils::compute_distance(pose->pose.position, last_pose_.pose.position);
      data.velocity /= dt;
    }

    data.pose = pose->pose;

    last_pose_.header = pose->header;
    last_pose_.pose = pose->pose;

    model_->new_measurement(data.pose.position.x, data.pose.position.y, data.pose.position.z);

    return data;
  }

  void AnalyseMovingObject::update_model_(const ros::TimerEvent& e)
  {
    model_->update();
  }

////////////////
// AnalyseMovingObjectNode
  AnalyseMovingObjectNode::AnalyseMovingObjectNode()
    : nh_tilde_("~")
  {
    odom_msg_.header.frame_id = "/shadowarm_base";
    odom_msg_.child_frame_id = "/tracked_object";
    odometry_pub_ = nh_tilde_.advertise<nav_msgs::Odometry>("odometry", 2);

    marker_pub_ = nh_tilde_.advertise<visualization_msgs::Marker>("visualisation", 0);

    moving_object_sub_ = nh_tilde_.subscribe("/object/position", 2, &AnalyseMovingObjectNode::new_measurement_cb_, this);
  }

  AnalyseMovingObjectNode::~AnalyseMovingObjectNode()
  {}

  void AnalyseMovingObjectNode::new_measurement_cb_(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    AnalysedData data = analyser_.new_measurement(msg);

    //publish the odometry message
    odom_msg_.pose.pose = data.pose;
    odom_msg_.twist.twist = data.twist;
    odometry_pub_.publish(odom_msg_);

    //publish the markers
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/shadowarm_base";
    marker.header.stamp = ros::Time();
    marker.ns = "analyse_moving_object";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.resize(2);
    marker.points[0] = data.pose.position;
    marker.points[1] = data.pose.position;
    marker.points[1].x += 1.5*data.twist.linear.x;
    marker.points[1].y += 1.5*data.twist.linear.y;
    marker.points[1].z += 1.5*data.twist.linear.z;

    marker.scale.x = 0.05;
    if( fabs(data.velocity) < 0.03)
      marker.scale.y = 0.03;
    else
      marker.scale.y = 2.0*fabs(data.velocity);

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
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
