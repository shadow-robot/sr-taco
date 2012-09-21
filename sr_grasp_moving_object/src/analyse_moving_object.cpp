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
  }

  AnalyseMovingObject::~AnalyseMovingObject()
  {}

  void AnalyseMovingObject::new_measurement(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    if( is_first_ )
    {
      //we ignore the first message for the twist as we don't have enough data
      is_first_ = false;
    }
    else
    {
      //compute twist from pose and last pose
      double dt = (pose->header.stamp - last_pose_.header.stamp).toSec();
      data_.twist.linear.x = pose->pose.position.x - last_pose_.pose.position.x;
      data_.twist.linear.x /= dt;

      data_.twist.linear.y = pose->pose.position.y - last_pose_.pose.position.y;
      data_.twist.linear.y /= dt;

      data_.twist.linear.z = pose->pose.position.z - last_pose_.pose.position.z;
      data_.twist.linear.z /= dt;

      //TODO: compute angular twist

      //compute the velocity
      data_.velocity = sr_utils::compute_distance(pose->pose.position, last_pose_.pose.position);
      data_.velocity /= dt;
    }

    data_.pose.pose.pose = pose->pose;

    last_pose_.header = pose->header;
    last_pose_.pose = pose->pose;

    model_->new_measurement(data_.pose.pose.pose.position.x, data_.pose.pose.pose.position.y, data_.pose.pose.pose.position.z);
  }

  AnalysedData AnalyseMovingObject::update_model()
  {
    geometry_msgs::PoseWithCovarianceStamped result = model_->update( data_.twist.linear.x,
                                                                      data_.twist.linear.y,
                                                                      data_.twist.linear.z );

    data_.pose = result;

    return data_;
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

    double refresh_freq = 0.0;
    nh_tilde_.param<double>("refresh_frequency", refresh_freq, 100.0);
    update_timer_ = nh_tilde_.createTimer(ros::Duration(1.0/refresh_freq), &AnalyseMovingObjectNode::update_model_, this);
  }

  AnalyseMovingObjectNode::~AnalyseMovingObjectNode()
  {}

  void AnalyseMovingObjectNode::new_measurement_cb_(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    analyser_.new_measurement(msg);
  }

  void AnalyseMovingObjectNode::update_model_(const ros::TimerEvent& e)
  {
    AnalysedData data = analyser_.update_model();

    //publish the odometry message
    odom_msg_.pose.pose = data.pose.pose.pose;
    odom_msg_.twist.twist = data.twist;
    odometry_pub_.publish(odom_msg_);

    //publish the markers
    visualization_msgs::Marker marker_arrow, marker_sphere;
    marker_arrow.header.frame_id = "/shadowarm_base";
    marker_arrow.header.stamp = ros::Time();
    marker_arrow.ns = "analyse_moving_object_a";
    marker_arrow.id = 0;
    marker_arrow.type = visualization_msgs::Marker::ARROW;
    marker_arrow.action = visualization_msgs::Marker::ADD;

    marker_arrow.points.resize(2);
    marker_arrow.points[0] = data.pose.pose.pose.position;
    marker_arrow.points[1] = data.pose.pose.pose.position;
    marker_arrow.points[1].x += 1.5*data.twist.linear.x;
    marker_arrow.points[1].y += 1.5*data.twist.linear.y;
    marker_arrow.points[1].z += 1.5*data.twist.linear.z;

    marker_arrow.scale.x = 0.05;
    if( fabs(data.velocity) < 0.03)
      marker_arrow.scale.y = 0.03;
    else
      marker_arrow.scale.y = 2.0*fabs(data.velocity);

    marker_arrow.color.a = 1.0;
    marker_arrow.color.r = 0.86;
    marker_arrow.color.g = 0.34;
    marker_arrow.color.b = 0.0;
    marker_pub_.publish(marker_arrow);

    marker_sphere.header.frame_id = "/shadowarm_base";
    marker_sphere.header.stamp = ros::Time();
    marker_sphere.ns = "analyse_moving_object_s";
    marker_sphere.id = 1;
    marker_sphere.type = visualization_msgs::Marker::SPHERE;
    marker_sphere.action = visualization_msgs::Marker::ADD;

    marker_sphere.pose = data.pose.pose.pose;

    //scale of the sphere based on the covariance
    marker_sphere.scale.x = data.pose.pose.covariance[0]/10.0;
    marker_sphere.scale.y = data.pose.pose.covariance[7]/10.0;
    marker_sphere.scale.z = data.pose.pose.covariance[14]/10.0;

    marker_sphere.color.a = 0.5;
    marker_sphere.color.r = 0.34;
    marker_sphere.color.g = 0.86;
    marker_sphere.color.b = 0.0;
    marker_pub_.publish(marker_sphere);
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
