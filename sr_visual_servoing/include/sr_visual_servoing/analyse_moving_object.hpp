/**
 * @file   analyse_moving_object.hpp
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

#ifndef _ANALYSE_MOVING_OBJECT_HPP_
#define _ANALYSE_MOVING_OBJECT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <sr_visual_servoing/prediction_model.hpp>

#include <tf/transform_listener.h>

namespace sr_taco
{
  struct AnalysedData
  {
    geometry_msgs::PoseWithCovarianceStamped pose;
    geometry_msgs::Twist twist;

    double velocity;
  };

  class AnalyseMovingObject
  {
  public:
    AnalyseMovingObject();
    ~AnalyseMovingObject();

    void new_measurement(const geometry_msgs::PoseStampedConstPtr& pose);

    /**
     * regularly updates the model:
     *  - disperses the object position
     *  @return the current object pose estimation
     */
    AnalysedData update_model();

  protected:
    geometry_msgs::PoseStamped last_pose_;
    bool is_first_;

    AnalysedData data_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<PredictionModel> model_;
    std::string frame_id_;
  };

  class AnalyseMovingObjectNode
  {
  public:
    AnalyseMovingObjectNode();
    virtual ~AnalyseMovingObjectNode();

  protected:
    ros::NodeHandle nh_tilde_;
    AnalyseMovingObject analyser_;
    ros::Subscriber moving_object_sub_;

    ///publishes an odometry message (pose and twist)
    ros::Publisher odometry_pub_;
    nav_msgs::Odometry odom_msg_;

    ///publishes rviz markers
    ros::Publisher marker_pub_;

    ros::Timer update_timer_;
    /**
     * regularly updates the model:
     *  - disperses the object position
     *  - returns the current object pose estimation
     */
    void update_model_(const ros::TimerEvent& e);

    void new_measurement_cb_(const geometry_msgs::PoseStampedConstPtr& msg);
  };
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
