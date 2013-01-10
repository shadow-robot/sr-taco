/*
 * cluster_segment.cpp
 *
 *  Created on: 14 Dec 2012
 *      Author: mda
 */

#include <sr_taco_openni/attention_manager.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "sr_pcl_tracking/cluster_segmentor.h"
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <image_geometry/pinhole_camera_model.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace sr_taco_openni
{
  using namespace std;
  using namespace ros;

  /**
   * @brief AttentionManager that extracts clusters after removing planar surfaces.
   */
  class ClusterSegment : public AttentionManager
  {
    public:
      // PCL
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;

      typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSub;
      typedef typename boost::shared_ptr<PointCloudSub> PointCloudSubPtr;

      typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSub;
      typedef typename boost::shared_ptr<CameraInfoSub> CameraInfoSubPtr;

      typedef message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> CloudSync;
      typedef typename boost::shared_ptr<CloudSync> CloudSyncPtr;
      typedef typename boost::shared_ptr<const CloudSync> CloudSyncConstPtr;

      ClusterSegment(){}

    private:
      NodeHandle nh_;

      PointCloudSubPtr pointcloud_sub_;
      CameraInfoSubPtr camerainfo_sub_;
      CloudSyncPtr pointcloud_sync_;

      sensor_msgs::CvBridge bridge_;
      image_geometry::PinholeCameraModel cam_model_;
      boost::shared_ptr<sensor_msgs::Image> saliency_map_spatial_;
      Publisher saliency_map_spatial_pub_;
      Publisher clusters_pub_;

      virtual void onInit()
      {
        NODELET_INFO("Starting ClusterSegment attention manager.");
        nh_ = getPrivateNodeHandle();

        saliency_map_spatial_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());

        // Setup a callback with the point cloud and camera info in sync.
        // Needed for generating depth images (ie saliency maps) from clouds.
        string in = "/tacoSensor/unfoveated";
        pointcloud_sub_ = PointCloudSubPtr(new PointCloudSub(nh_, in + "/pointcloud2", 1));
        camerainfo_sub_ = CameraInfoSubPtr(new CameraInfoSub(nh_, in + "/depth/camera_info", 1));
        pointcloud_sync_ = CloudSyncPtr(new CloudSync(*pointcloud_sub_, *camerainfo_sub_, 10));
        pointcloud_sync_->registerCallback( boost::bind(&ClusterSegment::cloudCb, this, _1, _2) );

        saliency_map_spatial_pub_ = nh_.advertise<sensor_msgs::Image>(
                "saliency_map_spatial/image", 5);
        saliency_map_spatial_->width = taco_width;
        saliency_map_spatial_->height = taco_height;
        //we're using mono8 as the saliency map can contain labels (change if needed)
        saliency_map_spatial_->encoding = sensor_msgs::image_encodings::MONO8;
        saliency_map_spatial_->step = taco_width; // * 1 as using mono
        saliency_map_spatial_->data.resize( taco_height * saliency_map_spatial_->step );

        clusters_pub_ = nh_.advertise<Cloud>("clusters/points", 5);
      }

      void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                     const sensor_msgs::CameraInfo::ConstPtr& info)
      {
        NODELET_INFO_STREAM_ONCE("cloudCb:" << cloud << "info: " << info);
      }

  };
}

PLUGINLIB_DECLARE_CLASS(sr_taco_openni,cluster_segment,sr_taco_openni::ClusterSegment,nodelet::Nodelet)
