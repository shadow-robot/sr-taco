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

namespace sr_taco_openni
{
  using namespace std;
  using namespace ros;
  using namespace sr_pcl_tracking;

  /**
   * @brief AttentionManager that extracts clusters after removing planar surfaces.
   */
  class ClusterSegment : public AttentionManager
  {
    public:
      typedef message_filters::Subscriber<Cloud> PointCloudSub;
      typedef typename boost::shared_ptr<PointCloudSub> PointCloudSubPtr;

      typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSub;
      typedef typename boost::shared_ptr<CameraInfoSub> CameraInfoSubPtr;

      typedef message_filters::TimeSynchronizer<Cloud, sensor_msgs::CameraInfo> CloudSync;
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
        nh_ = getNodeHandle();

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

      void cloudCb(const CloudConstPtr& cloud, const sensor_msgs::CameraInfo::ConstPtr& info)
      {
        NODELET_INFO_STREAM_ONCE("cloudCb:" << *cloud << "info: " << *info);

        string frame_id = cloud->header.frame_id;

        double min_cluster_size, max_cluster_size;
        // Default max to 1/10th of input cloud size
        double def_max = (double)cloud->size() * 0.1;
        nh_.param<double>("attention/segment_clusters/min_cluster_size", min_cluster_size, 50.0);
        nh_.param<double>("attention/segment_clusters/max_cluster_size", max_cluster_size, def_max);

        // Pull out the interesting clusters
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        cluster_segmentor.setInputCloud(cloud);
        cluster_segmentor.setMinClusterSize(min_cluster_size);
        cluster_segmentor.setMaxClusterSize(max_cluster_size);
        cluster_segmentor.extract(clusters);

        // Merge all the clusters into a single cloud
        Cloud::Ptr all_clusters(new Cloud);
        for (size_t i=0; i<clusters.size(); ++i)
        {
            Cloud cloud = *(clusters[i]);
            *all_clusters += cloud;
        }
        ROS_INFO_STREAM_ONCE("all_clusters: " << all_clusters);

        // Publish the cluster cloud for debug use
        all_clusters->header.stamp = ros::Time::now();
        all_clusters->header.frame_id = frame_id;
        clusters_pub_.publish(all_clusters);

        // Setup empty map
        saliency_map_spatial_->header.stamp = ros::Time::now();
        for(size_t i = 0; i < saliency_map_spatial_->data.size(); ++i)
            saliency_map_spatial_->data[i] = 0;

        // Convert saliency msg to cv image
        IplImage* image = NULL;
        try {
            image = bridge_.imgMsgToCv(saliency_map_spatial_, "mono8");
        }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
        }

        cam_model_.fromCameraInfo(info);

        // Draw the salient points as a little circles. This should fill in the gaps
        // from down sampling.
        // XXX : Now we are a nodelet how can we do this?
        //int radius = std::ceil(input_cloud_->size()/target_cloud_->size());
        int radius = 3;
        PointType pt;
        BOOST_FOREACH( pt, all_clusters->points )
        {
            // Point should be in the camera frame. Project that onto the image.
            cv::Point3d pt_cv(pt.x, pt.y, pt.z);
            cv::Point2d uv;
            cam_model_.project3dToPixel(pt_cv, uv);
            cvCircle(image, uv, radius, CV_RGB(255,255,255), -1);
        }

        // Convert the cv image back to msg and publish
        bridge_.cvToImgMsg(image, "mono8");
        saliency_map_spatial_pub_.publish(saliency_map_spatial_);
      }

  };
}

PLUGINLIB_DECLARE_CLASS(sr_taco_openni,cluster_segment,sr_taco_openni::ClusterSegment,nodelet::Nodelet)
