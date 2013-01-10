#include "sr_taco_openni/taco_openni_nodelet.h"
#include "sr_taco_openni/common.h"
#include <pluginlib/class_list_macros.h>

#include "sr_pcl_tracking/cluster_segmentor.h"

#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <pcl/range_image/range_image.h>

PLUGINLIB_DECLARE_CLASS(sr_taco_openni, taco_openni_nodelet, sr_taco_openni::TacoOpenNINodelet, nodelet::Nodelet)

namespace sr_taco_openni {

    using namespace sr_pcl_tracking;

// TacoOpenNIPubs -------------------------------------------------------------

    TacoOpenNIPubs::TacoOpenNIPubs(NodeHandle nh, string type) {
        pointCloud = nh.advertise<sensor_msgs::PointCloud2>( type+"/pointcloud2", 5);
        depthInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/depth/camera_info", 5);
        depthImage = nh.advertise<sensor_msgs::Image>( type+"/depth/image", 5);
        intensityInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/intensity/camera_info", 5);
        intensityImage = nh.advertise<sensor_msgs::Image>( type+"/intensity/image", 5);
    }


// TacoOpenNINodelet ----------------------------------------------------------------

    void TacoOpenNINodelet::onInit() {
        NODELET_INFO("Starting main Taco nodelet");

        nh_home = getPrivateNodeHandle();
        foveated_ = TacoOpenNIPubs(nh_home, "foveated");
        unfoveated_ = TacoOpenNIPubs(nh_home, "unfoveated");

        nh_home.param<string>("camera", camera, "camera");
        nh_home.param<double>("downsampling_grid_size", downsampling_grid_size_, 0.01);
        nh_home.param<double>("filter_z_min", filter_z_min_, 0.0);
        nh_home.param<double>("filter_z_max", filter_z_max_, 10.0);

        saliency_map_spatial_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());

        // Setup a callback with the point cloud and camera info in sync.
        // Needed for generating depth images (ie saliency maps) from clouds.
        string depth = camera + "/depth";
        pointcloud_sub_ = PointCloudSubPtr( new PointCloudSub(nh, depth + "/points", 1));
        camerainfo_sub_ = CameraInfoSubPtr(new CameraInfoSub(nh, depth + "/camera_info", 1));
        pointcloud_sync_ = CloudSyncPtr( new CloudSync(*pointcloud_sub_, *camerainfo_sub_, 10) );
        pointcloud_sync_->registerCallback( boost::bind(&TacoOpenNINodelet::cloudCb, this, _1, _2) );

        subs.push_back( nh.subscribe(depth + "/image",
               1, &TacoOpenNINodelet::depthImageIn, this) );

        saliency_map_spatial_pub_ = nh_home.advertise<sensor_msgs::Image>(
                "saliency_map_spatial/image", 5);
        saliency_map_spatial_->width = taco_width;
        saliency_map_spatial_->height = taco_height;
        //we're using mono8 as the saliency map can contain labels (change if
        //needed)
        saliency_map_spatial_->encoding = sensor_msgs::image_encodings::MONO8;
        saliency_map_spatial_->step = taco_width; // * 1 as using mono
        saliency_map_spatial_->data.resize( taco_height * saliency_map_spatial_->step );

        clusters_pub_ = nh_home.advertise<Cloud>("clusters/points", 5);
    }

    void TacoOpenNINodelet::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                       const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        input_cloud_.reset(new Cloud);
        CloudPtr tmp_cloud(new Cloud);
        pcl::fromROSMsg(*cloud, *input_cloud_);
        ROS_INFO_STREAM_ONCE("input_cloud: " << *input_cloud_);

        // Do some z filtering, to reduce number points and focus on foreground
        pcl::PassThrough<PointType> pass;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_z_min_, filter_z_max_);
        pass.setKeepOrganized (false);
        pass.setInputCloud (input_cloud_);
        pass.filter (*tmp_cloud);

        // Downsample the point cloud to speed up processing
        target_cloud_.reset(new Cloud);
        pcl::VoxelGrid<PointType> grid;
        grid.setInputCloud(tmp_cloud);
        grid.setLeafSize(downsampling_grid_size_, downsampling_grid_size_, downsampling_grid_size_);
        grid.filter(*target_cloud_);

        ROS_INFO_STREAM_ONCE("target_cloud: " << *target_cloud_);

        // No foveation yet so pub the same cloud twice
        sensor_msgs::PointCloud2Ptr out_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*target_cloud_, *out_cloud);
        unfoveated_.pointCloud.publish(out_cloud);
        foveated_.pointCloud.publish(out_cloud);

        //calculateSaliencyMap(info);
        //saliency_map_spatial_pub_.publish(saliency_map_spatial_);

        // Re-publish the camera info
        cameraInfoIn(info);
    }

    void TacoOpenNINodelet::calculateSaliencyMap(const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        string frame_id = target_cloud_->header.frame_id;

        double min_cluster_size, max_cluster_size;
        // Default max to 1/10th of input cloud size
        double def_max = (double)target_cloud_->size() * 0.1;
        nh_home.param<double>("attention/segment_clusters/min_cluster_size", min_cluster_size, 50.0);
        nh_home.param<double>("attention/segment_clusters/max_cluster_size", max_cluster_size, def_max);

        // Pull out the interesting clusters
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        cluster_segmentor.setInputCloud(target_cloud_);
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
        // from downsampling.
        int radius = std::ceil(input_cloud_->size()/target_cloud_->size());
        PointType pt;
        BOOST_FOREACH( pt, all_clusters->points )
        {
            // Point should be in the camera frame. Project that onto the image.
            cv::Point3d pt_cv(pt.x, pt.y, pt.z);
            cv::Point2d uv;
            cam_model_.project3dToPixel(pt_cv, uv);
            cvCircle(image, uv, radius, CV_RGB(255,255,255), -1);
        }

        // Convert the cv image back to msg
        bridge_.cvToImgMsg(image, "mono8");
    }

    void TacoOpenNINodelet::cameraInfoIn(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        foveated_.depthInfo.publish(msg);
        foveated_.intensityInfo.publish(msg);
        unfoveated_.depthInfo.publish(msg);
        unfoveated_.intensityInfo.publish(msg);
    }
        
    void TacoOpenNINodelet::depthImageIn(const sensor_msgs::Image::ConstPtr& msg) {
        foveated_.depthImage.publish(msg);
        unfoveated_.depthImage.publish(msg);

        // Just pub a blank image for now, not sure what we need here and we
        // don't use the intensity image atm.
        sensor_msgs::Image::Ptr blank_img(new sensor_msgs::Image);
        blank_img->width = taco_width;
        blank_img->height = taco_height;
        blank_img->encoding = sensor_msgs::image_encodings::MONO8;
        blank_img->step = taco_width; // * 1 as using mono
        blank_img->data.resize( taco_height * blank_img->step );
        foveated_.intensityImage.publish(blank_img);
        unfoveated_.intensityImage.publish(blank_img);
    }

} // sr_taco_openni::
