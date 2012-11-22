#include "sr_taco_openni/sr_taco_openni.h"

#include "sr_pcl_tracking/cluster_segmentor.h"

#include <sensor_msgs/image_encodings.h>

#include <pcl/range_image/range_image.h>

namespace sr_taco_openni {

    using namespace sr_pcl_tracking;

// TacoOpenNIPubs -------------------------------------------------------------

    TacoOpenNIPubs::TacoOpenNIPubs(string type) {
        NodeHandle nh("~");
        pointCloud = nh.advertise<sensor_msgs::PointCloud2>( type+"/pointcloud2", 5);
        depthInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/depth/camera_info", 5);
        depthImage = nh.advertise<sensor_msgs::Image>( type+"/depth/image", 5);
        intensityInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/intensity/camera_info", 5);
        intensityImage = nh.advertise<sensor_msgs::Image>( type+"/intensity/image", 5);
    }


// TacoOpenNI ----------------------------------------------------------------

    const unsigned int TacoOpenNI::taco_width  = 640;
    const unsigned int TacoOpenNI::taco_height = 480;

    TacoOpenNI::TacoOpenNI()
        : nh_home("~")
        , foveated("foveated")
        , unfoveated("unfoveated")
    {
        nh_home.param<string>("camera", camera, "camera");
        nh_home.param<double>("downsampling_grid_size", downsampling_grid_size_, 0.01);
        saliency_map_spatial = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());

        // Setup a callback with the point cloud and camera info in sync.
        // Needed for generating depth images (ie saliency maps) from clouds.
        string depth = camera + "/depth_registered";
        pointcloud_sub_ = PointCloudSubPtr( new PointCloudSub(nh, depth + "/points", 1));
        camerainfo_sub_ = CameraInfoSubPtr(new CameraInfoSub(nh, depth + "/camera_info", 1));
        pointcloud_sync_ = CloudSyncPtr( new CloudSync(*pointcloud_sub_, *camerainfo_sub_, 10) );
        pointcloud_sync_->registerCallback( boost::bind(&TacoOpenNI::cloudCb, this, _1, _2) );

        subs.push_back( nh.subscribe(camera + "/depth_registered/image",
               1, &TacoOpenNI::depthImageIn, this) );
        
        saliency_map_spatial_pub = nh_home.advertise<sensor_msgs::Image>(
                "saliency_map_spatial/image", 5);
        saliency_map_spatial->width = taco_width;
        saliency_map_spatial->height = taco_height;
        //we're using mono8 as the saliency map can contain labels (change if
        //needed)
        saliency_map_spatial->encoding = sensor_msgs::image_encodings::MONO8;
        saliency_map_spatial->step = taco_width; // * 1 as using mono
        saliency_map_spatial->data.resize( taco_height * saliency_map_spatial->step );

        clusters_pub_ = nh_home.advertise<Cloud>("clusters/points", 5);
    }

    void TacoOpenNI::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                       const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        CloudPtr input_cloud(new Cloud);
        pcl::fromROSMsg(*cloud, *input_cloud);
        ROS_INFO_STREAM_ONCE("input_cloud: " << *input_cloud);

        // Downsample the point cloud to speed up processing
        target_cloud_.reset(new Cloud);
        pcl::VoxelGrid<PointType> grid;
        grid.setInputCloud(input_cloud);
        grid.setLeafSize(downsampling_grid_size_, downsampling_grid_size_, downsampling_grid_size_);
        grid.filter(*target_cloud_);
        ROS_INFO_STREAM_ONCE("target_cloud: " << *target_cloud_);

        // No foveation yet so pub the same cloud twice
        sensor_msgs::PointCloud2 out_cloud;
        pcl::toROSMsg(*target_cloud_, out_cloud);
        unfoveated.pointCloud.publish(out_cloud);
        foveated.pointCloud.publish(out_cloud);

        calculateSaliencyMap();
        saliency_map_spatial_pub.publish(saliency_map_spatial);

        // Re-publish the camera info
        cameraInfoIn(info);
    }

    void TacoOpenNI::calculateSaliencyMap()
    {
        // Pull out the interesting clusters
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        cluster_segmentor.setInputCloud(target_cloud_);
        cluster_segmentor.extract(clusters);

        // Merge all the clusters into a single cloud
        Cloud::Ptr all_clusters(new Cloud);
        for (size_t i=0; i<clusters.size(); ++i)
        {
            Cloud cloud = *(clusters[i]);
            *all_clusters += cloud;
        }
        ROS_INFO_STREAM_ONCE("all_clusters: " << all_clusters);
        all_clusters->header.stamp = ros::Time::now();
        all_clusters->header.frame_id = target_cloud_->header.frame_id;
        clusters_pub_.publish(all_clusters);

        // Setup empty map
        saliency_map_spatial->header.stamp = ros::Time::now();
        for(size_t i = 0; i < saliency_map_spatial->data.size(); ++i)
            saliency_map_spatial->data[i] = 0;

        // Try to convert cluster cloud to a depth map
        // XXX: We get an image but it is not right :(
        float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
        float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.00;
        float minRange = 0.0f;
        int borderSize = 0;
        pcl::RangeImage rangeImage;
        rangeImage.createFromPointCloud(*all_clusters, angularResolution, maxAngleWidth, maxAngleHeight,
                                        sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
        ROS_INFO_STREAM_ONCE("rangeImage:" << rangeImage);

        // Convert range image to ros msg
        for (uint32_t y=0; y<rangeImage.height; ++y) {
            for (uint32_t x=0; x<rangeImage.width; ++x) {
                if ( rangeImage.isValid(x,y) ) continue;
                int n = y*saliency_map_spatial->width + x;
                //ROS_INFO("xy:%i,%i n:%i", x, y, (int)n);
                saliency_map_spatial->data[n] = 255;
            }
        }

    }

    void TacoOpenNI::cameraInfoIn(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        foveated.depthInfo.publish(msg);
        foveated.intensityInfo.publish(msg);
        unfoveated.depthInfo.publish(msg);
        unfoveated.intensityInfo.publish(msg);
    }
        
    void TacoOpenNI::depthImageIn(const sensor_msgs::Image::ConstPtr& msg) {
        foveated.depthImage.publish(msg);
        unfoveated.depthImage.publish(msg);
    }

} // sr_taco_openni::
