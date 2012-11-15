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

        // Just use depth not depth_registed as we don't need RGB/Depth alignment
        subs.push_back( nh.subscribe(camera + "/depth/points",
               1, &TacoOpenNI::pclIn, this) );
        subs.push_back( nh.subscribe(camera + "/depth/camera_info",
               1, &TacoOpenNI::cameraInfoIn, this) );
        subs.push_back( nh.subscribe(camera + "/depth/image",
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
    }
        
    void TacoOpenNI::pclIn(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        CloudPtr input_cloud(new Cloud);
        pcl::fromROSMsg(*cloud, *input_cloud);

        // Downsample the point cloud to speed up processing
        target_cloud_.reset(new Cloud);
        pcl::VoxelGrid<PointType> grid;
        grid.setLeafSize(downsampling_grid_size_, downsampling_grid_size_, downsampling_grid_size_);
        grid.setInputCloud(input_cloud);
        grid.filter(*target_cloud_);

        // No foveation yet so pub the same cloud twice
        sensor_msgs::PointCloud2 out_cloud;
        pcl::toROSMsg(*target_cloud_, out_cloud);
        unfoveated.pointCloud.publish(out_cloud);
        foveated.pointCloud.publish(out_cloud);

        calculateSaliencyMap();
        saliency_map_spatial_pub.publish(saliency_map_spatial);
    }
    
    void TacoOpenNI::calculateSaliencyMap()
    {
        // Pull out the interesting clusters
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        cluster_segmentor.setInputCloud(target_cloud_);
        cluster_segmentor.extract(clusters);

        // Setup empty map
        saliency_map_spatial->header.stamp = ros::Time::now();
        for(size_t i = 0; i < saliency_map_spatial->data.size(); ++i)
            saliency_map_spatial->data[i] = 0;

        // TODO: Merge all the points from the clusters into the range image

        // Try to convert current target_cloud to a depth map as a test.
        // XXX: We get an image but it is not right :(
        float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
        float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.00;
        float minRange = 0.0f;
        int borderSize = 1;
        pcl::RangeImage rangeImage;
        rangeImage.createFromPointCloud(*target_cloud_, angularResolution, maxAngleWidth, maxAngleHeight,
                                        sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
        //ROS_INFO_STREAM(rangeImage);

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
