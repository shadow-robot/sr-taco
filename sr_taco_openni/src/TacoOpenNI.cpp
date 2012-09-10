#include "sr_taco_openni/sr_taco_openni.h"
#include <sensor_msgs/image_encodings.h>

namespace sr_taco_openni {

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
        : nh_home("~"),
        foveated("foveated"),
        unfoveated("unfoveated")
    {
        nh_home.param<string>("camera", camera, "camera");
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
        
    void TacoOpenNI::pclIn(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        foveated.pointCloud.publish(msg);
        unfoveated.pointCloud.publish(msg);
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

        // XXX: For now when ever we pub a depth image also send out a blank
        // saliency image
        saliency_map_spatial->header.stamp = ros::Time::now();
        for(size_t i = 0; i < saliency_map_spatial->data.size(); ++i) {
            saliency_map_spatial->data[i] = 0;
        }
        saliency_map_spatial_pub.publish(saliency_map_spatial);
    }

} // sr_taco_openni::
