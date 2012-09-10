#include "sr_taco_openni/sr_taco_openni.h"

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

    TacoOpenNI::TacoOpenNI()
        : nh_home("~"),
        foveated("foveated"),
        unfoveated("unfoveated")
    {
        nh_home.param<string>("camera", camera, "camera");

        // Just use depth not depth_registed as we don't need RGB/Depth alignment
        pclSub = nh.subscribe(camera + "/depth/points",
               1, &TacoOpenNI::pclIn, this);
        cameraInfoSub = nh.subscribe(camera + "/depth/camera_info",
               1, &TacoOpenNI::cameraInfoIn, this);
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

} // sr_taco_openni::
