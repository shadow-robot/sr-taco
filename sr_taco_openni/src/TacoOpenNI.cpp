#include "sr_taco_openni/sr_taco_openni.h"

namespace sr_taco_openni {

    TacoOpenNI::TacoOpenNI() : nh_home("~") {
        nh_home.param<string>("camera", camera, "camera");

        // Just use depth not depth_registed as we don't need RGB/Depth alignment
        pclSub = nh.subscribe(camera + "/depth/points", 1, &TacoOpenNI::pclIn, this);

        foveatedPointCloudPub = nh_home.advertise<sensor_msgs::PointCloud2>(
                "foveated/pointcloud2", 5);
        unfoveatedPointCloudPub = nh_home.advertise<sensor_msgs::PointCloud2>(
                "unfoveated/pointcloud2", 5);
    }
        
    void TacoOpenNI::pclIn(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        foveatedPointCloudPub.publish(msg);
        unfoveatedPointCloudPub.publish(msg);
    }

} // sr_taco_openni::
