#pragma once

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace sr_taco_openni {

using namespace std;
using namespace ros;

class TacoOpenNI {
    public:
        TacoOpenNI(); 
        ~TacoOpenNI() {}

    private:
        NodeHandle nh, nh_home;

        // Namespace to find the camera in.
        string camera;

        Subscriber pclSub;
        void pclIn(const sensor_msgs::PointCloud2::ConstPtr& msg);

        Publisher unfoveatedPointCloudPub;
        Publisher foveatedPointCloudPub;
};

} // sr_taco_openni::
