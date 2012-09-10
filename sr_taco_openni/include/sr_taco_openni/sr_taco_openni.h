#pragma once

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

namespace sr_taco_openni {

using namespace std;
using namespace ros;

// Util class to hold the publishers.
// Will get one for foviated and one for unfoviated
class TacoOpenNIPubs {
    public:
        /** Setup the pubs for the type ("foveated","unfoveated") passed
         */
        TacoOpenNIPubs(string);
        ~TacoOpenNIPubs() {}

        Publisher pointCloud;
        Publisher depthInfo;
        Publisher depthImage;
        Publisher intensityInfo;
        Publisher intensityImage;
};

class TacoOpenNI {
    public:
        TacoOpenNI(); 
        ~TacoOpenNI() {}

    private:
        NodeHandle nh, nh_home;

        // Namespace to find the camera in.
        string camera;

        vector<Subscriber> subs;
        void pclIn(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void cameraInfoIn(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageIn(const sensor_msgs::Image::ConstPtr& msg);

        TacoOpenNIPubs foveated;
        TacoOpenNIPubs unfoveated;
};

} // sr_taco_openni::
