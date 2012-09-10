#pragma once

#include "ros/ros.h"
#include <boost/smart_ptr.hpp>
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

        boost::shared_ptr<sensor_msgs::Image> saliency_map_spatial;
        Publisher saliency_map_spatial_pub;
        
        //const width/height for our point cloud and image.
        //Should match the camera we are faking the taco with, not the actual
        //taco cam. e.g. 640*480 works for a kinect.
        static const unsigned int taco_width;
        static const unsigned int taco_height;
};

} // sr_taco_openni::
