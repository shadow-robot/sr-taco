#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <boost/smart_ptr.hpp>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "sr_taco_openni/common.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace sr_taco_openni {

using namespace std;
using namespace ros;

// Util class to hold the publishers.
// Will get one for foviated and one for unfoviated
class TacoOpenNIPubs {
    public:
        /** Setup the pubs for the type ("foveated","unfoveated") passed
         */
        TacoOpenNIPubs() {}
        TacoOpenNIPubs(NodeHandle, string);
        ~TacoOpenNIPubs() {}

        Publisher pointCloud;
        Publisher depthInfo;
        Publisher depthImage;
        Publisher intensityInfo;
        Publisher intensityImage;
};

class TacoOpenNINodelet : public nodelet::Nodelet {
    public:
        virtual ~TacoOpenNINodelet() {}

        virtual void onInit();

    private:
        NodeHandle nh, nh_home;

        // Namespace to find the camera in.
        string camera;

        // Leaf size (xyz) to downsample the camera feed
        double downsampling_grid_size_;
        double filter_z_min_, filter_z_max_;

        /// Cloud that came in on input, converted to PCL type
        CloudPtr input_cloud_;
        /// Cloud to work with, has been filtered and downsampled
        CloudPtr target_cloud_;

        // Stash all our subscribers to keep them alive.
        vector<Subscriber> subs;

        void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        void cameraInfoIn(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageIn(const sensor_msgs::Image::ConstPtr& msg);

        TacoOpenNIPubs foveated_;
        TacoOpenNIPubs unfoveated_;
};

} // sr_taco_openni::
