#pragma once

#include "ros/ros.h"
#include <boost/smart_ptr.hpp>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
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
        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::Ptr CloudPtr;
        typedef typename Cloud::ConstPtr CloudConstPtr;

        TacoOpenNI(); 
        ~TacoOpenNI() {}

    private:
        NodeHandle nh, nh_home;

        // Namespace to find the camera in.
        string camera;

        // Leaf size (xyz) to downsample the camera feed
        double downsampling_grid_size_;

        // Cloud to work with, has been filtered and downsampled
        Cloud target_cloud_;

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
