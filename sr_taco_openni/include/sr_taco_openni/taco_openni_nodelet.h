#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "nodelet/NodeletLoad.h"
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

/**
 * @brief Util class to hold some publishers need by TacoOpenNINodelet.
 *
 * Will get one for foviated and one for unfoviated
 */
class TacoOpenNIPubs {
    public:
        TacoOpenNIPubs() {}
        /** Setup the pubs for the type ("foveated","unfoveated") passed using
         * the node handle passed.
         */
        TacoOpenNIPubs(NodeHandle, string);
        ~TacoOpenNIPubs() {}

        Publisher pointCloud;
        Publisher depthInfo;
        Publisher depthImage;
        Publisher intensityInfo;
        Publisher intensityImage;
};

/**
 * @brief The main tacoSensor nodelet.
 *
 * Subscribers to the camera topics (e.g. a Kinect) and re-publishes messages
 * on the TACO topics.
 *
 * Point clouds are downsampled and z filtered, to reduce their size and
 * increase the speed they can be processed, before being republished.
 *
 * AttentionManager nodelets should subscribe to the unfoveated cloud published
 * by this nodelet so they see the down sampled version and it makes it easy
 * to change the source of the cloud (e.g. from a Kinect to a bag recording).
 */
class TacoOpenNINodelet : public nodelet::Nodelet {
    public:
        virtual ~TacoOpenNINodelet() {}

        virtual void onInit();
        virtual bool setFoveationMode(string);
        virtual string getFoveationMode();

    private:
        NodeHandle nh, nh_home;

        /// Name of the nodelet manager we are part of and managing (for loading attention manager nodelets).
        string manager_;

        /// Namespace to find the camera in.
        string camera;

        /// The attention manager to use.
        string foveation_mode_;

        /// Leaf size (xyz) to downsample the camera feed
        double downsampling_grid_size_;

        /// Z filter the cloud in this z range, which is by depth.
        double filter_z_min_, filter_z_max_;

        /// Cloud that came in on input, converted to PCL type
        CloudPtr input_cloud_;
        /// Cloud to work with, has been filtered and downsampled
        CloudPtr target_cloud_;

        TacoOpenNIPubs foveated_, unfoveated_;

        // Stash all our subscribers to keep them alive.
        vector<Subscriber> subs;

        void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageCb(const sensor_msgs::Image::ConstPtr& msg);
};

} // sr_taco_openni::
