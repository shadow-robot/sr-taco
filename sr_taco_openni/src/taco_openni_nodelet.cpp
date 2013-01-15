#include "sr_taco_openni/taco_openni_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include "sr_pcl_tracking/cluster_segmentor.h"
#include <sensor_msgs/image_encodings.h>

PLUGINLIB_DECLARE_CLASS(sr_taco_openni, taco_openni_nodelet, sr_taco_openni::TacoOpenNINodelet, nodelet::Nodelet)

namespace sr_taco_openni {

    using namespace sr_pcl_tracking;

// TacoOpenNIPubs -------------------------------------------------------------

    TacoOpenNIPubs::TacoOpenNIPubs(NodeHandle nh, string type) {
        pointCloud = nh.advertise<sensor_msgs::PointCloud2>( type+"/pointcloud2", 5);
        depthInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/depth/camera_info", 5);
        depthImage = nh.advertise<sensor_msgs::Image>( type+"/depth/image", 5);
        intensityInfo = nh.advertise<sensor_msgs::CameraInfo>( type+"/intensity/camera_info", 5);
        intensityImage = nh.advertise<sensor_msgs::Image>( type+"/intensity/image", 5);
    }


// TacoOpenNINodelet ----------------------------------------------------------------

    void TacoOpenNINodelet::onInit() {
        NODELET_INFO("Starting main Taco nodelet");

        nh_home = getPrivateNodeHandle();
        foveated_ = TacoOpenNIPubs(nh_home, "foveated");
        unfoveated_ = TacoOpenNIPubs(nh_home, "unfoveated");

        nh_home.param<string>("manager", manager_, "/tacoSensor_nodelet_manager");
        nh_home.param<string>("camera", camera, "camera");
        nh_home.param<string>("foveation_mode", foveation_mode_, "cluster_segment");
        nh_home.param<double>("downsampling_grid_size", downsampling_grid_size_, 0.01);
        nh_home.param<double>("filter_z_min", filter_z_min_, 0.0);
        nh_home.param<double>("filter_z_max", filter_z_max_, 10.0);

        string depth = camera + "/depth";

        subs.push_back( nh.subscribe(depth + "/points",
               1, &TacoOpenNINodelet::cloudCb, this) );

        subs.push_back( nh.subscribe(depth + "/camera_info",
               1, &TacoOpenNINodelet::cameraInfoCb, this) );

        subs.push_back( nh.subscribe(depth + "/image",
               1, &TacoOpenNINodelet::depthImageCb, this) );

        setFoveationMode(foveation_mode_);
    }

    string TacoOpenNINodelet::getFoveationMode()
    {
      return foveation_mode_;
    }

    bool TacoOpenNINodelet::setFoveationMode(string name)
    {
      string type = "sr_taco_openni/"+name;
      NODELET_INFO_STREAM("Setting foveation mode to " << name << " (" << type << ")");
//      // Call the service on our nodelet manager to load the new manager.
//      ros::ServiceClient load_client = nh_home.serviceClient<nodelet::NodeletLoad>(manager_ + "/load_nodelet");
//      if ( !load_client.waitForExistence(ros::Duration(5.0)) )
//      {
//        NODELET_ERROR_STREAM("Nodelet manager " << manager_ << " is not responding.");
//        return false;
//      }
//      NODELET_INFO_STREAM("Manager contacted " << manager_);
//      nodelet::NodeletLoad load_srv;
//      load_srv.request.name = name;
//      load_srv.request.type = type;
//      if ( !load_client.call(load_srv) )
//      {
//        NODELET_FATAL_STREAM("Service call failed to load nodelet "<< name << " of type " << type << " into manager" << manager_);
//        return false;
//      }

      // XXX Hack! Use fork/exec to call the nodelet exe.
      // The above service call only breifly starts the nodelet as the calling process needs
      // to hang around so we fork off a proc to do that. See: $(rospack find nodelet)/src/nodelet.cpp
      // rosrun nodelet nodelet load sr_taco_openni/attention_example_circle /tacoSensor_nodelet_manager __name:=attention_example_circle __ns:=/tacoSensor
      pid_t pid = fork();
      switch (pid)
      {
        case -1: // ERROR
          NODELET_ERROR("Fork failed");
          return false;
        case 0: // CHILD process
          string name_arg = "__name:=" + name;
          string ns_arg = "__ns:=/tacoSensor";
          execlp("rosrun", "rosrun", "nodelet", "nodelet", "load",  type.c_str(), manager_.c_str(), name_arg.c_str(), ns_arg.c_str(), NULL);
          // Exec will only return on error.
          NODELET_ERROR("Exec failed!");
          return false;
        // default: // PARENT process - keep running!
      }

      NODELET_INFO_STREAM("Loaded nodelet " << name << " of type " << type << " into nodelet manager " << manager_);
      foveation_mode_ = name;
      return true;
    }

    void TacoOpenNINodelet::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        input_cloud_.reset(new Cloud);
        CloudPtr tmp_cloud(new Cloud);
        pcl::fromROSMsg(*cloud, *input_cloud_);
        ROS_INFO_STREAM_ONCE("input_cloud: " << *input_cloud_);

        // Do some z filtering, to reduce number points and focus on foreground
        pcl::PassThrough<PointType> pass;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_z_min_, filter_z_max_);
        pass.setKeepOrganized (false);
        pass.setInputCloud (input_cloud_);
        pass.filter (*tmp_cloud);

        // Downsample the point cloud to speed up processing
        target_cloud_.reset(new Cloud);
        pcl::VoxelGrid<PointType> grid;
        grid.setInputCloud(tmp_cloud);
        grid.setLeafSize(downsampling_grid_size_, downsampling_grid_size_, downsampling_grid_size_);
        grid.filter(*target_cloud_);

        ROS_INFO_STREAM_ONCE("target_cloud: " << *target_cloud_);

        // No foveation yet so pub the same cloud twice
        sensor_msgs::PointCloud2Ptr out_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*target_cloud_, *out_cloud);
        unfoveated_.pointCloud.publish(out_cloud);
        foveated_.pointCloud.publish(out_cloud);
    }

    void TacoOpenNINodelet::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        foveated_.depthInfo.publish(msg);
        foveated_.intensityInfo.publish(msg);
        unfoveated_.depthInfo.publish(msg);
        unfoveated_.intensityInfo.publish(msg);
    }
        
    void TacoOpenNINodelet::depthImageCb(const sensor_msgs::Image::ConstPtr& msg) {
        foveated_.depthImage.publish(msg);
        unfoveated_.depthImage.publish(msg);

        // Just pub a blank image for now, not sure what we need here and we
        // don't use the intensity image atm.
        sensor_msgs::Image::Ptr blank_img(new sensor_msgs::Image);
        blank_img->width = taco_width;
        blank_img->height = taco_height;
        blank_img->encoding = sensor_msgs::image_encodings::MONO8;
        blank_img->step = taco_width; // * 1 as using mono
        blank_img->data.resize( taco_height * blank_img->step );
        foveated_.intensityImage.publish(blank_img);
        unfoveated_.intensityImage.publish(blank_img);
    }

} // sr_taco_openni::
