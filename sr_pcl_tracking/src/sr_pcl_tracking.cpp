// http://www.ros.org/wiki/pcl/Tutorials

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace sr_pcl_tracking {

class Tracker {

public:
    Tracker()
        : nh_home_("~")
    {
        input_sub_  = nh_home_.subscribe ("input", 1, &Tracker::cloud_cb, this);
        output_pub_ = nh_home_.advertise<sensor_msgs::PointCloud2> ("output", 1);
    }

    void run () { ros::spin(); }

protected:
    void
    cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        sensor_msgs::PointCloud2 cloud_filtered;

        float leaf_size = 0.01;

        // Perform the actual filtering
        pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter (cloud_filtered);

        // Publish the data
        output_pub_.publish (cloud_filtered);
    }

    ros::NodeHandle nh_, nh_home_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;

}; // Tracker

} // sr_pcl_tracking

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "sr_pcl_tracker");
  sr_pcl_tracking::Tracker node;
  node.run();
  return 0;
}
