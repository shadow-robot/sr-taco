#include "sr_pcl_tracking/cluster_segmentor.h"
#include "sr_pcl_tracking/template_alignment.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>


namespace sr_pcl_tracking {

using namespace pcl::tracking;

class Tracker {

public:
//    typedef pcl::PointXYZRGBA PointType;
    typedef pcl::PointXYZRGB PointType;
    typedef ParticleXYZRPY ParticleT;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef ParticleFilterTracker<PointType, ParticleT> ParticleFilter;

    Tracker()
        : nh_home_("~")
        , downsampling_grid_size_(0.01)
    {
        // ROS setup
        input_sub_ = nh_home_.subscribe("input", 1, &Tracker::cloud_cb, this);
        output_pub_ = nh_home_.advertise<sensor_msgs::PointCloud2>("output", 1);

        // PCL Tracking setup
        bool use_fixed = false;
        int thread_nr = 8;

        std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
        if (use_fixed) {
            boost::shared_ptr<ParticleFilterOMPTracker<PointType, ParticleT> > tracker(
                    new ParticleFilterOMPTracker<PointType, ParticleT>(thread_nr));
            tracker_ = tracker;
        }
        else {
            boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT> > tracker(
                    new KLDAdaptiveParticleFilterOMPTracker<PointType, ParticleT>(thread_nr));
            tracker->setMaximumParticleNum(500);
            tracker->setDelta(0.99);
            tracker->setEpsilon(0.2);
            ParticleT bin_size;
            bin_size.x = 0.1;
            bin_size.y = 0.1;
            bin_size.z = 0.1;
            bin_size.roll = 0.1;
            bin_size.pitch = 0.1;
            bin_size.yaw = 0.1;
            tracker->setBinSize(bin_size);
            tracker_ = tracker;
        }

        tracker_->setTrans(Eigen::Affine3f::Identity());
        tracker_->setStepNoiseCovariance(default_step_covariance);
        tracker_->setInitialNoiseCovariance(initial_noise_covariance);
        tracker_->setInitialNoiseMean(default_initial_mean);
        tracker_->setIterationNum(1);

        tracker_->setParticleNum(400);
        tracker_->setResampleLikelihoodThr(0.00);
        tracker_->setUseNormal(false);

        // setup coherences
        typename ApproxNearestPairPointCloudCoherence<PointType>::Ptr coherence =
                typename ApproxNearestPairPointCloudCoherence<PointType>::Ptr(
                        new ApproxNearestPairPointCloudCoherence<PointType>());

        boost::shared_ptr<DistanceCoherence<PointType> > distance_coherence = boost::shared_ptr<
                DistanceCoherence<PointType> >(new DistanceCoherence<PointType>());
        coherence->addPointCoherence(distance_coherence);

        boost::shared_ptr<HSVColorCoherence<PointType> > color_coherence = boost::shared_ptr<
                HSVColorCoherence<PointType> >(new HSVColorCoherence<PointType>());
        color_coherence->setWeight(0.1);
        coherence->addPointCoherence(color_coherence);

        boost::shared_ptr<pcl::search::Octree<PointType> > search(new pcl::search::Octree<PointType>(0.01));
        coherence->setSearchMethod(search);
        coherence->setMaximumDistance(0.01);
        tracker_->setCloudCoherence(coherence);

        // Start tracking an empty cloud
        CloudPtr ref_cloud(new Cloud);
        trackCloud(ref_cloud);
    }

    void run () { ros::spin(); }

protected:
    void
    cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        // Convert incoming cloud to PCL type
        cloud_pass_.reset (new Cloud);
        cloud_pass_downsampled_.reset (new Cloud);
        pcl::fromROSMsg(*cloud, *cloud_pass_);

        // TODO - Filter z?

        gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
        // Avoid lots of noisey pcl lib output when no cloud to track
        if (reference_->points.size() > 0) {
            tracker_->setInputCloud (cloud_pass_downsampled_);
            tracker_->compute ();
        }

        sensor_msgs::PointCloud2 out_cloud;
        pcl::toROSMsg(*cloud_pass_downsampled_, out_cloud);
        output_pub_.publish (out_cloud);
    }

    void
    gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
    {
      pcl::ApproximateVoxelGrid<PointType> grid;
      grid.setLeafSize (leaf_size, leaf_size, leaf_size);
      grid.setInputCloud (cloud);
      grid.filter (result);
    }

    void
    gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
    {
      pcl::VoxelGrid<PointType> grid;
      grid.setLeafSize (leaf_size, leaf_size, leaf_size);
      grid.setInputCloud (cloud);
      grid.filter (result);
    }

    void
    trackCloud (const CloudConstPtr &ref_cloud)
    {
        Eigen::Vector4f c;
        CloudPtr transed_ref (new Cloud);
        pcl::compute3DCentroid<PointType> (*ref_cloud, c);
        Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
        trans.translation () = Eigen::Vector3f (c[0], c[1], c[2]);
        pcl::transformPointCloud<PointType> (*ref_cloud, *transed_ref, trans.inverse ());
        CloudPtr transed_ref_downsampled (new Cloud);
        gridSample (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
        tracker_->setReferenceCloud (transed_ref_downsampled);
        tracker_->setTrans (trans);
        reference_ = transed_ref;
        tracker_->setMinIndices (ref_cloud->points.size () / 2);
    }

    ros::NodeHandle nh_, nh_home_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;

    boost::shared_ptr<ParticleFilter> tracker_;
    CloudPtr cloud_pass_;
    CloudPtr cloud_pass_downsampled_;
    CloudPtr reference_;
    double downsampling_grid_size_;

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
