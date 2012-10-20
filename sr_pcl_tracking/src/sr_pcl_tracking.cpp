#include "sr_pcl_tracking/cluster_segmentor.h"
#include "sr_pcl_tracking/template_alignment.h"
#include "sr_pcl_tracking/SaveReference.h"
#include "sr_pcl_tracking/LoadReference.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

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
#include <boost/filesystem.hpp>
#include <wordexp.h>


namespace sr_pcl_tracking {

using namespace pcl::tracking;
namespace fs = boost::filesystem;

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
        , reference_dir_ ("~/.ros/sr_pcl_tracking")
        , downsampling_grid_size_(0.01)
    {
        // ROS setup
        input_sub_ = nh_home_.subscribe("input", 1, &Tracker::cloud_cb, this);
        output_pub_ = nh_home_.advertise<sensor_msgs::PointCloud2>("output", 1);
        particle_cloud_pub_ = nh_home_.advertise<sensor_msgs::PointCloud2>("particle_cloud", 1);
        result_cloud_pub_ = nh_home_.advertise<sensor_msgs::PointCloud2>("result_cloud", 1);
        track_nearest_srv_ = nh_home_.advertiseService("track_nearest", &Tracker::trackNearest_cb, this);
        track_centered_srv_ = nh_home_.advertiseService("track_centered", &Tracker::trackCentred_cb, this);
        load_srv_ = nh_home_.advertiseService("load_reference", &Tracker::loadReference_cb, this);
        save_srv_ = nh_home_.advertiseService("save_reference", &Tracker::saveReference_cb, this);

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
        input_ = cloud;

        // Convert incoming cloud to PCL type
        CloudPtr input_cloud(new Cloud);
        pcl::fromROSMsg(*cloud, *input_cloud);

        cloud_pass_.reset (new Cloud);
        cloud_pass_downsampled_.reset (new Cloud);

        pcl::PassThrough<PointType> pass;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 10.0);
        pass.setKeepOrganized (false);
        pass.setInputCloud (input_cloud);
        pass.filter (*cloud_pass_);

        gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

        if (reference_->points.size() > 0)
            tracking ();

        sensor_msgs::PointCloud2 out_cloud;
        pcl::toROSMsg(*cloud_pass_downsampled_, out_cloud);
        output_pub_.publish (out_cloud);
    }

    void
    tracking ()
    {
        tracker_->setInputCloud(cloud_pass_downsampled_);
        tracker_->compute();

        // Publish the particle cloud
        typename ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
        if (particles) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (size_t i = 0; i < particles->points.size(); i++) {
                pcl::PointXYZ point;
                point.x = particles->points[i].x;
                point.y = particles->points[i].y;
                point.z = particles->points[i].z;
                particle_cloud->points.push_back(point);
            }
            sensor_msgs::PointCloud2 out_cloud;
            pcl::toROSMsg(*particle_cloud, out_cloud);
            // Copy the header so we get the right frame
            // XXX - Should we update the time stamp?
            out_cloud.header = input_->header;
            particle_cloud_pub_.publish (out_cloud);
        }

        // Publish the result cloud
        ParticleXYZRPY result = tracker_->getResult();
        Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
        // move a little bit for better visualization
//       transformation.translation() += Eigen::Vector3f(0.0, 0.0, -0.005);
        CloudPtr result_cloud(new Cloud());
//        if (!visualize_non_downsample_)
//            pcl::transformPointCloud<PointType>(*(tracker_->getReferenceCloud()), *result_cloud, transformation);
//        else
        pcl::transformPointCloud<PointType>(*reference_, *result_cloud, transformation);
        {
            sensor_msgs::PointCloud2 out_cloud;
            pcl::toROSMsg(*result_cloud, out_cloud);
            out_cloud.header = input_->header;
            result_cloud_pub_.publish (out_cloud);
        }

        // TODO: Publish the transformation (pose)
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

    enum SegmentSort
    {
        SEGMENT_SORT_BY_DISTANCE,
        SEGMENT_SORT_BY_CENTERED,
    };

    bool
    trackNearest_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
    {
        return segmentReferece(SEGMENT_SORT_BY_DISTANCE);
    }

    bool
    trackCentred_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
    {
        return segmentReferece(SEGMENT_SORT_BY_CENTERED);
    }

    bool
    segmentReferece(SegmentSort sort_type)
    {
        // If this fails we get an empty ref_cloud
        CloudPtr ref_cloud(new Cloud);
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        cluster_segmentor.setInputCloud(cloud_pass_);
        if (sort_type == SEGMENT_SORT_BY_CENTERED)
            cluster_segmentor.extractByCentered(clusters);
        else
            cluster_segmentor.extractByDistance(clusters);
        if (clusters.size() > 0)
            ref_cloud = clusters[0];

        std::cout << "ref_cloud: "
                << " points: " << ref_cloud->points.size()
                << " wh:" << ref_cloud->width << "x" << ref_cloud->height
                << " is_dense: " << (ref_cloud->is_dense ? "Yes" : "No")
                << std::endl;

        trackCloud(ref_cloud);

        return true;
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

    /**
     * Return referece_dir_ as a boost::filesystem::path after performing word expansion like a
     * POSIX shell (e.g. expand ~).
     */
    fs::path
    referenceDirPath ()
    {
        wordexp_t exp_result;
        wordexp(reference_dir_.c_str(), &exp_result, 0);
        fs::path path(exp_result.we_wordv[0]);
        wordfree (&exp_result);

        if ( !fs::exists(path) )
            throw ros::Exception("Reference dir '" + path.string() + "' does not exist. "
                    "You will need to create it.");
        if ( !fs::is_directory(path) )
            throw ros::Exception("Reference dir '" + path.string() + "' is not a directory!");

        return path;
    }

    bool
    saveReference_cb (SaveReferenceRequest &req, SaveReferenceResponse &res)
    {
        if (req.name == "")
            throw ros::Exception("Empty name");
        fs::path path;
        path /= referenceDirPath();
        path /= req.name + ".pcd";
        pcl::io::savePCDFileASCII(path.c_str(), *reference_);
        ROS_INFO_STREAM("Saved: " << path);
        return true;
    }

    bool
    loadReference_cb (LoadReferenceRequest &req, LoadReferenceResponse &res)
    {
        if (req.name == "")
            throw ros::Exception("Empty name");
        fs::path path;
        path /= referenceDirPath();
        path /= req.name + ".pcd";
        ROS_INFO_STREAM("Loading: " << path);
        CloudPtr ref_cloud(new Cloud);
        CloudPtr load_cloud(new Cloud);
        if (pcl::io::loadPCDFile<PointType>(path.c_str(), *load_cloud) == -1)
            throw ros::Exception("Failed to read file: " + path.string());
//        pcl::io::savePCDFileASCII("load_cloud.pcd", *load_cloud);
//        pcl::io::savePCDFileASCII("load_cloud_pass.pcd", *cloud_pass_);
        findCloud(load_cloud, ref_cloud);
//        pcl::io::savePCDFileASCII("load_transformed_cloud.pcd", *ref_cloud);
        trackCloud(ref_cloud);
        return true;
    }

    // http://answers.ros.org/question/9515/how-to-convert-between-different-point-cloud-types-using-pcl/
    void copyPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr dst)
    {
      dst->resize (src->size());
      dst->width = src->width;
      dst->height = src->height;
      dst->is_dense = src->is_dense;
      for (size_t i = 0; i < src->size(); ++i )
      {
        dst->points[i].x = src->points[i].x;
        dst->points[i].y = src->points[i].y;
        dst->points[i].z = src->points[i].z;
      }
    }

    /**
     * Find the passed cloud in the input cloud
     */
    void findCloud(CloudPtr find_cloud, CloudPtr out_cloud) {
        PCL_INFO("Searching for cloud\n");

        // Find all the clusters. We then try to match the find cloud against each cluster.
        std::vector<CloudPtr> clusters;
        ClusterSegmentor<PointType> cluster_segmentor;
        // XXX Use downsampled here?
        cluster_segmentor.setInputCloud(cloud_pass_);
        cluster_segmentor.extract(clusters);

        // Convert the cloud we want to find.
        // (We need PointXYZ but input tracking is using PointXYZRGB)
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_template(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(find_cloud, object_template);
        pcl::io::savePCDFileASCII("find_object_template.pcd", *object_template);

        // Set the TemplateAlignment inputs
        TemplateAlignment template_align;
//      for (size_t i = 0; i < object_templates.size (); ++i)
//      {
//        template_align.addTemplateCloud (object_templates[i]);
//      }
        FeatureCloud template_cloud;
        template_cloud.setInputCloud(object_template);
        template_align.addTemplateCloud(template_cloud);

        TemplateAlignment::Result best_result;
        best_result.fitness_score = 10000;
        for (size_t i = 0; i < clusters.size(); ++i) {
            // Assign to the target FeatureCloud
            // We need PointXYZ but openni tracking is using PointXYZRGBA
            FeatureCloud target_cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            copyPointCloud(clusters[i], temp_cloud);
            target_cloud.setInputCloud(temp_cloud);
            template_align.setTargetCloud(target_cloud);

            // Find the best template alignment
            ROS_INFO("Searching cluster %i", i);
            TemplateAlignment::Result best_alignment;
            template_align.findBestAlignment(best_alignment);
//            int best_index = template_align.findBestAlignment(best_alignment);
//            const FeatureCloud &best_template = object_templates[best_index];
//            const FeatureCloud &best_template = template_cloud;

            // Print the alignment fitness score (values less than 0.00002 are good)
            printf("Best fitness score: %f\n", best_alignment.fitness_score);
            // Print the rotation matrix and translation vector
            Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
            Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
            printf("\n");
            printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
            printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
            printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
            printf("\n");
            printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

            if (best_alignment.fitness_score < best_result.fitness_score) best_result = best_alignment;
        }

        ROS_INFO("Overall best fitness score: %f\n", best_result.fitness_score);
        // Transform the find_cloud so it aligns with the input cloud
        pcl::transformPointCloud(*find_cloud, *out_cloud, best_result.final_transformation);
    }

    ros::NodeHandle nh_, nh_home_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    ros::Publisher particle_cloud_pub_;
    ros::Publisher result_cloud_pub_;
    ros::ServiceServer track_nearest_srv_, track_centered_srv_, load_srv_, save_srv_;
    sensor_msgs::PointCloud2ConstPtr input_;

    /// Directory to load and save reference objects to and from
    std::string reference_dir_;

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
