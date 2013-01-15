/**
 * @author: mda
 * @date 15th Jan 2013
 */

#include <sr_taco_openni/attention_manager.h>
#include <pluginlib/class_list_macros.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

namespace sr_taco_openni
{
  using namespace std;
  using namespace ros;

  /**
   * @brief Example AttentionManager that just puts a circle on the saliency map.
   *
   * This is a nodelet to run along with the main taco_openni_nodelet.
   *
   * Publishes a saliency_map_spatial/image.
   */
  class AttentionExampleCircle : public AttentionManager
  {
    public:
      AttentionExampleCircle(){}

    private:
      NodeHandle nh_;

      Subscriber pointcloud_sub_;
      Publisher saliency_map_spatial_pub_;

      sensor_msgs::CvBridge bridge_;

      /// Radius of the circle to draw
      int radius_;
      /// x position of the circle. Default is middle.
      int x_;
      /// y position of the circle. Default is middle.
      int y_;

      virtual void onInit()
      {
        NODELET_INFO("Starting ExampleCirle attention manager.");
        nh_ = getNodeHandle();

        nh_.param<int>("attention/example_circle/radius", radius_, 100);
        nh_.param<int>("attention/example_circle/x", x_, taco_width/2);
        nh_.param<int>("attention/example_circle/y", y_, taco_height/2);

        // Setup a callback so we pub in step with the cloud. ie in its frame.
        pointcloud_sub_ = nh_.subscribe("/tacoSensor/unfoveated/pointcloud2", 1, &AttentionExampleCircle::cloudCb, this);

        saliency_map_spatial_pub_ = nh_.advertise<sensor_msgs::Image>( "saliency_map_spatial/image", 5);
      }

      void cloudCb(const CloudConstPtr& cloud)
      {
        NODELET_INFO_STREAM_ONCE("cloudCb:" << *cloud);

        SaliencyMapPtr map = newSaliencyMap();
        string frame_id = cloud->header.frame_id;
        map->header.frame_id = frame_id;

        // Convert saliency msg to cv image
        IplImage* image = NULL;
        try {
            image = bridge_.imgMsgToCv(map, "mono8");
        }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
        }

        cv::Point2d pt(x_,y_);
        cvCircle(image, pt, radius_, CV_RGB(255,255,255), -1);

        // Convert the cv image back to msg and publish
        bridge_.cvToImgMsg(image, "mono8");
        saliency_map_spatial_pub_.publish(map);
      }

  };
}

PLUGINLIB_DECLARE_CLASS(sr_taco_openni,attention_example_circle,sr_taco_openni::AttentionExampleCircle,nodelet::Nodelet)
