
#include "ros/ros.h"
#include "sr_taco_openni/sr_taco_openni.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tacoSensor");
    ros::NodeHandle n; // inits this node
    sr_taco_openni::TacoOpenNI obj;
    ros::spin();
    return 0;
}
