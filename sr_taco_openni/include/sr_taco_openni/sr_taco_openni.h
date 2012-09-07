#pragma once

#include "ros/ros.h"

namespace sr_taco_openni {

class TacoOpenNI {
    public:
        TacoOpenNI(); 
        ~TacoOpenNI() {}

    private:
        ros::NodeHandle nh, nh_home;
};

} // sr_taco_openni::
