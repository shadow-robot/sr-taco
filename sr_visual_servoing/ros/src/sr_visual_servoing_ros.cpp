// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <sr_visual_servoing/sr_visual_servoingConfig.h>

// ROS message includes
#include <sr_visual_servoing/VisualServoingAction.h>




#include <sr_visual_servoing_common.cpp>


class sr_visual_servoing_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<sr_visual_servoing::sr_visual_servoingConfig> server;
  		dynamic_reconfigure::Server<sr_visual_servoing::sr_visual_servoingConfig>::CallbackType f;
		

		

	actionlib::SimpleActionServer<sr_visual_servoing::VisualServoingAction> as_visual_servo_;
        
 
        sr_visual_servoing_data component_data_;
        sr_visual_servoing_config component_config_;
        sr_visual_servoing_impl component_implementation_;

        sr_visual_servoing_ros()
        :as_visual_servo_(n_, "visual_servo", boost::bind(&sr_visual_servoing_impl::callback_visual_servo_, &component_implementation_, _1, &as_visual_servo_), false)
        {
       	
  			f = boost::bind(&sr_visual_servoing_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
 			as_visual_servo_.start();
        	
        
  	

            
        }
        
		
		void configure_callback(sr_visual_servoing::sr_visual_servoingConfig &config, uint32_t level) 
		{
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "sr_visual_servoing");

	sr_visual_servoing_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update() 
	ros::spin();
	
    return 0;
}
