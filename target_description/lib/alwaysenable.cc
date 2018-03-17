#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
 
namespace gazebo
{
  class alwaysenable : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
	  ROS_INFO("Load_Always_Enable_Target");
      this->model = _parent;
      this->model->SetAutoDisable(false);
    }
	
	//! Constructor and destroyer
		public: alwaysenable(){}
		public: ~alwaysenable(){
			ROS_INFO("Done");	
		}
	
    // Pointer to the model
    private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(alwaysenable)
}
