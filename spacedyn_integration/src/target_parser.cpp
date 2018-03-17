#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/target_parser.h"
#include <vector>

namespace gazebo {
/**Constructor**/
	GazeboRosSPDTargetParser::GazeboRosSPDTargetParser()
	{
		//!==========Locate and Load the Model Parameters=============!//
		std::string modelpath= ros::package::getPath("spacedyn_integration"); //added 28.1.17
		model(modelpath + "/config/model_target.def", mt );  // modified 28.1.17 
        GazeboRosSPDTargetParser::initSpd(); //!Initialize SpaceDyn variables and references
	}
	
/**Destructor**/
	
	GazeboRosSPDTargetParser::~GazeboRosSPDTargetParser()
	{
		event::Events::DisconnectWorldUpdateBegin(updateConnection);
		//node_handle_->shutdown();
		//delete node_handle_;
	}

	void GazeboRosSPDTargetParser::initSpd()
	{
		/**Global Initialization **/
		model_init(mt); //!initialize state values of model and environment            
	}

	/** Loading information from the SDF **/

	void GazeboRosSPDTargetParser::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{ 
	  
		if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
		}
	  
	  //! Aim all the pointers to the elements in Gazebo
	  this->world_=_model->GetWorld();
	  ROS_INFO("Loading the SpaceDyn Plugin for Target");
	  this->parent_=_model;
	  this->base_link_=parent_->GetLink("base_footprint");
	  GazeboRosSPDTargetParser::ModelLoad(parent_, mt, _sdf); //! Map model from gazebo to MODEL m in model.h
	  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  boost::bind(&GazeboRosSPDTargetParser::OnUpdate, this, _1));
	}

	// Update in every simulation step
	void GazeboRosSPDTargetParser::OnUpdate(const common::UpdateInfo & )
	{
		//!static bool flag_INIT = true;
		static unsigned int times = 0;
		
		//! Simulation Cycle
		//! f_dyn_rk(mt,Gravity,d_time); //! spacedyn LEAVE DISABLED
		GazeboRosSPDTargetParser::tMapping(mt); //! Mapping of the positions of the joints and state of the base. Similar to Calculation of Forward Dynamics from Gazebo : Space DYN f_dyn_rk(m,Gravity,d_time); 
		
       //! for debugging purposes
		if (times == 0) {
			math::Vector3 postarget=this->base_link_->GetWorldInertialPose().pos; 
			ROS_WARN("position gazebo of target x:%f, y:%f, z:%f",postarget[0],postarget[1],postarget[2]);
			//ROS_WARN("position spacedyn of base x:%f, y:%f, z:%f",mt.POS0[0],mt.POS0[1],mt.POS0[2]);
			times=-1;
		}
    };
	GZ_REGISTER_MODEL_PLUGIN(GazeboRosSPDTargetParser)
}
// --- EOF ---

