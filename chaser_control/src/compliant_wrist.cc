#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gazebo/math/gzmath.hh>
#include "spacedyn_integration/dataext.h"
//! PLUGING FOR COMPLIANT WRIST

namespace gazebo
{
	class compliant_wrist : public ModelPlugin
	{
		//! Variable declaration for compliant_wrist
		private: physics::ModelPtr model;
		private: physics::JointPtr d1;

		//! Node and connector declaration
		private: event::ConnectionPtr updateConnection;

		//! Variables for current status
		private: double stiffness_;
		private: double damping_;
		private: double force_;
		private: double ref_;
		private: int axis_;
		
		//! Constructor and destroyer
		public: compliant_wrist(){}
		public: ~compliant_wrist(){
			event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
			ROS_INFO("Compliant Wrist Disabled");	
		}

		//! Gazebo runs this method when the model is spawned
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			//! Telling to ROS for compliant_wrist
			//! Register the topic "joint_angle_compliant_wrist"
			ROS_INFO("LOADING COMPLIANT WRIST");	
			
			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "compliant_wrist");

			//! Get instances for each joint from the model
			this->model = _parent;
			this->model->SetAutoDisable(false);
			d1 = this->model->GetJoint("d1");
			//!hand=this->model->GetLink("hand");
			//! Connect the update handler with an event
			
					//! variable initialization
			stiffness_= 530; //! 530, 5100
			damping_=1.4;
			ref_=0.0;
			axis_=2;
			
			if (_sdf->HasElement("stiffnessW")){
				_sdf->GetElement("stiffnessW")->GetValue()->Get(stiffness_);}
				
			if (_sdf->HasElement("dampingW")){
				_sdf->GetElement("dampingW")->GetValue()->Get(damping_);}
			
			if (_sdf->HasElement("referenceW")){
				_sdf->GetElement("referenceW")->GetValue()->Get(ref_);}				
			
			if (_sdf->HasElement("Axis")){
				_sdf->GetElement("Axis")->GetValue()->Get(axis_);}				
			
			
			//! load variables for Spacedyn Impulse Controller 
			dw=damping_;
			kw=stiffness_;
			
			
			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&compliant_wrist::OnUpdate,
										this, _1));
	
		}

		//! Gazebo periodically runs this callback to update compliant_wrist's status 
		public: void OnUpdate(const common::UpdateInfo &)
		{			
			force_=((ref_-d1->GetAngle(this->axis_).Radian())*stiffness_)-(d1->GetVelocity(this->axis_)*damping_); //!0.00 is the reference of start in the spring
			//!ROS_INFO("%f",force);
			d1->SetForce(this->axis_,force_); //!0.00 is the reference of start in the spring
		} 
	};
	//! Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(compliant_wrist)
}




