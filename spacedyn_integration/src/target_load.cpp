//! Source file that extracts the vectors of the kinematic chain in the robot from Gazebo to SpaceDyn CtoJ and JtoC

//! CAREFULL GAZEBO USES ABSOLUTE RPY WHEREAS SPACEDYN USES RELATIVE RPY

#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/target_parser.h"
#include <vector>

#include "spacedyn_integration/dataext.h"



namespace gazebo {

	void GazeboRosSPDTargetParser::ModelLoad(physics::ModelPtr _modelmap, MODEL &mt, sdf::ElementPtr _sdf)   //! MAPPING PARAMETERS GAZEBO <--> SPACE DYN 
	{   		
		//! Get the mass
		flag_TARGET_=true;
		mt.link_M[0]=this->base_link_->GetInertial()->GetMass();
	
		//ROS_WARN("mass %f", mt.link_M[0]);
		
		//!  Get the inertia
		
		GazeboRosSPDTargetParser::ConvInertial(mt.link_I[0],this->base_link_);
		calc_SPN( mt ); //! convert parameters to spatial notation
	
	}
	void GazeboRosSPDTargetParser::ConvInertial(Matrix3& link_I_, const physics::LinkPtr& _link) {
		
		//###Intertia_Matrix  //! Taken from model.cpp SPD
		//###[_I11_I12_I13_]
		//###[_I12_I22_I23_]
		//###[_I13_I23_I33_]
		
			  link_I_[0*3+0]=_link->GetInertial()->GetIXX();
			  link_I_[1*3+1]=_link->GetInertial()->GetIYY();
			  link_I_[2*3+2]=_link->GetInertial()->GetIZZ();
			  link_I_[0*3+1]=_link->GetInertial()->GetIXY(); link_I_[1*3+0] = link_I_[0*3+1];
			  link_I_[0*3+2]=_link->GetInertial()->GetIXZ(); link_I_[2*3+0] = link_I_[0*3+2];
			  link_I_[1*3+2]=_link->GetInertial()->GetIYZ(); link_I_[2*3+1] = link_I_[1*3+2];
		}

}

