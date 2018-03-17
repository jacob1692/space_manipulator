//! Source file about functions that link the forward dynamics and kinematics from Gazebo and SPACE DYN

#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/target_parser.h"
#include <vector>

namespace gazebo {

	void GazeboRosSPDTargetParser::tMapping(MODEL& mt)   //! MAPPING PARAMETERS GAZEBO <--> SPACE DYN 
	{   
		//!== Mapping of Base ==/

		for (unsigned int i=0;i<3;++i){
			mt.F0[i]=this->base_link_->GetWorldForce()[i];
			mt.T0[i]=this->base_link_->GetWorldTorque()[i];
			mt.POS0[i]=this->base_link_->GetWorldInertialPose().pos[i]; //! position of the base
			mt.v0[i]=this->base_link_->GetWorldLinearVel()[i]; //! base world linear vel
			mt.vd0[i]=this->base_link_->GetWorldLinearAccel()[i]; //! base world linear accel
			mt.w0[i]=this->base_link_->GetWorldAngularVel()[i]; //! base world angular vel
			mt.wd0[i]=this->base_link_->GetWorldAngularAccel()[i]; //! base world angular accel
		} 
		
		mt.Qtn0[0]=this->base_link_->GetWorldInertialPose().rot.w; 
		mt.Qtn0[1]=this->base_link_->GetWorldInertialPose().rot.x;
		mt.Qtn0[2]=this->base_link_->GetWorldInertialPose().rot.y;
		mt.Qtn0[3]=this->base_link_->GetWorldInertialPose().rot.z;   
		
		qtn2dc( mt.Qtn0, mt.A0 ); //! Transform the quaternion into a transformation matrix. 
		matrix_cpy(3,3,mt.A0,target_A0);
	}
}
