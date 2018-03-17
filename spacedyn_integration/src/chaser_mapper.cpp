//! Source file about functions that link the forward dynamics and kinematics from Gazebo and SPACE DYN

#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/chaser_controller.h"
#include <vector>

namespace gazebo {

	void GazeboRosSPDChaserController::qMapping(MODEL& m , const double& dt)   //! MAPPING PARAMETERS GAZEBO <--> SPACE DYN 
	{   
				//!== Mapping of Base ==/
		
		for (unsigned int i=0;i<3;++i){
			m.F0[i]=this->base_link_->GetWorldForce()[i];  //! Map forces and torques in the base of the robot
		    m.T0[i]=this->base_link_->GetWorldTorque()[i];
			m.POS0[i]=this->base_link_->GetWorldInertialPose().pos[i]; //! position of the base
			m.v0[i]=this->base_link_->GetWorldLinearVel()[i]; //! base world linear vel
			m.vd0[i]=this->base_link_->GetWorldLinearAccel()[i]; //! base world linear accel
			m.w0[i]=this->base_link_->GetWorldAngularVel()[i]; //! base world angular vel
			m.wd0[i]=this->base_link_->GetWorldAngularAccel()[i]; //! base world angular accel
		} 
		
		//!Mapping of joint parameters //
		static double freq = 1/dt;
		//! GetAngle(1) fix joint base  
		m.q[1] = this->j1_->GetAngle(2).Radian();  //! m.q[0] is not used because length (m.q) = LINKNUM, #joints=LINKNUM-1
		m.q[2] = this->j2_->GetAngle(3).Radian();
		m.q[3] = this->j3_->GetAngle(4).Radian();
		m.q[4] = this->j4_->GetAngle(5).Radian();
		m.q[5] = this->j5_->GetAngle(6).Radian();
		m.q[6] = this->j6_->GetAngle(7).Radian();
		//!  GetAngle(8) fix joint camera
		m.q[7] = this->d1_->GetAngle(9).Radian();
		m.qd[1] = this->j1_->GetVelocity(2);
		m.qd[2] = this->j2_->GetVelocity(3);
		m.qd[3] = this->j3_->GetVelocity(4);
		m.qd[4] = this->j4_->GetVelocity(5);
		m.qd[5] = this->j5_->GetVelocity(6);
		m.qd[6] = this->j6_->GetVelocity(7);
		m.qd[7] = this->d1_->GetVelocity(9);
		
		
		m.qdd[1] = (m.qd[1]-m.qd_prev[1])*freq; // indirect calculation of joint acceleration
		m.qdd[2] = (m.qd[2]-m.qd_prev[2])*freq; 
		m.qdd[3] = (m.qd[3]-m.qd_prev[3])*freq;
		m.qdd[4] = (m.qd[4]-m.qd_prev[4])*freq;
		m.qdd[5] = (m.qd[5]-m.qd_prev[5])*freq;
		m.qdd[6] = (m.qd[6]-m.qd_prev[6])*freq;
		m.qdd[7] = (m.qd[7]-m.qd_prev[7])*freq; 	
		

		m.Qtn0[0]=this->base_link_->GetWorldInertialPose().rot.w; 
		m.Qtn0[1]=this->base_link_->GetWorldInertialPose().rot.x;
		m.Qtn0[2]=this->base_link_->GetWorldInertialPose().rot.y;
		m.Qtn0[3]=this->base_link_->GetWorldInertialPose().rot.z;   
		
		qtn2dc( m.Qtn0, m.A0 ); //! Transform the quaternion into a transformation matrix.
		
		matrix_cpy(3,3,m.A0,chaser_A0);
		
		this->ee_world_pose_=math::Pose(m.CtoE[m.E_NUM-1][0],m.CtoE[m.E_NUM-1][1],m.CtoE[m.E_NUM-1][2],0,0,0).operator+(this->hand_->GetWorldInertialPose()); //! World
		
		//! Mapping of end effector forces
		math::Vector3 force_world_ee = this->hand_->GetWorldForce();
		math::Vector3 torque_world_ee =  this->hand_->GetWorldTorque().operator+((this->ee_world_pose_.pos.operator-(this->hand_->GetWorldInertialPose().pos)).Cross(force_world_ee)); 
		vConvert(force_world_ee,m.Fe[1]);
		vConvert(torque_world_ee,m.Te[1]);
		
	   m.qd_prev = m.qd; //! Save the last joint velocity to calculate acceleration;
	}
}
