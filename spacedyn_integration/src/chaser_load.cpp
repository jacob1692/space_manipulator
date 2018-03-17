//! Source file that extracts the vectors of the kinematic chain in the robot from Gazebo to SpaceDyn CtoJ and JtoC

//! CAREFULL GAZEBO USES ABSOLUTE RPY WHEREAS SPACEDYN USES RELATIVE RPY

#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/chaser_controller.h"
#include <vector>



namespace gazebo {

	void GazeboRosSPDChaserController::ModelLoad(physics::ModelPtr _modelmap, MODEL &m, sdf::ElementPtr _sdf)   //! MAPPING PARAMETERS GAZEBO <--> SPACE DYN 
	{   	
		/*  THIS IS BETTER EXTRACTED FROM MODEL.DEF
		std::string space; 
		if (_sdf->HasElement("BB")){   //! Extract BB from SDF e.g "_0_1_2_3_4_5_6_"
			std::string BBchain;
			BBchain=_sdf->GetElement("BB")->Get<std::string>();	
			ifstream ifs(BBchain.c_str());
			for (unsigned int i=1;i<m.LINKNUM;++i){ //! "_0_1_2_3_4_5_6"
				ifs >> space; //! "_"
				ifs >> m.BB[i];
			}
			ifs >> space; //! "_" last space 
			ifs.close();
		 }
		 else {
			for (unsigned int i=0;i<m.LINKNUM-2;++i){
				m.BB[i]=i; //! 0 1 2 3 4 5 6 
			}
		 }	
		 if (_sdf->HasElement("EE")){   //! Extract EE from SDF e.g "_0_0_0_0_0_0_1_"
			std::string EEchain;
			EEchain=_sdf->GetElement("EE")->Get<std::string>();	
			ifstream ifs(EEchain.c_str());
			for (unsigned int i=1;i<m.LINKNUM;++i){ //! "_0_0_0_0_0_0_1"
				ifs >> space; //! "_"
				ifs >> m.EE[i];
			}
			ifs >> space; //! "_" last space 
			ifs.close();
		 }
		 else { m.EE[m.LINKNUM-1]=1; //! By default, the last link has a end effector 
			 }
		  if (_sdf->HasElement("jType")){   //! Extract EE from SDF e.g "_0_0_0_0_0_0_1_"
			std::string jTypechain;
			jTypechain=_sdf->GetElement("jType")->Get<std::string>();	
			ifstream ifs(jTypechain.c_str());
			for (unsigned int i=1;i<m.LINKNUM;++i){ //! "_0_0_0_0_0_0_1"
				ifs >> space; //! "_"
				ifs >> m.J_type[i];
			}
			ifs >> space; //! "_" last space 
			ifs.close();
		 }
		 else {m.J_type[m.LINKNUM-1]=1; //! By default, the last joint is prismatic 
			}
		*/
		
		
		
		//! Get the mass of the links
	
		m.link_M[0]=this->base_link_->GetInertial()->GetMass();
		m.link_M[1]=this->arm0_->GetInertial()->GetMass();
		m.link_M[2]=this->arm1_->GetInertial()->GetMass();
		m.link_M[3]=this->arm1_1_->GetInertial()->GetMass();
		m.link_M[4]=this->arm2_->GetInertial()->GetMass();
		m.link_M[5]=this->arm2_1_->GetInertial()->GetMass();
		m.link_M[6]=this->arm3_->GetInertial()->GetMass();
		m.link_M[7]=this->hand_->GetInertial()->GetMass();
		
		//ROS_WARN("masses %f  %f %f %f %f %f %f %f", m.link_M[0], m.link_M[1], m.link_M[2], m.link_M[3], m.link_M[4],m.link_M[5],m.link_M[6],m.link_M[7]);
		
		//!  Get the inertias for each link
		
		ConvInertial(m.link_I[0],this->base_link_);
		ConvInertial(m.link_I[1],this->arm0_);
		ConvInertial(m.link_I[2],this->arm1_);
		ConvInertial(m.link_I[3],this->arm1_1_);
		ConvInertial(m.link_I[4],this->arm2_);
		ConvInertial(m.link_I[5],this->arm2_1_);
		ConvInertial(m.link_I[6],this->arm3_);
		ConvInertial(m.link_I[7],this->hand_);

		
		//! take as reference the notation in the file model.sdf which is the conversion from xacro to sdf
		
			//! Retrieving the JtoC's in Gazebo
	
		math::Vector3 JtoC1_gazebo=this->arm0_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC2_gazebo=this->arm1_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC3_gazebo=this->arm1_1_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC4_gazebo=this->arm2_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC5_gazebo=this->arm2_1_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC6_gazebo=this->arm3_->GetInertial()->GetPose().pos;
		math::Vector3 JtoC7_gazebo=this->hand_->GetInertial()->GetPose().pos;
		
		vConvert(JtoC1_gazebo,m.JtoC[1]);
		vConvert(JtoC2_gazebo,m.JtoC[2]);
		vConvert(JtoC3_gazebo,m.JtoC[3]);
		vConvert(JtoC4_gazebo,m.JtoC[4]);
		vConvert(JtoC5_gazebo,m.JtoC[5]);
		vConvert(JtoC6_gazebo,m.JtoC[6]);
		vConvert(JtoC7_gazebo,m.JtoC[7]);
		
		/*
		ROS_WARN("m.J1toC1 x: %f , y: %f , z: %f",m.JtoC[1][0],m.JtoC[1][1],m.JtoC[1][2]);
		ROS_WARN("m.J2toC2 x: %f , y: %f , z: %f",m.JtoC[2][0],m.JtoC[2][1],m.JtoC[2][2]);
		ROS_WARN("m.J3toC3 x: %f , y: %f , z: %f",m.JtoC[3][0],m.JtoC[3][1],m.JtoC[3][2]);
		ROS_WARN("m.J4toC4 x: %f , y: %f , z: %f",m.JtoC[4][0],m.JtoC[4][1],m.JtoC[4][2]);
		ROS_WARN("m.J5toC5 x: %f , y: %f , z: %f",m.JtoC[5][0],m.JtoC[5][1],m.JtoC[5][2]);
		ROS_WARN("m.J6toC6 x: %f , y: %f , z: %f",m.JtoC[6][0],m.JtoC[6][1],m.JtoC[6][2]);
		ROS_WARN("m.J7toC7 x: %f , y: %f , z: %f",m.JtoC[7][0],m.JtoC[7][1],m.JtoC[7][2]);
		*/
			
		//! Retrieving the CtoJ's in Gazebo
		
		math::Vector3 CtoJ1_gazebo=(this->j1_->GetWorldPose().operator-(this->base_link_->GetWorldInertialPose())).pos; //! In this context j1.POSE.pos ==>arm0.POSE.pos because of internal conversion to SDF;
		math::Vector3 CtoJ2_gazebo=(this->j2_->GetWorldPose().operator-(this->arm0_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ3_gazebo=(this->j3_->GetWorldPose().operator-(this->arm1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ4_gazebo=(this->j4_->GetWorldPose().operator-(this->arm1_1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ5_gazebo=(this->j5_->GetWorldPose().operator-(this->arm2_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ6_gazebo=(this->j6_->GetWorldPose().operator-(this->arm2_1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ7_gazebo=(this->d1_->GetWorldPose().operator-(this->arm3_->GetWorldInertialPose())).pos;
			
			vConvert(CtoJ1_gazebo,m.CtoJ[1]);
			vConvert(CtoJ2_gazebo,m.CtoJ[2]);
			vConvert(CtoJ3_gazebo,m.CtoJ[3]);
			vConvert(CtoJ4_gazebo,m.CtoJ[4]);
			vConvert(CtoJ5_gazebo,m.CtoJ[5]);
			vConvert(CtoJ6_gazebo,m.CtoJ[6]);
			vConvert(CtoJ7_gazebo,m.CtoJ[7]);
		
		/*
		ROS_WARN("m.C0toJ1 x: %f , y: %f , z: %f",m.CtoJ[1][0],m.CtoJ[1][1],m.CtoJ[1][2]);
		ROS_WARN("m.C1toJ2 x: %f , y: %f , z: %f",m.CtoJ[2][0],m.CtoJ[2][1],m.CtoJ[2][2]);
		ROS_WARN("m.C2toJ3 x: %f , y: %f , z: %f",m.CtoJ[3][0],m.CtoJ[3][1],m.CtoJ[3][2]);
		ROS_WARN("m.C3toJ4 x: %f , y: %f , z: %f",m.CtoJ[4][0],m.CtoJ[4][1],m.CtoJ[4][2]);
		ROS_WARN("m.C4toJ5 x: %f , y: %f , z: %f",m.CtoJ[5][0],m.CtoJ[5][1],m.CtoJ[5][2]);
		ROS_WARN("m.C5toJ6 x: %f , y: %f , z: %f",m.CtoJ[6][0],m.CtoJ[6][1],m.CtoJ[6][2]);
		ROS_WARN("m.C6toJ7 x: %f , y: %f , z: %f",m.CtoJ[7][0],m.CtoJ[7][1],m.CtoJ[7][2]);
		*/

		//! CtoE assigned a priori to the unique end effector/tip 
		
		m.CtoE[0][2]=0.02626; //! where the end effector/tip of the hand is located [m] 
		
		
		//! Retrieve the Relative Orientation of the joint w.r.t its Parent's Frame of Reference
		math::Quaternion qi_1_gazebo=(this->j1_->GetWorldPose().operator-(this->base_link_->GetWorldInertialPose())).rot;
		math::Quaternion qi_2_gazebo=(this->j2_->GetWorldPose().operator-(this->j1_->GetWorldPose())).rot;
		math::Quaternion qi_3_gazebo=(this->j3_->GetWorldPose().operator-(this->j2_->GetWorldPose())).rot;
		math::Quaternion qi_4_gazebo=(this->j4_->GetWorldPose().operator-(this->j3_->GetWorldPose())).rot;
		math::Quaternion qi_5_gazebo=(this->j5_->GetWorldPose().operator-(this->j4_->GetWorldPose())).rot;
		math::Quaternion qi_6_gazebo=(this->j6_->GetWorldPose().operator-(this->j5_->GetWorldPose())).rot;
		math::Quaternion qi_7_gazebo=(this->d1_->GetWorldPose().operator-(this->j6_->GetWorldPose())).rot;
		
		ConvEuler(qi_1_gazebo,m.Qi[1]);
		ConvEuler(qi_2_gazebo,m.Qi[2]);
		ConvEuler(qi_3_gazebo,m.Qi[3]);
		ConvEuler(qi_4_gazebo,m.Qi[4]);
		ConvEuler(qi_5_gazebo,m.Qi[5]);
		ConvEuler(qi_6_gazebo,m.Qi[6]);
		ConvEuler(qi_7_gazebo,m.Qi[7]);
		

		/*
		ROS_WARN("Qi_1 x: %f , y: %f , z: %f",m.Qi[1][0],m.Qi[1][1],m.Qi[1][2]);
		ROS_WARN("Qi_2 x: %f , y: %f , z: %f",m.Qi[2][0],m.Qi[2][1],m.Qi[2][2]);
		ROS_WARN("Qi_3  x: %f , y: %f , z: %f",m.Qi[3][0],m.Qi[3][1],m.Qi[3][2]);
		ROS_WARN("Qi_4  x: %f , y: %f , z: %f",m.Qi[4][0],m.Qi[4][1],m.Qi[4][2]);
		ROS_WARN("Qi_5  x: %f , y: %f , z: %f",m.Qi[5][0],m.Qi[5][1],m.Qi[5][2]);
		ROS_WARN("Qi_6  x: %f , y: %f , z: %f",m.Qi[6][0],m.Qi[6][1],m.Qi[6][2]);
		ROS_WARN("Qi_7  x: %f , y: %f , z: %f",m.Qi[7][0],m.Qi[7][1],m.Qi[7][2]);
		*/

		
		for (unsigned int i=0;i<3;++i){
			m.Qe[1][i]=0; //! the hand tip is not rotated with respect to the 
		}
		  calc_SPN( m ); //! convert parameters to spatial notation
	
	}
	
	
	void GazeboRosSPDChaserController::vConvert(const math::Vector3& vgazebo, Vector3& vspd){
		vspd[0]=vgazebo.x; ; vspd[1]=vgazebo.y; vspd[2]=vgazebo.z;
		}
	void GazeboRosSPDChaserController::qConvert(const math::Quaternion& qgazebo, Vector4& qspd){
		qspd[0]=qgazebo.w; qspd[1]=qgazebo.x; qspd[2]=qgazebo.y; qspd[3]=qgazebo.z;
		}
		
	void GazeboRosSPDChaserController::ConvEuler(const math::Quaternion& vqgazebo, Vector3& vqspd){
		Vector4 tempq;
		Matrix3 tempA;
		qConvert(vqgazebo,tempq);
		qtn2dc( tempq, tempA ); //! Convert Quaternion to Transformation Matrix3  A0
		dc2rpy(tempA,vqspd);
		}
		
	void GazeboRosSPDChaserController::ConvInertial(Matrix3& link_I_, const physics::LinkPtr& _link) {
		
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

