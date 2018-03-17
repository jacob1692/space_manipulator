//! Source file that extracts the vectors of the kinematic chain in the robot from Gazebo to SpaceDyn CtoJ and JtoC

#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/chaser_controller.h"
#include <vector>

namespace gazebo {

	void GazeboRosSpaceDyn::ModelLoad( MODEL &m)   //! MAPPING PARAMETERS GAZEBO <--> SPACE DYN 
	{   
		  
	/*  math::Pose axis_j1_=j1_->GetInitialAnchorPose();
	  math::Pose axis_j2_=j2_->GetInitialAnchorPose();
	  math::Pose axis_j3_=j3_->GetInitialAnchorPose();
	  math::Pose axis_j4_=j4_->GetInitialAnchorPose();
	  math::Pose axis_j5_=j5_->GetInitialAnchorPose();
	  math::Pose axis_j6_=j6_->GetInitialAnchorPose();
	  math::Pose axis_d1_=d1_->GetInitialAnchorPose();
	  ROS_WARN("anchor j1, x,y,z: %f ,%f ,%f",axis_j1_.pos.x,axis_j1_.pos.y,axis_j1_.pos.z);
	  ROS_WARN("anchor j2, x,y,z: %f ,%f ,%f",axis_j2_.pos.x,axis_j2_.pos.y,axis_j2_.pos.z);
	  ROS_WARN("anchor j3, x,y,z: %f ,%f ,%f",axis_j3_.pos.x,axis_j3_.pos.y,axis_j3_.pos.z);
	  ROS_WARN("anchor j4, x,y,z: %f ,%f ,%f",axis_j4_.pos.x,axis_j4_.pos.y,axis_j4_.pos.z);
	  ROS_WARN("anchor j5, x,y,z: %f ,%f ,%f",axis_j5_.pos.x,axis_j5_.pos.y,axis_j5_.pos.z);
	  ROS_WARN("anchor j6, x,y,z: %f ,%f ,%f",axis_j6_.pos.x,axis_j6_.pos.y,axis_j6_.pos.z);
	  ROS_WARN("anchor j7, x,y,z: %f ,%f ,%f",axis_d1_.pos.x,axis_d1_.pos.y,axis_d1_.pos.z);
	  */

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
		
		ROS_WARN("m.J1toC1 x: %f , y: %f , z: %f",m.JtoC[1][0],m.JtoC[1][1],m.JtoC[1][2]);
		ROS_WARN("m.J2toC2 x: %f , y: %f , z: %f",m.JtoC[2][0],m.JtoC[2][1],m.JtoC[2][2]);
		ROS_WARN("m.J3toC3 x: %f , y: %f , z: %f",m.JtoC[3][0],m.JtoC[3][1],m.JtoC[3][2]);
		ROS_WARN("m.J4toC4 x: %f , y: %f , z: %f",m.JtoC[4][0],m.JtoC[4][1],m.JtoC[4][2]);
		ROS_WARN("m.J5toC5 x: %f , y: %f , z: %f",m.JtoC[5][0],m.JtoC[5][1],m.JtoC[5][2]);
		ROS_WARN("m.J6toC6 x: %f , y: %f , z: %f",m.JtoC[6][0],m.JtoC[6][1],m.JtoC[6][2]);
		ROS_WARN("m.J7toC7 x: %f , y: %f , z: %f",m.JtoC[7][0],m.JtoC[7][1],m.JtoC[7][2]);
		
			
		//! Retrieving the CtoJ's in Gazebo
		//math::Vector3 CtoJ0_gazebo=(this->arm0_->GetInertial()->GetPose().pos).operator-(); //! small offset in the base
		math::Vector3 CtoJ1_gazebo=(this->arm0_->GetWorldPose().operator-(this->base_link_->GetWorldInertialPose())).pos; //! In this context j1.POSE.pos ==>arm0.POSE.pos because of internal conversion to SDF;
		math::Vector3 CtoJ2_gazebo=(this->arm1_->GetWorldPose().operator-(this->arm0_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ3_gazebo=(this->arm1_1_->GetWorldPose().operator-(this->arm1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ4_gazebo=(this->arm2_->GetWorldPose().operator-(this->arm1_1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ5_gazebo=(this->arm2_1_->GetWorldPose().operator-(this->arm2_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ6_gazebo=(this->arm3_->GetWorldPose().operator-(this->arm2_1_->GetWorldInertialPose())).pos;
		math::Vector3 CtoJ7_gazebo=(this->hand_->GetWorldPose().operator-(this->arm3_->GetWorldInertialPose())).pos;
			
			vConvert(CtoJ1_gazebo,m.CtoJ[1]);
			vConvert(CtoJ2_gazebo,m.CtoJ[2]);
			vConvert(CtoJ3_gazebo,m.CtoJ[3]);
			vConvert(CtoJ4_gazebo,m.CtoJ[4]);
			vConvert(CtoJ5_gazebo,m.CtoJ[5]);
			vConvert(CtoJ6_gazebo,m.CtoJ[6]);
			vConvert(CtoJ7_gazebo,m.CtoJ[7]);
		
		ROS_WARN("m.C0toJ1 x: %f , y: %f , z: %f",m.CtoJ[1][0],m.CtoJ[1][1],m.CtoJ[1][2]);
		ROS_WARN("m.C1toJ2 x: %f , y: %f , z: %f",m.CtoJ[2][0],m.CtoJ[2][1],m.CtoJ[2][2]);
		ROS_WARN("m.C2toJ3 x: %f , y: %f , z: %f",m.CtoJ[3][0],m.CtoJ[3][1],m.CtoJ[3][2]);
		ROS_WARN("m.C3toJ4 x: %f , y: %f , z: %f",m.CtoJ[4][0],m.CtoJ[4][1],m.CtoJ[4][2]);
		ROS_WARN("m.C4toJ5 x: %f , y: %f , z: %f",m.CtoJ[5][0],m.CtoJ[5][1],m.CtoJ[5][2]);
		ROS_WARN("m.C5toJ6 x: %f , y: %f , z: %f",m.CtoJ[6][0],m.CtoJ[6][1],m.CtoJ[6][2]);
		ROS_WARN("m.C6toJ7 x: %f , y: %f , z: %f",m.CtoJ[7][0],m.CtoJ[7][1],m.CtoJ[7][2]);
		

		//! CtoE assigned a priori to the unique end effector/tip 
		
		m.CtoE[1][2]=0.02626; //! where the end effector/tip of the hand is located [m] 
		
		
		//! Retrieve the Relative Orientation of the joint w.r.t its Parent's Frame of Reference
		math::Vector3 qi_1_gazebo=this->j1_->GetAxisFrameOffset(2).operator*(this->arm0_->GetWorldPose().operator-(this->base_link_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_2_gazebo=this->j2_->GetAxisFrameOffset(3).operator*(this->arm1_->GetWorldPose().operator-(this->arm0_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_3_gazebo=this->j3_->GetAxisFrameOffset(4).operator*(this->arm1_1_->GetWorldPose().operator-(this->arm1_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_4_gazebo=this->j4_->GetAxisFrameOffset(5).operator*(this->arm2_->GetWorldPose().operator-(this->arm1_1_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_5_gazebo=this->j5_->GetAxisFrameOffset(6).operator*(this->arm2_1_->GetWorldPose().operator-(this->arm2_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_6_gazebo=this->j6_->GetAxisFrameOffset(2).operator*(this->arm3_->GetWorldPose().operator-(this->arm2_1_->GetWorldPose()).rot).GetAsEuler();
		math::Vector3 qi_7_gazebo=this->d1_->GetAxisFrameOffset(2).operator*(this->hand_->GetWorldPose().operator-(this->arm3_->GetWorldPose()).rot).GetAsEuler();
		
		vConvert(qi_1_gazebo,m.Qi[1]);
		vConvert(qi_2_gazebo,m.Qi[2]);
		vConvert(qi_3_gazebo,m.Qi[3]);
		vConvert(qi_4_gazebo,m.Qi[4]);
		vConvert(qi_5_gazebo,m.Qi[5]);
		vConvert(qi_6_gazebo,m.Qi[6]);
		vConvert(qi_7_gazebo,m.Qi[7]);
		
		
		ROS_WARN("Qi_1 x: %f , y: %f , z: %f",m.Qi[1][0],m.Qi[1][1],m.Qi[1][2]);
		ROS_WARN("Qi_2 x: %f , y: %f , z: %f",m.Qi[2][0],m.Qi[2][1],m.Qi[2][2]);
		ROS_WARN("Qi_3  x: %f , y: %f , z: %f",m.Qi[3][0],m.Qi[3][1],m.Qi[3][2]);
		ROS_WARN("Qi_4  x: %f , y: %f , z: %f",m.Qi[4][0],m.Qi[4][1],m.Qi[4][2]);
		ROS_WARN("Qi_5  x: %f , y: %f , z: %f",m.Qi[5][0],m.Qi[5][1],m.Qi[5][2]);
		ROS_WARN("Qi_6  x: %f , y: %f , z: %f",m.Qi[6][0],m.Qi[6][1],m.Qi[6][2]);
		ROS_WARN("Qi_7  x: %f , y: %f , z: %f",m.Qi[7][0],m.Qi[7][1],m.Qi[7][2]);
		
		
		for (unsigned int i=0;i<3;++i){
			m.Qe[1][i]=0; //! the hand tip is not rotated with respect to the 
		}
		
		//! FOR DEBUGGING PURPOSES
		
		math::Quaternion quat1 = this->j1_->GetAxisFrameOffset(2);
		math::Quaternion quat2 = this->j2_->GetAxisFrameOffset(3);
		math::Quaternion quat3 = this->j3_->GetAxisFrameOffset(4);
		ROS_WARN("GetAxisFrameOffset1 x: %f , y: %f , z: %f , w: %f",quat1.x,quat1.y,quat1.z,quat1.w);
		ROS_WARN("GetAxisFrameOffset2 x: %f , y: %f , z: %f , w: %f",quat2.x,quat2.y,quat2.z,quat2.w);
		ROS_WARN("GetAxisFrameOffset3 x: %f , y: %f , z: %f , w: %f",quat3.x,quat3.y,quat3.z,quat3.w);
		
		  calc_SPN( m ); //! convert parameters to spatial notation
	}
	
	void GazeboRosSpaceDyn::vConvert(const math::Vector3& vgazebo, Vector3& vspd){
		vspd[0]=vgazebo.x; vspd[1]=vgazebo.y; vspd[2]=vgazebo.z;
		}
	
}
