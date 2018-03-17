#include <ctime>
#include <cmath>
#include <stdlib.h>
#include <ros/package.h>
#include "spacedyn_integration/chaser_controller.h"
#include <vector>

namespace gazebo {

/**Constructor**/

	GazeboRosSPDChaserController::GazeboRosSPDChaserController()
	{
		//!==========Locate and Load the Model Parameters=============!//
		std::string modelpath= ros::package::getPath("spacedyn_integration"); //added 28.1.17
		model(modelpath + "/config/model_chaser.def", m );  // modified 28.1.17 
        GazeboRosSPDChaserController::initSpd(); //!Initialize SpaceDyn variables and references
       
       //! Create Vector for Impedance Control
  
	}
	
/**Destructor**/
	
	GazeboRosSPDChaserController::~GazeboRosSPDChaserController()
	{
	/******************************SpaceDyn****************************/
			delete[] Gravity;
			delete[] chaser_A0;
			delete[] ORI_e;
			delete[] POS_e;
			//delete[] des_q;
			//delete[] des_qd;
			//delete[] Q0_rad;
			//delete[] Q0_deg;
			//delete[] Qe_rad;
			//delete[] Qe_deg;
			delete[] q_deg;
			delete[] q_vel; 
			
	 //!     IMPULSE BASED VARIABLE IMPEDANCE CONTROL 
			
			delete[] xh;  //! Linear - Angular Position of the end-effector
			delete[] dxh; //! Linear - Angular Velocity of the end effector
			delete[] ddxh;//! Linear - Angular Acceleration of the end effector 
			
			//! Reference trajectories
			delete[] Nd; 
			delete[] Nvd; 
			delete[] Nad; 
	
			delete[] tMi;
			delete[] tDdi;
			delete[] tKdi;
			
			//! Kinematics and Dynamics
			
			delete[] dxgh; 

			delete[] GJe;
			delete[] GJe_prev;
			delete[] dGJe;

			delete[] Je;
			delete[] GH;
			delete[] Gc;

	//!     HYBRID VARIABLE IMPEDANCE - POSITION CONTROL 
			
			delete[] kP;
			delete[] kD;
			delete[] kI;
			
			delete[] tRp; 
			//! Selection matrixes for hybrid control
			delete[] S_imp;
			delete[] S_pos;	
			
			//! Reference trajectories
			delete[] tNd; 
			delete[] tNvd; 
			delete[] tNad; 
			
			delete[] tRI;
			delete[] tCI;
			
			//! Kinematics and Dynamics
			
			delete[] tdxghI;
			delete[] tdxghI_prev;
			delete[] tddxghI; 
			
			delete[] tdxhr;
			delete[] tdxhr_prev;
			delete[] tddxhr;
			
			delete[] tdxhI; 
			
			//! Transformation to the target frame of reference
			
			delete[] tGJeI;
			delete[] tGJeI_prev;
			delete[] tdGJeI;
			
			delete[] tGHI;
			delete[] tGcI;
  			
			m.destructor();
			m.destruct_ee();
			flag.F_SPD = false;
	/******************************************************************/
		
		event::Events::DisconnectWorldUpdateBegin(updateConnection);
		//node_handle_->shutdown();
		//delete node_handle_;
	}

	void GazeboRosSPDChaserController::initSpd()
	{
		/**Global Initialization **/
		
		model_init(m); //!initialize state values of model and environment
		_linknum_chaser=m.LINKNUM; //! Defined for allocating the memory of the matrixes in dataexp.cpp, firstly parsed from model_chaser.def
		matrix_Z(6,1,Gravity);               
		matrix_Z(3,3,chaser_A0);
		matrix_Z(3,3,ORI_e);
		matrix_Z(m.E_NUM,3,POS_e);
		//matrix_Z(_linknum_chaser,1,des_q);
		//matrix_Z(_linknum_chaser,1,des_qd);
		//matrix_Z(3,1,Q0_deg);
		//matrix_Z(3,1,Q0_rad);
		//matrix_Z(3,1,Qe_rad);
		//matrix_Z(3,1,Qe_deg);
		matrix_Z(_linknum_chaser,1,q_deg);
		matrix_Z(_linknum_chaser,1,q_vel);
		
		
		////! For hybrid position-impedance controller
		
		 d_time= 0.0;
		
		////! IMPULSE BASED VARIABLE IMPEDANCE CONTROL
		
		 _linknum_chaser=8;
		
		  //! Hand Position, Velocity and Acceleration
		  
		  matrix_Z(6,1,xh);   //! Linear - Angular Position of the end-effector
		  matrix_Z(6,1,dxh);  //! Linear - Angular Velocity of the end effector
		  matrix_Z(6,1,ddxh); //! Linear - Angular Acceleration of the end effector
		  
		  
		  //! Reference trajectories
		 
		 matrix_Z(6,1,Nd); //! desired reference position
		 matrix_Z(6,1,Nvd);//! desired reference velocity
		 matrix_Z(6,1,Nad);//! desired reference acceleration 
		 
		 //! Contact Characteristics
		
		 Idn=0.0; //! Desired normal impulse
		 Dampd=0.0; //! Desired Damping Ratio (!= Damping Coefficient)
		 Wdn=0.0;  //! Desired Angular Velocity
		 matrix_Z(6,6,tMi); //!desired impedance mass R6X6
		 matrix_Z(6,6,tDdi); //! desired impedance damping R6X6 
		 matrix_Z(6,6,tKdi); //! desired impedance stiffness R6X6
		 
		 tmi=0.0; //!desired impedance mass 1D
		 tddi=0.0;//! desired impedance damping 1D 
		 tkdi=0.0; //! desired impedance stiffness 1D
		
			//! Contact Characteristics
		//! flag_IN_CONTACT_=false; SEE: the package chaser_control/gazebo_ros_bumper_flag.cpp

			//! Relative Motion
		 v_0=0.0; //! initial relative velocity 1D
		 y_0=0.0; //! initial relative position 1D
		 
			//! Compliant Wrist
		 
		//! kw=0.0; //! Stiffness of compliant wrist   : SEE compliant_wrist.cpp
		//! dw=0.0; //! Damping coefficient of compliant wrist : SEE compliant_wrist.cpp
		 
		//! Kinematics and dynamics
		
		 matrix_Z(6,1,dxgh); //! Linear - Angular Velocity from the COG to the Hand
		
		 matrix_Z(6,_linknum_chaser-1,GJe); //!Generalized Jacobian Matrix
 		 matrix_Z(6,_linknum_chaser-1,GJe_prev); //! Generalized Jacobian Matrix previous iteration
		 matrix_Z(6,_linknum_chaser-1,dGJe); //!Time Derivative Generalized Jacobian Matrix

		 matrix_Z(6,_linknum_chaser-1,Je);  //!Jacobian Matrix for the End Effector
		 matrix_Z(_linknum_chaser-1,_linknum_chaser-1,GH);  //!Generalized Inertia Matrix
		 matrix_Z(6,1,Gc);  //!Generalized non-linear velocity dependent matrix		 

		 //! HYBRID VARIABLE IMPEDANCE - POSITION CONTROL
		 
		 matrix_Z(6,6,kP);
		 matrix_Z(6,6,kD);
		 matrix_Z(6,6,kI);
		 
		 matrix_Z(3,1,tRp); //! Contact Point
		 
		 
		 //! Selection matrix for hybrid control
		 matrix_Z(6,1,S_imp);
		 matrix_Z(5,6,S_pos); 
		 
		 //! Reference trajectories
		 
		 vector_z(6,tNd); //! desired reference position w.r.t. target
		 vector_z(6,tNvd);//! desired reference velocity w.r.t target 
		 vector_z(6,tNad);//! desired reference acceleration w.r.t target
		 
		 matrix_Z(6,6,tRI); //! Transformation Matrix from Inertia to Target
		 matrix_Z(3,3,tCI); //! Cosine Direction Matrix from EI to Et
		 
		 //! Kinematics and dynamics
		 
		 matrix_Z(6,1,tdxghI); //! dxgh measured from the target frame of reference
		 matrix_Z(6,1,tdxghI_prev); //! last measurement of tdxghI
		 matrix_Z(6,1,tddxghI); //! xgh acceleration measured from the target frame of reference
		 
		 matrix_Z(6,1,tdxhr); //! Linear - Angular Velocity from the influence of the angular velocity of the target.
 		 matrix_Z(6,1,tdxhr_prev); //! Previous Iteration tdxhr
		 matrix_Z(6,1,tddxhr); //! acceleration for xhr

		 matrix_Z(6,1,tdxhI); //! total linear-angular velocity of the hand measured from the target

		//! Transformation to the target frame of reference
		
		 matrix_Z(6,_linknum_chaser-1,tGJeI); //! Generalized Jacobian for the end effector measured in the target frame of reference
 		 matrix_Z(6,_linknum_chaser-1,tGJeI_prev);
		 matrix_Z(6,_linknum_chaser-1,tdGJeI); //! d(J*)ij / d_timeatrix

		 matrix_Z(_linknum_chaser-1,_linknum_chaser-1,tGHI); //! Generalized Inertia Matrix measured in the target frame of reference;
		 matrix_Z(6,1,tGcI); //! Generalized non-linear velocity term measured from target frame of reference

		//! ============ CONTROL WITH INCERTIA UNCERTAINTY =======//
		 Em=0.0; //! Mass Estimation Error;
		 EI=0.0; //! Inertia Estimation Error;
		 
		 ttest=0.0; //! time of estimation of inertia
		 
		//! References for plotting the data:
			//!Desired Angles [rad]
		//	des_q[0] = deg2pi(-90); //! desired joint position j1
		//	des_q[1] = deg2pi(90); //! desired joint position j2
			//! Desired Speed[rad/s]
		//	des_qd[0] = 0; //! desired joint velocity
		//	des_qd[1] = 0; //! desired joint velocity
		//    matrix_trans(3,3,m.A0,A0);
		//    dc2rpy(A0,Q0_rad);
	}

	/** Loading information from the SDF **/

	void GazeboRosSPDChaserController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{ 
	  if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
		}

	  this->ee_world_pose_=math::Pose(0.0,0.0,0.0,0.0,0.0,0.0);
	  d_time=0.0;
	  
	  //if(_sdf->HasElement("withTARGET")) { 
	  	  //_sdf->GetElement("withTARGET")->GetValue()->Get<bool>(flag_TARGET_);
		//}
	  
	  
	  //! Aim all the pointers to the elements in Gazebo
	  this->world_=_model->GetWorld();
	  ROS_INFO("Loading the SpaceDyn Plugin for Chaser");
	  this->parent_=_model;
	  this->base_link_=parent_->GetLink("base_footprint");
	  this->arm0_=parent_->GetLink("arm0");
	  this->arm1_=parent_->GetLink("arm1");
	  this->arm1_1_=parent_->GetLink("arm1_1");
	  this->arm2_=parent_->GetLink("arm2");
	  this->arm2_1_=parent_->GetLink("arm2_1");
	  this->arm3_=parent_->GetLink("arm3");
	  this->hand_=parent_->GetLink("hand");
	  this->j1_=parent_->GetJoint("j1");
	  this->j2_=parent_->GetJoint("j2");
	  this->j3_=parent_->GetJoint("j3");
	  this->j4_=parent_->GetJoint("j4");
	  this->j5_=parent_->GetJoint("j5");
	  this->j6_=parent_->GetJoint("j6");
	  this->d1_=parent_->GetJoint("d1");
	  GazeboRosSPDChaserController::ModelLoad(this->parent_, m, _sdf); //! Map model from gazebo to MODEL m in model.h
	  GazeboRosSPDChaserController::ImpParamLoad(_sdf);
	  last_time_=this->world_->GetSimTime();
	  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  boost::bind(&GazeboRosSPDChaserController::OnUpdate, this, _1));
	}

	// Update in every simulation step
	void GazeboRosSPDChaserController::OnUpdate(const common::UpdateInfo & )
	{

		boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchornization Mechanism of the Different Threads
		//!static bool flag_INIT = true;
		static unsigned int times = 0;	
		//! Get simulation time
		this->sim_time_ = this->world_->GetSimTime();
		d_time = (this->sim_time_ - this->last_time_).Double();
		if (d_time == 0.0) return;
		
		//! Simulation Cycle
		//! f_dyn_rk(m,Gravity,d_time); //! spacedyn LEAVE DISABLED
		GazeboRosSPDChaserController::qMapping(m, d_time); 
		//! Mapping of the positions of the joints and state of the base.
		// if (flag_INIT==true){ //!execute only once
		//	flag_INIT=false;
		//}
		
		
		calc_SP(m); //!Calc pos. & ori. of each joint w.r.t. link's fixed frame;
        //! Joint Position
        f_kin_j(m);
        //! Calc pos. & ori. of the selected end-effector
        f_kin_e(m,m.E_NUM); //! Calculate position and orientation of end effector (tip of hand)
        
        
		
		//! Saving the Pose (position - orientation RPY) of the end-effector into the global variable
		for (unsigned int i=0; i<3; ++i){
			xh[i]=m.POS_e[m.E_NUM][i];
			xh[3+i]=m.Qe[m.E_NUM][i]; //! Relative RPY - Space Dyn
		}
       
        if (flag_TARGET_){
			//ROS_WARN("AQUI");
			GazeboRosSPDChaserController::ImpulseController(m, mt, this->parent_);
        }
     

        
       //! for debugging purposes
       //  ROS_WARN("angles q1 1=%f, 2=%f, 3=%f,4=%f,5=%f,6=%f,7=%f,8=%f",m.q[0],m.q[1],m.qd[2],m.q[3],m.q[4],m.q[5],m.q[6],m.q[7]);
		if (times == 0) {
			
			math::Vector3 posbase=this->base_link_->GetWorldInertialPose().pos; 
			math::Vector3 posj1=this->j1_->GetWorldPose().pos; //! Remember that after SDF conversion, the link pose is actually the joint pose
			math::Vector3 posj2=this->j2_->GetWorldPose().pos;
			math::Vector3 posj3=this->j3_->GetWorldPose().pos;
			math::Vector3 posj4=this->j4_->GetWorldPose().pos;
			math::Vector3 posj5=this->j5_->GetWorldPose().pos;
			math::Vector3 posj6=this->j6_->GetWorldPose().pos;
			math::Vector3 posd1=this->d1_->GetWorldPose().pos;
			
			if (flag_TARGET_){
				ROS_WARN("position spacedyn of target x:%f, y:%f, z:%f",mt.POS0[0],mt.POS0[1],mt.POS0[2]);
			}
				ROS_WARN("position gazebo of base x:%f, y:%f, z:%f",posbase[0],posbase[1],posbase[2]);
				ROS_WARN("position spacedyn of base x:%f, y:%f, z:%f",m.POS0[0],m.POS0[1],m.POS0[2]);
				ROS_WARN("position gazebo of j1 x:%f, y:%f, z:%f",posj1[0],posj1[1],posj1[2]);
				ROS_WARN("position spacedyn of j1 x:%f, y:%f, z:%f",m.POS_j[1][0],m.POS_j[1][1],m.POS_j[1][2]);
				ROS_WARN("position gazebo of j2 x:%f, y:%f, z:%f",posj2[0],posj2[1],posj2[2]);
				ROS_WARN("position spacedyn of j2 x:%f, y:%f, z:%f",m.POS_j[2][0],m.POS_j[2][1],m.POS_j[2][2]);
				ROS_WARN("position gazebo of j3 x:%f, y:%f, z:%f",posj3[0],posj3[1],posj3[2]);
				ROS_WARN("position spacedyn of j3 x:%f, y:%f, z:%f",m.POS_j[3][0],m.POS_j[3][1],m.POS_j[3][2]);
				ROS_WARN("position gazebo of j4 x:%f, y:%f, z:%f",posj4[0],posj4[1],posj4[2]);
				ROS_WARN("position spacedyn of j4 x:%f, y:%f, z:%f",m.POS_j[4][0],m.POS_j[4][1],m.POS_j[4][2]);
				ROS_WARN("position gazebo of j5 x:%f, y:%f, z:%f",posj5[0],posj5[1],posj5[2]);
				ROS_WARN("position spacedyn of j5 x:%f, y:%f, z:%f",m.POS_j[5][0],m.POS_j[5][1],m.POS_j[5][2]);
				ROS_WARN("position gazebo of j6 x:%f, y:%f, z:%f",posj6[0],posj6[1],posj6[2]);
				ROS_WARN("position spacedyn of j6 x:%f, y:%f, z:%f",m.POS_j[6][0],m.POS_j[6][1],m.POS_j[6][2]);
				ROS_WARN("position gazebo of j7 x:%f, y:%f, z:%f",posd1[0],posd1[1],posd1[2]);
				ROS_WARN("position spacedyn of j7 x:%f, y:%f, z:%f",m.POS_j[7][0],m.POS_j[7][1],m.POS_j[7][2]);
				ROS_WARN("position gazebo of tiphand x:%f, y:%f, z:%f",ee_world_pose_.pos[0],ee_world_pose_.pos[1],ee_world_pose_.pos[2]);
				ROS_WARN("position spacedyn of tiphand x:%f, y:%f, z:%f",m.POS_e[1][0],m.POS_e[1][1],m.POS_e[1][2]);
				ROS_WARN("force at tip: x: %f y:%f z:%f",m.Fe[1][0],m.Fe[1][1],m.Fe[1][2]);
				ROS_WARN("torque at tip: x: %f y:%f z:%f",m.Te[1][0],m.Te[1][1],m.Te[1][2]);
			//matrix_print( 3, 1, m.POS_j[7] ); //! Print in screen the position of the end effector index 1 out of e_num
			times=1;
		}
			//matrix_print( 3, 3, m.ORI_e[1] ); //! Print in screen the orientation of the end effector
			//m.tau[1] = kd*(des_qd[0]-m.qd[1])+kp*(des_q[0]-m.q[1]);
			//m.tau[2] = kd*(des_qd[1]-m.qd[2])+kp*(des_q[1]-m.q[2]);
		//!
     
       /**
        // TREATMENT OF DATA BEFORE SAVING FOR SAVE QE AND POSE
        
        matrix_trans(3,3,m.A0,A0); //! A0=m.A0' ; dim(A0)=3x3
        dc2rpy(A0,Q0_rad); // SV.q0(3) == Q0_rad[2]

        matrix_trans(3,3,m.ORI_e[1],ORI_e);
        dc2rpy(ORI_e, Qe_rad);
        for(int i = 0; i < 3; i++){
            Qe_deg[i] = pi2deg(Qe_rad[i]);
            Q0_deg[i] = pi2deg(Q0_rad[i]);
            POS_e[i] = m.POS_e[1][i];
        }
	**/
        //
        //q_deg[0] = pi2deg(m.q[1]);  //! For saving information look at datasave.cpp  //
        //q_deg[1] = pi2deg(m.q[2]);

        // 
        //q_vel[0] = pi2deg(m.qd[1]);
        //q_vel[1] = pi2deg(m.qd[2]);

        //=== Output simulation data to file ===//
        //======================================//
        //if(flag.F_SD == true){// flag.F_SD to be able to save "datasave.cpp"
        //    if(data_counter % 1 == 0){// ¨data_counter % 10 == 0 
        //        dataSave.saveData(time);
        //    }
        //}
        
		last_time_ = sim_time_;  

    };
	GZ_REGISTER_MODEL_PLUGIN(GazeboRosSPDChaserController)
}
// --- EOF ---

