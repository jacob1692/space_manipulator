/* copyright (C) 2017 <tohoku university/jacob hernandez> */
/* license BSD */
//! Plugin for changing between individual PID joint for compliant pasiveness during navigation and 
//! the hybrid control before contact

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "chaser_control/gazebo_joints_PID.h"


namespace gazebo 
{

  GazeboJointsPID::GazeboJointsPID() {
	  bool flag_ENABLE_HYBCTRL_ = false;
	  }
  GazeboJointsPID::~GazeboJointsPID() {}
	  
  // Load the controller
  void GazeboJointsPID::Load(physics::ModelPtr _model, 
      sdf::ElementPtr sdf) 
  {
	
    parent_ = _model;
    j1_=parent_->GetJoint("j1"); //! axis 2  //! axis 1 is the fixed base_joint between base_footprint and base_link
    j2_=parent_->GetJoint("j2"); //! axis 3
    j3_=parent_->GetJoint("j3"); //! axis 4
    j4_=parent_->GetJoint("j4"); //! axis 5
    j5_=parent_->GetJoint("j5"); //! axis 6
    j6_=parent_->GetJoint("j6"); //! axis 7, //! axis 8 is the fixed jointfor the camera 
    //! d1_=parent_->GetJoint("d1"); //! Compliant Wrist
    
    Jc1_= new physics::JointController(parent_);
    Jc2_= new physics::JointController(parent_);
    Jc3_= new physics::JointController(parent_);
    Jc4_= new physics::JointController(parent_);
    Jc5_= new physics::JointController(parent_);
    Jc6_= new physics::JointController(parent_);
    
    
    /* Parse parameters */
    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("JointsPID missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
    robot_namespace_ = 
       sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "joy";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("JointsPID (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }
    //!Initialization of private variables;
    std::string j1PID_str, j2PID_str, j3PID_str, j4PID_str, j5PID_str, j6PID_str; 
    
        /** PID controllers parameters **/
		PIDj1_=math::Vector4(0.0,0.0,0.0,0.0);
		PIDj2_=math::Vector4(0.0,0.0,0.0,0.0);
		PIDj3_=math::Vector4(0.0,0.0,0.0,0.0);
		PIDj4_=math::Vector4(0.0,0.0,0.0,0.0);
		PIDj5_=math::Vector4(0.0,0.0,0.0,0.0);
		PIDj6_=math::Vector4(0.0,0.0,0.0,0.0); 
		  
    /**                                       **/  
    std::stringstream j1PID_stream, j2PID_stream, j3PID_stream, j4PID_stream, j5PID_stream, j6PID_stream; 

    //! "kp ki kd i_clamp"     
    j1PID_str="1000 0.01 0.1 1000";
    j2PID_str="1000 0.01 10 1000";
    j3PID_str="100 0.01 0.1 1000";
    j4PID_str="1000 0.01 0.1 1000";
    j5PID_str="100 0.01 0.1 1000";
    j6PID_str="1000 0.01 0.1 1000";
    
	if (sdf->HasElement("j1KpKdKi")){
			sdf->GetElement("j1KpKdKi")->GetValue()->Get<std::string>(j1PID_str);
	}
	
	if (sdf->HasElement("j2KpKdKi")){
			sdf->GetElement("j2KpKdKi")->GetValue()->Get<std::string>(j2PID_str);
	}
	
	if (sdf->HasElement("j3KpKdKi")){
			sdf->GetElement("j3KpKdKi")->GetValue()->Get<std::string>(j3PID_str);
	}	
	
	if (sdf->HasElement("j4KpKdKi")){
			sdf->GetElement("j4KpKdKi")->GetValue()->Get<std::string>(j4PID_str);
	}	
	
	if (sdf->HasElement("j5KpKdKi")){
			sdf->GetElement("j5KpKdKi")->GetValue()->Get<std::string>(j5PID_str);
	}	
	
	if (sdf->HasElement("j6KpKdKi")){
			sdf->GetElement("j6KpKdKi")->GetValue()->Get<std::string>(j6PID_str);
	}	
	
	j1PID_stream.str(j1PID_str.c_str()); //! Convert the strings in to a stream
	j2PID_stream.str(j2PID_str.c_str());
	j3PID_stream.str(j3PID_str.c_str());
	j4PID_stream.str(j4PID_str.c_str());
	j5PID_stream.str(j5PID_str.c_str());
	j6PID_stream.str(j6PID_str.c_str());

    j1PID_stream>>PIDj1_.x>>PIDj1_.y>>PIDj1_.z>>PIDj1_.w;
    j2PID_stream>>PIDj2_.x>>PIDj2_.y>>PIDj2_.z>>PIDj2_.w;
    j3PID_stream>>PIDj3_.x>>PIDj3_.y>>PIDj3_.z>>PIDj3_.w;
    j4PID_stream>>PIDj4_.x>>PIDj4_.y>>PIDj4_.z>>PIDj4_.w;
    j5PID_stream>>PIDj5_.x>>PIDj5_.y>>PIDj5_.z>>PIDj5_.w;
    j6PID_stream>>PIDj6_.x>>PIDj6_.y>>PIDj6_.z>>PIDj6_.w;
    
	//! DEBUG
		ROS_INFO("the PID values of joint2 are: %f, %f, %f, %f",PIDj2_.x,PIDj2_.y,PIDj2_.z,PIDj2_.w);
  
		bax_change_ctrl1_=2; //! XBOX: Left Rear Button 
		bax_change_ctrl2_=5; //! XBOX: Right Rear Button 
		
		//!Initialization of persistent states for buttons
		
		p_b_ctrl1_=0; p_b_ctrl2_=0;
		
		//! Initialization of flags;
		flag_CH_CTRL_=false;
    
   
	//!Retrieve the joystick axes configuration from the SDF file 
      
    if (sdf->HasElement("joyBAxisChangeCtrl1")){
      sdf->GetElement("joyBAxisChangeCtrl1")->GetValue()->Get(bax_change_ctrl1_);      
	}
	//! Retrieve the scale factors for the angular and linear spans
	
	if (sdf->HasElement("joyBAxisChangeCtrl2")){
      sdf->GetElement("joyBAxisChangeCtrl2")->GetValue()->Get(bax_change_ctrl2_);      
	}
    // Ensure that ROS has been initialized and subscribe to joy
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("JointsPID (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_INFO(" GazeboJointsPID (%s) has been loaded!", 
        robot_namespace_.c_str());

   //! PID	(	double 	_p = 0.0, double 	_i = 0.0, double 	_d = 0.0, double 	_imax = 0.0, double 	_imin = 0.0, double 	_cmdMax = 0.0, double 	_cmdMin = 0.0  )	
	 //this->_pid1=common::PID(); this->_pid2=common::PID(); this->_pid3=common::PID(); this->_pid4=common::PID(); this->_pid5=common::PID(); this->_pid6=common::PID(); 
	 
	 //this->_pid1=common::PID(PIDj1_[0],PIDj1_[1],PIDj1_[2],-PIDj1_[3],PIDj1_[3],this->j1_->GetLowerLimit(2).Radian(),this->j1_->GetUpperLimit(2).Radian());
	 //this->_pid2=common::PID(PIDj2_[0],PIDj2_[1],PIDj2_[2],-PIDj2_[3],PIDj2_[3],this->j2_->GetLowerLimit(3).Radian(),this->j2_->GetUpperLimit(3).Radian()); 
	 //this->_pid3=common::PID(PIDj3_[0],PIDj3_[1],PIDj3_[2],-PIDj3_[3],PIDj3_[3],this->j3_->GetLowerLimit(4).Radian(),this->j3_->GetUpperLimit(4).Radian()); 
	 //this->_pid4=common::PID(PIDj4_[0],PIDj4_[1],PIDj4_[2],-PIDj4_[3],PIDj4_[3],this->j4_->GetLowerLimit(5).Radian(),this->j4_->GetUpperLimit(5).Radian()); 
	 //this->_pid5=common::PID(PIDj5_[0],PIDj5_[1],PIDj5_[2],-PIDj5_[3],PIDj5_[3],this->j5_->GetLowerLimit(6).Radian(),this->j5_->GetUpperLimit(6).Radian()); 
	 //this->_pid6=common::PID(PIDj6_[0],PIDj6_[1],PIDj6_[2],-PIDj6_[3],PIDj6_[3],this->j6_->GetLowerLimit(7).Radian(),this->j6_->GetUpperLimit(7).Radian()); 

		//this->Jc1_->SetPositionPID ("j1",_pid1);
		//this->Jc2_->SetPositionPID ("j2",_pid2);
		//this->Jc3_->SetPositionPID ("j3",_pid3);
		//this->Jc4_->SetPositionPID ("j4",_pid4);
		//this->Jc5_->SetPositionPID ("j5",_pid5);
		//this->Jc6_->SetPositionPID ("j6",_pid6);
	
	
    // subscribe to the joystick
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<sensor_msgs::Joy>(command_topic_, 1,
          boost::bind(&GazeboJointsPID::JoyCallback, this, _1),
          ros::VoidPtr(), &queue_);
    joy_sub_ = rosnode_->subscribe(so);

     //start custom queue
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboJointsPID::QueueThread, this));
	
	//this->last_time_ = parent_->GetWorld()->GetSimTime();
    
    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboJointsPID::OnUpdate, this));
    
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Update the controller
  void GazeboJointsPID::OnUpdate() 
  {
	
	static int n_persist = 5;
	static int times=0;
    boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchronization Mechanism of the Different Threads
    common::Time current_time = parent_->GetWorld()->GetSimTime();
    math::Pose pose = parent_->GetWorldPose();
   
   if ( (p_b_ctrl1_>=n_persist) &&  (p_b_ctrl2_>=n_persist) ) { 
	    this->flag_CH_CTRL_=!this->flag_CH_CTRL_;
		p_b_ctrl1_=p_b_ctrl2_=0;
		if (this->flag_CH_CTRL_==false){
			ROS_INFO("THE JOINTS ARE IN INDIVIDUAL STIFF CONTROL");
		}
		if (this->flag_CH_CTRL_==true){
			ROS_INFO("THE ROBOT IS IN HYBRID IMPEDANCE - POSITION CONTROL"); 
		}
	}
	common::Time _dt = current_time - last_time_;
	if (this->flag_CH_CTRL_==false)
		{
			if (times==0){
				ROS_INFO("INDIVIDUAL JOINTS POSITION CONTROL");
				ROS_INFO( "JC1KP: %f " , PIDj1_.x);
			}
		//physics::joint::SetPositionTarget("j1",0.0); //! Radians
		//this->_pid2.SetPositionTarget("j2",0.0); //! Radians
		//this->_pid3.SetPositionTarget("j3",0.0); //! Radians
		//this->_pid4.SetPositionTarget("j4",0.0); //! Radians
		//this->_pid5.SetPositionTarget("j5",0.0); //! Radians
		//this->_pid6.SetPositionTarget("j6",0.0); //! Radians
		
		
		//this->_pid1.Update(-this->j1_->GetTorque(2),_dt); //! Radians
		//this->_pid2.Update(-this->j2_->GetTorque(3),_dt); //! Radians
		//this->_pid3.Update(-this->j3_->GetTorque(4),_dt); //! Radians
		//this->_pid4.Update(-this->j4_->GetTorque(5)._dt); //! Radians
		//this->_pid5.Update(-this->j5_->GetTorque(6),_dt); //! Radians
		//this->_pid6.Update(-this->j6_->GetTorque(7)); //! Radians
		
		
		//this->Jc1_->SetPositionTarget("j1",0.0); //! Radians
		//this->Jc2_->SetPositionTarget("j2",0.0); //! Radians
		//this->Jc3_->SetPositionTarget("j3",0.0); //! Radians
		//this->Jc4_->SetPositionTarget("j4",0.0); //! Radians
		//this->Jc5_->SetPositionTarget("j5",0.0); //! Radians
		//this->Jc6_->SetPositionTarget("j6",0.0); //! Radians
		
		double t1= -this->j1_->GetAngle(2).Radian()*PIDj1_.x - this->j1_->GetVelocity(2)*PIDj1_.z;
		double t2= -this->j2_->GetAngle(3).Radian()*PIDj2_.x - this->j2_->GetVelocity(3)*PIDj2_.z;
		double t3= -this->j3_->GetAngle(4).Radian()*PIDj3_.x - this->j3_->GetVelocity(4)*PIDj3_.z;
		double t4= -this->j4_->GetAngle(5).Radian()*PIDj4_.x - this->j4_->GetVelocity(5)*PIDj4_.z;
		double t5= -this->j5_->GetAngle(6).Radian()*PIDj5_.x - this->j5_->GetVelocity(6)*PIDj5_.z;
		double t6= -this->j6_->GetAngle(7).Radian()*PIDj6_.x - this->j6_->GetVelocity(7)*PIDj6_.z;

		
		this->j1_->SetForce(2,t1*0.001); 
		this->j2_->SetForce(3,t2*0.001); 
		this->j3_->SetForce(4,t3*0.001); 
		this->j4_->SetForce(5,t4*0.001); 
		this->j5_->SetForce(6,t5*0.001); 
		this->j6_->SetForce(7,t6*0.001); 
		
			
		
		
		//this->j2_->SetPosition(3,0.0); //! Radians
		//this->j3_->SetPosition(4,0.0); //! Radians
		//this->j4_->SetPosition(5,0.0); //! Radians
		//this->j5_->SetPosition(6,0.0); //! Radians
		//this->j6_->SetPosition(7,0.0); //! Radians
		
		//this->j1_->Update();
		//this->j2_->Update();
		//this->j3_->Update();
		//this->j4_->Update();
		//this->j5_->Update();
		//this->j6_->Update();
		}
	
	flag_ENABLE_HYBCTRL_=this->flag_CH_CTRL_; //! Global variable for the hybrid gazebo-spacedyn platform see: dataext.cpp
	times=1;
	this->last_time_ = current_time;
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Finalize the controller
  void GazeboJointsPID::Finalize() {
	alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboJointsPID::JoyCallback( const sensor_msgs::Joy::ConstPtr& joy ) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    
		if (joy->axes[bax_change_ctrl1_]==-1)
			this->p_b_ctrl1_++;
		if (joy->axes[bax_change_ctrl2_]==-1)
			this->p_b_ctrl2_++;			
	}

/****
 * 
 * 
 * 
 *                                                                                                                 ****/


  void GazeboJointsPID::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

		
  GZ_REGISTER_MODEL_PLUGIN(GazeboJointsPID)
}
