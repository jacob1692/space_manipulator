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
  GazeboJointsPID::~GazeboJointsPID() {
	  this->Finalize();
	  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
	  ROS_INFO("JointsPID Disable");
	  }
  
  
	  
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
    
    scale_=0.01; //! Scale for the K's
    alive_ = true;
    
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
    
    errorlimit_=10;
    
    if (sdf->HasElement("scale")){
			sdf->GetElement("scale")->GetValue()->Get(scale_);
	}
    
    if (sdf->HasElement("errorLimit")){
			sdf->GetElement("errorLimit")->GetValue()->Get(errorlimit_);
	}
    
    
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
		
		//!Initialization of persistent states for buttons
		
		p_b_ctrl1_=0; 
		
		//! Initialization of flags;
		flag_CH_CTRL_=false;
    
   
	//!Retrieve the joystick axes configuration from the SDF file 
      
    if (sdf->HasElement("joyBAxisChangeCtrl1")){
      sdf-> GetElement("joyBAxisChangeCtrl1")-> GetValue()-> Get(bax_change_ctrl1_);      
	}
	//! Retrieve the scale factors for the angular and linear spans
	
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
	
	
    // subscribe to the joystick
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<sensor_msgs::Joy>(command_topic_, 1,
          boost::bind(&GazeboJointsPID::JoyCallback, this, _1),
          ros::VoidPtr(), &queue_);
    joy_sub_ = rosnode_->subscribe(so);

     //start custom queue
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboJointsPID::QueueThread, this));
	
	this->last_time_ = parent_->GetWorld()->GetSimTime();
    
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
	boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchronization Mechanism of the Different Threads
    common::Time current_time = parent_->GetWorld()->GetSimTime();
	
	static int n_persist = 3;
	static int times=0;
	static double intsum1=0.0; static double intsum2=0.0; static double intsum3=0.0; static double intsum4=0.0; static double intsum5=0.0; static double intsum6=0.0;
	
	static double e1=0.0; static double e2=0.0; static double e3=0.0; static double e4=0.0; static double e5=0.0; static double e6=0.0;
	
   
   if ( (p_b_ctrl1_>=n_persist)) { 
	    this->flag_CH_CTRL_=!this->flag_CH_CTRL_;
		if (this->flag_CH_CTRL_==false){
			ROS_WARN("THE JOINTS ARE IN INDIVIDUAL STIFF CONTROL");
		}
		if (this->flag_CH_CTRL_==true){
			ROS_WARN("THE ROBOT IS IN HYBRID IMPEDANCE - POSITION CONTROL"); 
		}
		p_b_ctrl1_=0;
	}
	//static int times2=0;
	//if (flag_IN_CONTACT_){
		//this->flag_CH_CTRL_=true;
		////if (times2==0) {
			//////ROS_WARN("CONTACT: THE ROBOT IS IN HYBRID IMPEDANCE - POSITION CONTROL");
			////times2=-1;
		//}

	
	common::Time _dt = current_time - this->last_time_ ;
	if (this->flag_CH_CTRL_==false)
		{
			if (times==0){
				ROS_INFO("INDIVIDUAL JOINTS POSITION CONTROL");
				//ROS_INFO( "JC1KP: %f " , PIDj1_.x);
			}
		
		e1=- this->j1_->GetAngle(2).Radian();
		e2=- this->j2_->GetAngle(3).Radian();
		e3=- this->j3_->GetAngle(4).Radian();
		e4=- this->j4_->GetAngle(5).Radian();
		e5=- this->j5_->GetAngle(6).Radian();
		e6=- this->j6_->GetAngle(7).Radian();
		
		if (errorlimit_>0 && fabs(e1) > errorlimit_) e1 = (e1<0 ? -1 : 1) * errorlimit_;
		if (errorlimit_>0 && fabs(e2) > errorlimit_) e2 = (e2<0 ? -1 : 1) * errorlimit_;
		if (errorlimit_>0 && fabs(e3) > errorlimit_) e3 = (e3<0 ? -1 : 1) * errorlimit_;
		if (errorlimit_>0 && fabs(e4) > errorlimit_) e4 = (e4<0 ? -1 : 1) * errorlimit_;
		if (errorlimit_>0 && fabs(e5) > errorlimit_) e5 = (e5<0 ? -1 : 1) * errorlimit_;
		if (errorlimit_>0 && fabs(e6) > errorlimit_) e6 = (e6<0 ? -1 : 1) * errorlimit_;
		
		if (fabs(intsum1) > PIDj1_.w){
			intsum1=0;
		}
		if (fabs(intsum2) > PIDj2_.w){
			intsum2=0;
		}
		if (fabs(intsum3) > PIDj3_.w){
			intsum3=0;
		}
		if (fabs(intsum4) > PIDj4_.w){
			intsum4=0;
		}
		if (fabs(intsum5) > PIDj5_.w){
			intsum5=0;
		}
		if (fabs(intsum6) > PIDj6_.w){
			intsum6=0;
		}
		
		
		double t1= e1*PIDj1_.x + intsum1*_dt.Double()*PIDj1_.y - this->j1_->GetVelocity(2)*PIDj1_.z;
		double t2= e2*PIDj2_.x + intsum2*_dt.Double()*PIDj2_.y - this->j2_->GetVelocity(3)*PIDj2_.z;
		double t3= e3*PIDj3_.x + intsum3*_dt.Double()*PIDj3_.y - this->j3_->GetVelocity(4)*PIDj3_.z;
		double t4= e4*PIDj4_.x + intsum4*_dt.Double()*PIDj4_.y - this->j4_->GetVelocity(5)*PIDj4_.z;
		double t5= e5*PIDj5_.x + intsum5*_dt.Double()*PIDj5_.y - this->j5_->GetVelocity(6)*PIDj5_.z;
		double t6= e6*PIDj6_.x + intsum6*_dt.Double()*PIDj6_.y - this->j6_->GetVelocity(7)*PIDj6_.z;

		
		this->j1_->SetForce(2,t1*scale_); 
		this->j2_->SetForce(3,t2*scale_); 
		this->j3_->SetForce(4,t3*scale_); 
		this->j4_->SetForce(5,t4*scale_); 
		this->j5_->SetForce(6,t5*scale_); 
		this->j6_->SetForce(7,t6*scale_); 
	
	
		intsum1+=e1;
		intsum2+=e2;
		intsum3+=e3;
		intsum4+=e4;
		intsum5+=e5;
		intsum6+=e6;
		
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
	if (joy->axes[bax_change_ctrl1_] == -1)
			++this->p_b_ctrl1_;			
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
