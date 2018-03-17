/* copyright (C) 2017 <tohoku university/jacob hernandez> */
/* license BSD */
//! Plugin for changing the linear and angular velocity of the target after the user indicates it with an interruption in the joystick.

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "target_description/gazebo_target_m.h"
#include "spacedyn_integration/dataext.h"

namespace gazebo 
{

  GazeboTargetM::GazeboTargetM() {
	  }
  GazeboTargetM::~GazeboTargetM() {}
	  
  // Load the controller
  void GazeboTargetM::Load(physics::ModelPtr _model, 
      sdf::ElementPtr sdf) 
  {
	ROS_INFO("LOADING PLUGIN FOR TARGET INPUT VELOCITY");
    this->parent_ = _model;
    this->base_=parent_->GetLink("base_footprint");
    
    this->linearv_=math::Vector3(0.0,0.0,0.0);//! linear velocity
    this->angularv_=math::Vector3(0.0,0.0,0.0);//! angular velocity
    
    this->alive_ = true;
    
    /* Parse parameters */
    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("GazeboTargetM missing <robotNamespace>, "
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
      ROS_WARN("GazeboTargetM (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }
   
    
		b_enablecin_ = 0;  //! XBOX: A 
		
		//!Initialization of persistent states for buttons
		
		p_b_enablecin_=0;
		
   
	//!Retrieve the joystick button configuration from the SDF file 
      
    if (sdf->HasElement("joyBEnableCin")){
      sdf-> GetElement("joyBEnableCin")-> GetValue()-> Get(b_enablecin_);      
	}
	
    // Ensure that ROS has been initialized and subscribe to joy
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("TargetM (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_INFO(" GazeboTargetM (%s) has been loaded!", 
        robot_namespace_.c_str());
	
	
    // subscribe to the joystick
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<sensor_msgs::Joy>(command_topic_, 1,
          boost::bind(&GazeboTargetM::JoyCallback, this, _1),
          ros::VoidPtr(), &queue_);
    joy_sub_tgt_ = rosnode_->subscribe(so);

     //start custom queue
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboTargetM::QueueThread, this));
	
    
    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboTargetM::OnUpdate, this));
    
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Update the controller
  void GazeboTargetM::OnUpdate() 
  {
	boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchronization Mechanism of the Different Threads
    //common::Time current_time = parent_->GetWorld()->GetSimTime();
	
	static int n_persist = 3;
	//static int times=0;
	static bool o_while=false;
	
	if(!flag_IN_CONTACT_) { //! Only works if the target is not in contact with the chaser
   
	   if ( (p_b_enablecin_>=n_persist)) { 
		        std::string inputString;
				std::stringstream input_stream;
				std::string x = ""; //! dummy
			while (!o_while){
				inputString.clear();
				input_stream.clear();
				x.clear();
				std::cout <<"Please introduce the linear and angular velocities: x y z - x y z "<<endl;
				std::cin.clear();
				//std::cin.ignore(INT_MAX,'\n');
				std::getline(std::cin, inputString);
				input_stream.str(inputString.c_str());
				input_stream>>linearv_.x>>linearv_.y>>linearv_.z>>x>>angularv_.x>>angularv_.y>>angularv_.z;
				if (x=="-") {
					o_while=true;}
			}
			
			if (o_while){
				this->base_->SetLinearVel(linearv_);
				this->base_->SetAngularVel(angularv_);
				p_b_enablecin_=0;
				o_while=false;
			}
			
		}
			
			
	}
 	//times=1;
		
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Finalize the controller
  void GazeboTargetM::Finalize() {
	alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboTargetM::JoyCallback( const sensor_msgs::Joy::ConstPtr& joy ) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
	if (joy->buttons[b_enablecin_] == 1)
			++this->p_b_enablecin_;
	}

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  void GazeboTargetM::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

		
  GZ_REGISTER_MODEL_PLUGIN(GazeboTargetM)
}
