//Modified version of the Gazebo Planar Movement to allow changes in attitude.

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

#include "chaser_control/gazebo_ros_3D_thruster.h"

namespace gazebo 
{

  GazeboRos3DThruster::GazeboRos3DThruster() {}

  GazeboRos3DThruster::~GazeboRos3DThruster() {}

  // Load the controller
  void GazeboRos3DThruster::Load(physics::ModelPtr parent, 
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;
    base_link_ = parent_->GetChildLink("base_footprint");

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("3DThrusterPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("3DThrusterPlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }
    
    /** PID controllers parameters **/
    
		controllers_.co_angular_vel_x_.Load(sdf, "rollpitchvel");
		controllers_.co_angular_vel_y_.Load(sdf, "rollpitchvel");
		controllers_.co_angular_vel_z_.Load(sdf, "yawvel");
		controllers_.co_angular_pos_x_.Load(sdf, "rollpitchpos");
		controllers_.co_angular_pos_y_.Load(sdf, "rollpitchpos");
		controllers_.co_angular_pos_z_.Load(sdf, "yawpos");
		controllers_.co_linear_vel_x_.Load(sdf, "velocityXY");
		controllers_.co_linear_vel_y_.Load(sdf, "velocityXY");
		controllers_.co_linear_vel_z_.Load(sdf, "velocityZ");
		
    /**                                                 **/
    
    
	command_vel_x_ = 0.0;
	command_vel_y_ = 0.0;
	command_vel_z_ = 0.0;
	linear_velocity_limit_=1; //![m/s]
	command_vel_yaw_= 0.0;
	command_vel_roll_= 0.0;
	command_vel_pitch_= 0.0;
	command_pos_yaw_= 0.0;
	command_pos_roll_= 0.0;
	command_pos_pitch_= 0.0;
    relative_force_ = math::Vector3(0.0,0.0,0.0);
    relative_torque_ = math::Vector3(0.0,0.0,0.0);
	linear_cmd_=math::Vector3(0.0,0.0,0.0);
   // angular_cmd_=math::Vector3(0.0,0.0,0.0);
    
    alive_ = true;
    
    nominal_thrust_= 100; //! [N/s]
    dist_base_thruster_COM_=0.150; //![m]
	nominal_reaction_torque_=50; //! angular_impulse_ [N.m.s]

	if (sdf->HasElement("velocityXYZLimit"))
      (sdf->GetElement("velocityXYZLimit")->GetValue()->Get(linear_velocity_limit_));  
    if (sdf->HasElement("nominalThrust"))
      (sdf->GetElement("nominalThrust")->GetValue()->Get(nominal_thrust_));  
    if (sdf->HasElement("distThrusterBaseCOM"))
      (sdf->GetElement("distThrusterBaseCOM")->GetValue()->Get(dist_base_thruster_COM_));   
    if (sdf->HasElement("nominalReactionTorque"))
      (sdf->GetElement("nominalReactionTorque")->GetValue()->Get(nominal_reaction_torque_));    

	ROS_INFO("3DThrusterPlugin (ns = %s) nominalThrust = %f", robot_namespace_.c_str(), nominal_thrust_);
	ROS_INFO("3DThrusterPlugin (ns = %s) nominalReactionTorque = %f", robot_namespace_.c_str(), nominal_reaction_torque_);
    ROS_INFO("3DThrusterPlugin (ns = %s) distThrusterBaseCOM_ = %f", robot_namespace_.c_str(), dist_base_thruster_COM_);
	

    // joint_state_idle_sec
    joint_state_idle_sec_ = -1;
    if (sdf->HasElement("joint_state_idle_sec"))
      (sdf->GetElement("joint_state_idle_sec")->GetValue()->Get(joint_state_idle_sec_));
    last_cmd_subscribe_time_ = parent_->GetWorld()->GetSimTime();
	

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("3DThrusterPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

   // tf_prefix_ = tf::getPrefixParam(*rosnode_);
   // transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the cmd vel topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRos3DThruster::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);
    vel_sub_ = rosnode_->subscribe(so);
    

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRos3DThruster::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRos3DThruster::UpdateChild, this));

    this->last_time_ = parent_->GetWorld()->GetSimTime();
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Update the controller
  void GazeboRos3DThruster::UpdateChild() 
  {
    boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchornization Mechanism of the Different Threads
    common::Time current_time = parent_->GetWorld()->GetSimTime();
    math::Pose pose = parent_->GetWorldPose();
    math::Vector3 principal_moments;
    math::Vector3 products_inertia;
    math::Vector3 max_angular_acc;
    math::Vector3 max_angular_vel;
    static int stopx = 0;
    static int stopy = 0;
    static int stopz = 0;
    
    float dt  = (current_time-this->last_time_).Double();

	    //Calculation of the force and torque in the thrusters
	static double force_thrusters = nominal_thrust_*2; //Dual Burn 
    static double torque_thrustersandwheels = force_thrusters*dist_base_thruster_COM_ + nominal_reaction_torque_; //Pair Moment
	
	/**
	static double mass= base_link_->GetInertial()->GetMass();
	principal_moments= base_link_->GetInertial()->GetPrincipalMoments(); //Ixx Iyy Izz
	products_inertia= base_link_->GetInertial()->GetProductsofInertia(); // Ixy Ixz Iyz
	static double max_linear_acc=force_thrusters/mass;
	max_angular_acc= math::Vector3(torque_thrustersandwheels/(principal_moments.x + products_inertia.x + products_inertia.y),
									torque_thrustersandwheels/(principal_moments.y + products_inertia.x + products_inertia.z),
									torque_thrustersandwheels/(principal_moments.z + products_inertia.y + products_inertia.z));   
    static double max_linear_vel=max_linear_acc*dt;
    // ROS_WARN("max_linear_vel = %f", max_linear_vel);
    max_angular_vel = math::Vector3(max_angular_acc.x*dt,
														max_angular_acc.y*dt,
														max_angular_acc.z*dt);
    **/
    // set controllers limits
    /**
		controllers_.co_angular_vel_x_.limit=max_angular_vel.x;
		controllers_.co_angular_vel_y_.limit=max_angular_vel.y;
		controllers_.co_angular_vel_z_.limit=max_angular_vel.z;
		controllers_.co_angular_pos_x_.limit=max_angular_vel.x*dt;
		controllers_.co_angular_pos_y_.limit=max_angular_vel.y*dt;
		controllers_.co_angular_pos_z_.limit=max_angular_vel.z*dt;
		controllers_.co_linear_vel_x_.limit=max_linear_vel*2; //x2x2 thrusters
		controllers_.co_linear_vel_y_.limit=max_linear_vel;
		controllers_.co_linear_vel_z_.limit=max_linear_vel;
    **/
    
    
	//Get Velocities
	
	
    math::Vector3 linear_vel = base_link_->GetRelativeLinearVel();
	math::Vector3 linear_acc = base_link_->GetRelativeLinearAccel();
	math::Vector3 world_angular_acc = base_link_->GetWorldAngularAccel();
	math::Vector3 world_angular_pos = base_link_->GetWorldPose().rot.GetAsEuler();
	math::Vector3 world_angular_vel = base_link_->GetWorldAngularVel();

    // check for joint_state_idle_sec_
    if ( joint_state_idle_sec_ > 0 &&
         (current_time-this->last_cmd_subscribe_time_).Double() > joint_state_idle_sec_ ) {
      if (int((current_time-this->last_cmd_subscribe_time_).Double()*1000)%5000 < 1 ) {
        ROS_WARN("3DThrusterPlugin (ns = %s) did not received %s for %f sec",
                 robot_namespace_.c_str(), command_topic_.c_str(), joint_state_idle_sec_);
      }
      
      command_vel_x_ = command_vel_y_= command_vel_z_=0;
      command_vel_yaw_=command_vel_pitch_=command_vel_roll_=0;
      command_pos_yaw_=command_pos_pitch_=command_pos_roll_=0;
    }
    
    //! identify stop gesture in the GUI
	if (command_vel_x_==linear_velocity_limit_){
		if (stopx==0) stopx =1;}
	if (command_vel_x_==-linear_velocity_limit_){
		if (stopx==1) stopx =2;}
	if (command_vel_y_==linear_velocity_limit_){
		if (stopy==0) stopy =1;}
	if (command_vel_y_==-linear_velocity_limit_){
		if (stopy==1) stopy =2;}
	if (command_vel_z_==linear_velocity_limit_){
		if (stopz==0) stopz =1;}
	if (command_vel_y_==-linear_velocity_limit_){
		if (stopz==1) stopz =2;}
	
	
	linear_cmd_ = math::Vector3 (command_vel_x_, command_vel_y_, command_vel_z_);
	
	//! the stop gesture := go to the upper limit and then to the lower limit of the control... 
	
	if (stopx==2) linear_cmd_.x=0.0;
	if (stopy==2) linear_cmd_.y=0.0;
	if (stopz==2) linear_cmd_.z=0.0;
	
	if (linear_cmd_.x==0.0 && fabs(linear_vel.x)<=1e-4) stopx=0.0; //! restore the controllability of the speed after the vehicle stops
	if (linear_cmd_.y==0.0 && fabs(linear_vel.y)<=1e-4) stopy=0.0;
	if (linear_cmd_.z==0.0 && fabs(linear_vel.z)<=1e-4) stopz=0.0;
	
    //angular_cmd_ = math::Vector3(command_vel_roll_, command_vel_pitch_,command_vel_yaw_);
	
	if ((command_vel_x_ > 0 || command_vel_x_ < 0)||stopx==2) relative_force_.x = controllers_.co_linear_vel_x_.update(linear_cmd_.x,linear_vel.x,linear_acc.x,dt);
	else relative_force_.x=0;
	if ((command_vel_y_ > 0 || command_vel_y_ < 0)||stopy==2)	relative_force_.y = controllers_.co_linear_vel_y_.update(linear_cmd_.y,linear_vel.y,linear_acc.y,dt);
	else relative_force_.y=0;
	if ((command_vel_z_ > 0 || command_vel_z_ < 0)||stopz==2)	relative_force_.z = controllers_.co_linear_vel_z_.update(linear_cmd_.z,linear_vel.z,linear_acc.z,dt);
    else relative_force_.z=0;
    
    /** ALWAYS CONTROL THE ATTITUDE**/
    
    if (true)	{
		command_vel_roll_= controllers_.co_angular_pos_x_.update(command_pos_roll_,world_angular_pos.x,world_angular_vel.x,dt);
		relative_torque_.x = controllers_.co_angular_vel_x_.update(command_vel_roll_,world_angular_vel.x,world_angular_acc.x,dt);
	}
    
    // else relative_torque_.x=0;
	if (true)	{
		command_vel_pitch_= controllers_.co_angular_pos_y_.update(command_pos_pitch_,world_angular_pos.y,world_angular_vel.y,dt);
		relative_torque_.y = controllers_.co_angular_vel_y_.update(command_vel_pitch_,world_angular_vel.y,world_angular_acc.y,dt);
	}
	// else relative_torque_.y=0;
	if (true)	{
		command_vel_yaw_= controllers_.co_angular_pos_z_.update(command_pos_yaw_,world_angular_pos.z,world_angular_vel.z,dt);
		relative_torque_.z = controllers_.co_angular_vel_z_.update(command_vel_yaw_,world_angular_vel.z,world_angular_acc.z,dt);
	}
	// else relative_torque_.z=0;
	
	/** **/
	
    
    /** TRUNCATE THE OUTPUT OF THE CONTROLLER TO THE FORCE / TORQUE LIMITS **/
	if (fabs(relative_force_.x) > force_thrusters) relative_force_.x = (relative_force_.x<0 ? -1.0 : 1.0) * force_thrusters;
	if (fabs(relative_force_.y) > force_thrusters) relative_force_.y = (relative_force_.y<0 ? -1.0 : 1.0) * force_thrusters;
	if (fabs(relative_force_.z) > force_thrusters) relative_force_.z = (relative_force_.z<0 ? -1.0 : 1.0) * force_thrusters;
	if (fabs(relative_torque_.x) > torque_thrustersandwheels) relative_torque_.x = (relative_torque_.x<0 ? -1.0 : 1.0) * torque_thrustersandwheels;
	if (fabs(relative_torque_.y) > torque_thrustersandwheels) relative_torque_.y = (relative_torque_.y<0 ? -1.0 : 1.0) * torque_thrustersandwheels;
	if (fabs(relative_torque_.z) > torque_thrustersandwheels) relative_torque_.z = (relative_torque_.z<0 ? -1.0 : 1.0) * torque_thrustersandwheels;
	
    
	//!Aplication of the relative forces and torques


    base_link_->AddRelativeForce(relative_force_);
    base_link_->AddRelativeTorque(relative_torque_);
    
    /**DEBUGGING MESSAGES **/
    //ROS_WARN("relative_accx= %f",base_link_->GetRelativeLinearAccel().x );
	//ROS_WARN("relative_torquey= %f",relative_torque_.y );
	//ROS_WARN("relative_torquez= %f",relative_torque_.z );
	
	/**   													**/
    this->last_time_ = parent_->GetWorld()->GetSimTime();
  }

	

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Finalize the controller
  void GazeboRos3DThruster::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRos3DThruster::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    command_vel_x_ = cmd_msg->linear.x;
    command_vel_y_ = cmd_msg->linear.y;
    command_pos_yaw_ = cmd_msg->angular.z;
    command_vel_z_ = cmd_msg->linear.z;
    command_pos_roll_ = cmd_msg->angular.x;
    command_pos_pitch_ = cmd_msg->linear.y;
    last_cmd_subscribe_time_ = parent_->GetWorld()->GetSimTime();
  }

  void GazeboRos3DThruster::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

	//Reset the controller
	void GazeboRos3DThruster::Reset()
		{
		controllers_.co_linear_vel_x_.reset();
		controllers_.co_linear_vel_y_.reset();
		controllers_.co_linear_vel_z_.reset();
		controllers_.co_angular_vel_x_.reset();
		controllers_.co_angular_vel_y_.reset();
		controllers_.co_angular_vel_z_.reset();
		command_vel_x_ = 0.0;
		command_vel_y_ = 0.0;
		command_vel_z_ = 0.0;
		command_vel_yaw_= 0.0;
		command_vel_roll_= 0.0;
		command_vel_pitch_= 0.0;
		command_pos_yaw_= 0.0;
		command_pos_roll_= 0.0;
		command_pos_pitch_= 0.0;
		relative_force_ = math::Vector3(0.0,0.0,0.0);
		relative_torque_ = math::Vector3(0.0,0.0,0.0); 
		linear_cmd_ = math::Vector3(0.0,0.0,0.0); 
		//angular_cmd_ = math::Vector3(0.0,0.0,0.0);  
		}


  GZ_REGISTER_MODEL_PLUGIN(GazeboRos3DThruster)
}

