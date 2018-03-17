/* copyright (C) 2017 <tohoku university/jacob hernandez> */
/* license BSD */

#ifndef GAZEBO_TARGET_M_HH
#define GAZEBO_TARGET_M_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>


#include <gazebo/math/gzmath.hh>

#include "spacedyn_integration/dataext.h"


namespace gazebo {

  class GazeboTargetM : public ModelPlugin {

    public: 
      GazeboTargetM();
      ~GazeboTargetM();
      void Load(physics::ModelPtr, sdf::ElementPtr);
      void OnUpdate(); 
     
    private:
      void Finalize();
      physics::ModelPtr parent_;

      
      physics::LinkPtr base_;
    
          
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_; //! nodehandle from gazebo
      ros::Subscriber joy_sub_tgt_;
      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
         
      
      //Custom Callback Subscriber Queue
      common::Time last_time_;
      bool alive_;
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();
      
      //joystick message Callback

      void JoyCallback (const sensor_msgs::Joy::ConstPtr& joy);
      int b_enablecin_; 
      
      //! both flags are redundant acknowledgment of change of control mode
      
      int p_b_enablecin_;
      
      gazebo::math::Vector3 linearv_, angularv_;      
  };
}

#endif /* end of include guard: GAZEBO_TARGET_M_HH */
