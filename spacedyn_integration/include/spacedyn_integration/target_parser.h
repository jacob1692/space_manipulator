#ifndef TARGET_PARSER_HH
#define TARGET_PARSER_HH
//====Gazebo Model Plugin Required Libraries ===//
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <stdio.h>
#include <map>
#include <gazebo/common/common.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//!SpaceDynLibraries //
//!        C++       //
#include <iostream>
#include <cstdio>
//using namespace std;
//!     SpDyn C++    //
#include "spacedyn_ros/spd/spd_m.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
//!       Others    //
#include "spacedyn_integration/dataext.h" // Global and define
//#include "spacedyn_integration/timer.h"
//#include "spacedyn_integration/datasave.h"


namespace gazebo {

  class GazeboRosSPDTargetParser : public ModelPlugin {

    public: 
      GazeboRosSPDTargetParser();
      virtual ~GazeboRosSPDTargetParser();
    protected: 
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      virtual void OnUpdate(const common::UpdateInfo &);
      void initSpd();

    private:
      void ModelLoad(physics::ModelPtr _modelmap, MODEL &m, sdf::ElementPtr _sdf);
      //! Map the angles and velocities from Gazebo to the MODEL in SpaceDyn ;
      //!Initialization of Spacedyn Variables and Structures
      void tMapping(MODEL& mt); 
      void ConvInertial(Matrix3& link_I_, const physics::LinkPtr& _link);
      physics::WorldPtr world_;
      physics::ModelPtr parent_;
      physics::LinkPtr base_link_;
   
      event::ConnectionPtr updateConnection;
      
      boost::mutex lock;
      boost::thread callback_queue_thread_;
      
      common::Time last_time_;
      common::Time sim_time_;
    
    //=== SpaceDyn ===//
	    double d_time;   
  };
}

#endif /* end of include guard: TARGET_PARSER_HH */
