#ifndef CHASER_CONTROLLER_HH
#define CHASER_CONTROLLER_HH
//====Gazebo Model Plugin Required Libraries ===//
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <stdio.h>
#include <map>
#include <gazebo/common/common.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>

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

  class GazeboRosSPDChaserController : public ModelPlugin {

    public: 
      GazeboRosSPDChaserController ();
      virtual ~GazeboRosSPDChaserController ();
    
   //======================================//
   //void stopThreadSpd();
   //Timer timer;
   //double *inputVal;

    double extime;
    protected: 
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      virtual void OnUpdate(const common::UpdateInfo &);
      void initSpd();
     //virtual void FiniChild();
     //virtual void Reset();

    private:
      void ModelLoad(physics::ModelPtr _modelmap, MODEL& m, sdf::ElementPtr _sdf);
      void ImpParamLoad(sdf::ElementPtr _sdf);
      void ImpParamCalc(double _Idn, double _dampD, double _v0, double _y0, double _tmi, double _kw, double _dw, double& _Wdn, double& _tddi, double& _tkdi); 
      void ImpulseController(MODEL& m_, MODEL& mt_, physics::ModelPtr _model);
      //! Map the angles and velocities from Gazebo to the MODEL in SpaceDyn ;
      //!Initialization of Spacedyn Variables and Structures
      void qMapping(MODEL& m, const double& deltat ); 
      void vConvert(const math::Vector3& vgazebo, Vector3& vspd);
      void qConvert(const math::Quaternion& qgazebo, Vector4& qspd);
      void ConvEuler(const math::Quaternion& vqgazebo, Vector3& vqspd);
      void ConvInertial(Matrix3& link_I_, const physics::LinkPtr& _link);
      math::Pose ee_world_pose_;
      sensors::ContactSensorPtr Contact_Sensor_;
      physics::WorldPtr world_;
      physics::ModelPtr parent_;
      physics::LinkPtr base_link_;
      physics::LinkPtr arm0_;
      physics::LinkPtr arm1_;
      physics::LinkPtr arm1_1_;
      physics::LinkPtr arm2_;
      physics::LinkPtr arm2_1_;
      physics::LinkPtr arm3_;
      physics::LinkPtr hand_; 
      physics::JointControllerPtr contr_j1_;
      physics::JointControllerPtr contr_j2_;
      physics::JointControllerPtr contr_j3_;
      physics::JointControllerPtr contr_j4_;
      physics::JointControllerPtr contr_j5_;
      physics::JointControllerPtr contr_j6_;
      //! physics::JointControllerPtr contr_d1_; Passive Compliant Wrist
      physics::JointPtr j1_;
      physics::JointPtr j2_;
      physics::JointPtr j3_;
      physics::JointPtr j4_;
      physics::JointPtr j5_;
      physics::JointPtr j6_;
      physics::JointPtr d1_;
      event::ConnectionPtr updateConnection;
      
      boost::mutex lock;
      boost::thread callback_queue_thread_;
      
      //bool alive_;
      common::Time last_time_;
      common::Time sim_time_;
	//common::Time last_cmd_subscribe_time_;  to any topic    
    
    //! SpaceDyn ===//
    	//MODEL m;
   	 //- Timer
	   // double time;  NOT REQUIRED IN THIS SCRIPT
	   // double d_time;
	   //double f_time; NOT REQUIRED IN THIS SCRIPT
    //- Counters
    //	    int data_counter;    
   
    
    //!*********************Others************************//
    
	//boost::shared_ptr<ros::NodeHandle> rosnode_;
	//void QueueThread();
	//std::string robot_namespace_;
	//std::string command_topic_;
	// Custom Callback Queue
	//ros::CallbackQueue queue_;
	//ros::Subscriber vel_sub_; 
	//boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
	//std::string tf_prefix_;
	//command velocity callback
	//void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  };
}

#endif /* end of include guard: CHASER_CONTROLLER_HH */
