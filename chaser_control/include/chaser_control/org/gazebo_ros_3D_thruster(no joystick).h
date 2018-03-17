/*Modified header from gazebo_ros_planar_move for the purpose to implement thruster navigation
*
***********************************************************************************************/
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

#ifndef GAZEBO_ROS_3D_THRUSTER_HH
#define GAZEBO_ROS_3D_THRUSTER_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "pid_controller.h"

namespace gazebo {

  class GazeboRos3DThruster : public ModelPlugin {

    public: 
      GazeboRos3DThruster();
      ~GazeboRos3DThruster();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();
      virtual void Reset();

    private:
      //void publishOdometry(double step_time);

      physics::ModelPtr parent_;
      physics::LinkPtr base_link_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Subscriber vel_sub_;
      
      //boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      //std::string tf_prefix_;
      	
      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double command_vel_x_;
      double command_vel_y_;
      double command_vel_z_;
      double linear_velocity_limit_;
      double command_pos_yaw_;
      double command_pos_roll_;
      double command_pos_pitch_;
      double command_vel_yaw_;
      double command_vel_roll_;
      double command_vel_pitch_;
      bool alive_;
      double nominal_thrust_; //! [N]
      double nominal_reaction_torque_; //! [N]
      double dist_base_thruster_COM_; //! in [m]
      double joint_state_idle_sec_;
	
      common::Time last_time_;
      common::Time last_cmd_subscribe_time_;
      math::Vector3 relative_force_;
      math::Vector3 relative_torque_;
      math::Vector3 linear_cmd_;
      //math::Vector3 angular_cmd_; //ATTITUDE CONTROLLER ALWAYS ACTIVE!
      math::Vector3 available_thrust_;
      struct Controller{
	PIDController co_linear_vel_x_;
	PIDController co_linear_vel_y_;
	PIDController co_linear_vel_z_;
	PIDController co_angular_vel_x_; // roll
	PIDController co_angular_vel_y_; //pítch
	PIDController co_angular_vel_z_; //yaw  
	PIDController co_angular_pos_x_; // roll
	PIDController co_angular_pos_y_; //pítch
	PIDController co_angular_pos_z_; //yaw    
      } controllers_;
  };
  
  
  

}

#endif /* end of include guard: GAZEBO_ROS_3D_THRUSTER_HH */
