/* Copyright (c) 2014,2015 , Stefan Isler, islerstefan@bluewin.ch
*
This file is part of youbot_controllers, a ROS package providing some additional controllers for the KUKA youbot,

youbot_controllers is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
youbot_controllers is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with youbot_controllers. If not, see <http://www.gnu.org/licenses/>.
*/

/// base position control adapter using tf position on top of the base velocity controller, based on the JointPositionController in the ros_control/velocity_controllers package by Dave Coleman

#pragma once

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread/mutex.hpp>

class SimpleBasePositionController
{
public:
  
  struct PlanarPosition
  {
    double x;
    double y;
    double theta;
  };
  
  struct PlanarVelocity
  {
    double dx;
    double dy;
    double dtheta;
  };
  
  /// to store position and velocity commands
   struct Commands
  {
    PlanarPosition position;
    PlanarVelocity velocity;
    bool has_velocity; // false if no velocity command has been specified
  };
  
  /** class constructor, also initializes the class, loads parameters from parameter server
   */
  SimpleBasePositionController(ros::NodeHandle &_nh);
  
  /** destructor
   */
  ~SimpleBasePositionController();
  
  /** starts loop that keeps updating */
  void start();
  
  /** Give set position of the joint for next update
   */
  void setCommand(double _x, double _y, double _theta);
  
  /** Give set position of the joint for next update
   */
  void setCommand( PlanarPosition _commanded_pos );
  
  /** returns current transform from robot base_frame to world position_frame (how entities would be transformed)
   */
  tf::StampedTransform getCurrentTransform();
  
  /** returns the robot state associated with the passed transform from base_frame to world position_frame  (how entities would be transformed)
   */
  PlanarPosition getBaseState( tf::StampedTransform& _transform );
  
  /** enforces that velocities are in a feasible range */
  void enforceVelocityLimits( PlanarVelocity& _velocity );
  
  /** enforces limits on _value -> helper function for enforceVelocityLimits(...) */
  void singleVelLimitEnforcer( double& _value, std::pair<double,double>& _bounds, std::pair<double,double>& _feasibility_limits, double _feasibility_threshold );
  
  /** enforces space limits if they have been set */
  void enforceSpaceLimits( PlanarPosition& _position );
  
  /** initializes the position command to the current position and resets the pid controller */
  void init();
  
  /** Issues commands to the joint and should thus be called at regular intervals
   * @param _time time of the update
   * @param _period time since last update
   */
  void update( const ros::Time& _time, const ros::Duration& _period );
  
  /** issues a command message for the base and publishes state information if activated
   * @param _base_velocity new velocity for the base
   */
  void publishCommand( geometry_msgs::Twist _base_velocity );
  
  /** sends zero velocities to the youbot, telling the base to stop its movement
   */
  void halt();

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance);
  
  /// Turns robot a specified amount of radians
  bool turnOdom(bool clockwise, double radians);
private:
  ros::NodeHandle nh_; /// ROS node handle 
  ros::Publisher velocity_commander_; /// velocity command publisher
  ros::Publisher state_publisher_; /// publishes the "state" of the base relative to the odometry frame
  tf::TransformListener listener_; /// TF transform listener
  
  std::string velocity_topic_; /// where the velocity commands are published to
  std::string base_frame_; /// tf frame that represents the youbot position
  std::string position_frame_; /// tf frame that represents the world (e.g. the odometry frame)
  
  bool publish_state_; /// whether the position state of the controller is published on /joint_states or not
  ros::Time last_published_; /// when states were published last
  ros::Duration publish_period_; /// publish period
  std::string base_name_; /// name of the "virtual base link"
  std::string base_x_name_; /// name of the x "joint"
  std::string base_y_name_; /// name of the y "joint"
  std::string base_theta_name_; /// name of the theta "joint"
  
  bool use_velocity_limits_;
  double unfeasible_rounding_threshold_linear_;
  double unfeasible_rounding_threshold_angular_;
  std::pair<double,double> dx_limits_;
  std::pair<double,double> dy_limits_;
  std::pair<double,double> dtheta_limits_;
  std::pair<double,double> dx_unfeasible_;
  std::pair<double,double> dy_unfeasible_;
  std::pair<double,double> dtheta_unfeasible_;
  
  
  bool use_space_limits_;
  std::pair<double,double> x_limits_;
  std::pair<double,double> y_limits_;
  
  control_toolbox::Pid pid_x_; /// internal PID controller
  control_toolbox::Pid pid_y_; /// internal PID controller
  control_toolbox::Pid pid_theta_; /// internal PID controller
  
  boost::mutex command_protector_; /// to ensure that no one writes onto the command when it is read
  Commands command_;
};