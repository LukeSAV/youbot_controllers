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

class SimpleBasePositionController
{
public:
  
  /// to store position and velocity commands
   struct Commands
  {
  double position_; // Last commanded position
  double velocity_; // Last commanded velocity
  bool has_velocity_; // false if no velocity command has been specified
  };
  
  /** initializes the class
   */
  SimpleBasePositionController(ros::NodeHandle &nh);
  
  /** Give set position of the joint for next update
   */
  void setCommand(double pos_target);
  
  /** Issues commands to the joint. Should be called at regular intervals
  */
  void update(const ros::Time& time, const ros::Duration& period);

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance);
  
  /// Turns robot a specified amount of radians
  bool turnOdom(bool clockwise, double radians);
private:
  ros::NodeHandle nh_; /// ROS node handle 
  ros::Publisher velocity_commander_; /// velocity command publisher
  ros::Publisher state_publisher_; /// publishes the "state" of the base relative to the odometry frame
  tf::TransformListener listener_; /// TF transform listener
  
  control_toolbox::Pid pid_controller_; /// internal PID controller
  
  
};