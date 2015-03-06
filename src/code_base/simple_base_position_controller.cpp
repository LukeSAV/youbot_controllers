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

#include "youbot_controllers/simple_base_position_controller.h"

//! ROS node initialization
SimpleBasePositionController::SimpleBasePositionController(ros::NodeHandle &nh)
{
  nh_ = nh;
  
  if( !nh_.getParam("/youbot_base/velocity_topic"), velocity_topic_ )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/velocity_topic parameters was found on parameter server.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/position_frame"), position_frame_ )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/position_frame parameters was found on parameter server.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/base_frame"), base_frame_ )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/base_frame parameters was found on parameter server.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/base_name"), base_name_ )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/base_name parameters was found on parameter server.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/publish_base_state"), publish_state_ )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/publish_base_state parameters was found on parameter server.");
    ros::shutdown();
    return;
  }
  if( !pid_controller_.init( ros::NodeHandle(nh_, "/youbot_base/pid") ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/pid parameters were found on parameter server.");
    ros::shutdown();
    return;
  }
  
  //set up the publisher for the velocity command topic
  velocity_commander_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic_, 1000);
  
  // set up publisher for /joint_states topic
  if( publish_state_ )
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

void SimpleBasePositionController::setCommand(double x, double y, double theta)
{
  
}

//! Drive forward a specified distance based on odometry information
bool SimpleBasePositionController::driveForwardOdom(double distance)
{
  //wait for the listener to get the first message
  listener_.waitForTransform("base_footprint", "odom", 
			      ros::Time(0), ros::Duration(1.0));
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_footprint", "odom", 
			    ros::Time(0), start_transform);
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to go forward at 0.25 m/s
  base_cmd.linear.y = base_cmd.angular.z = 0;
  base_cmd.linear.x = 0.25;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_footprint", "odom", 
				ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    //see how far we've traveled
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    double dist_moved = relative_transform.getOrigin().length();

    if(dist_moved > distance)
    {
      base_cmd.linear.x = 0;
      cmd_vel_pub_.publish(base_cmd);
      done = true;
    }
  }
  if (done) return true;
  return false;
}

bool SimpleBasePositionController::turnOdom(bool clockwise, double radians)
{
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;

  //wait for the listener to get the first message
  listener_.waitForTransform("base_footprint", "odom", 
			      ros::Time(0), ros::Duration(1.0));
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_footprint", "odom", 
			    ros::Time(0), start_transform);
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.75;
  if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
  
  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);
  if (!clockwise) desired_turn_axis = -desired_turn_axis;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_footprint", "odom", 
				ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis = 
      relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) < 1.0e-2) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians) done = true;
  }
  if (done) return true;
  return false;
}
