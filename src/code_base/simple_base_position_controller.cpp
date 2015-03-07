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
#include <angles/angles.h>

//! ROS node initialization
SimpleBasePositionController::SimpleBasePositionController(ros::NodeHandle &_nh)
{
  nh_ = _nh;
  
  if( !nh_.getParam("/youbot_base/velocity_topic", velocity_topic_ ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/velocity_topic parameters was found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/position_frame", position_frame_ ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/position_frame parameters was found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/base_frame", base_frame_ ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/base_frame parameters was found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/base_name", base_name_ ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/base_name parameters was found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/publish_base_state", publish_state_ ) )
  {
    ROS_INFO("SimpleBasePositionController could'nt find a youbot_base/publish_base_state parameter on the parameter server. State will be published.");
    publish_state_ = true;
  }
  if( publish_state_ )
  {
    double publish_rate;
    if( !nh_.getParam("/youbot_base/publish_rate", publish_rate ) )
    {
      ROS_INFO("SimpleBasePositionController:: Publishing enabled but no publishing period defined. Using 30 Hz.");
      publish_period_ = ros::Duration(1/30);
    }
    else
    {
      if( publish_rate==0 )
      {
	ROS_FATAL("SimpleBasePositionController:: Publishing enabled but publishing period set to 0 Hz which is invalid. Shutting down the node.");
	ros::shutdown();
	return;
      }
      publish_period_ = ros::Duration(1/publish_rate);
    }
  }
  double p_pos, i_pos, d_pos;
  if( !nh_.getParam("/youbot_base/pid_pos/p", p_pos ) ||
      !nh_.getParam("/youbot_base/pid_pos/i", i_pos ) ||
      !nh_.getParam("/youbot_base/pid_pos/d", d_pos )
  )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/pid_pos parameters were found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  else
  {
    nh_.setParam("/youbot_base/pid_pos/x/p",p_pos);
    nh_.setParam("/youbot_base/pid_pos/x/i",i_pos);
    nh_.setParam("/youbot_base/pid_pos/x/d",d_pos);
    nh_.setParam("/youbot_base/pid_pos/y/p",p_pos);
    nh_.setParam("/youbot_base/pid_pos/y/i",i_pos);
    nh_.setParam("/youbot_base/pid_pos/y/d",d_pos);
    pid_x_.init( ros::NodeHandle(nh_, "/youbot_base/pid_pos/x") );
    pid_y_.init( ros::NodeHandle(nh_, "/youbot_base/pid_pos/y") );
  }
  if( !pid_theta_.init( ros::NodeHandle(nh_, "/youbot_base/pid_theta") ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't be initialized because no youbot_base/pid_theta parameters were found on parameter server. Shutting down the node.");
    ros::shutdown();
    return;
  }
  if( !nh_.getParam("/youbot_base/use_velocity_limits", use_velocity_limits_ ) )
  {
    ROS_FATAL("SimpleBasePositionController couldn't find a youbot_base/use_velocity_limits parameter on the parameter server. This is necessary. Shutting down node.");
    ros::shutdown();
    return;
  }
  else if( use_velocity_limits_ )
  {
    double dx_min, dx_max, dy_min, dy_max, dtheta_min, dtheta_max, dx_feas_min, dx_feas_max, dy_feas_min, dy_feas_max, dtheta_feas_min, dtheta_feas_max;
    if( !nh_.getParam("/youbot_base/velocity_limits/upper/x/min", dx_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/x/max", dx_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/y/min", dy_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/y/max", dy_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/theta/min", dtheta_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/theta/max", dtheta_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/x/neg", dx_feas_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/x/pos", dx_feas_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/y/neg", dy_feas_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/y/pos", dy_feas_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/theta/neg", dtheta_feas_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/theta/pos", dtheta_feas_max ) ||
        !nh_.getParam("/youbot_base/unfeasible_rounding_threshold/linear", unfeasible_rounding_threshold_linear_ ) || 
        !nh_.getParam("/youbot_base/unfeasible_rounding_threshold/angular", unfeasible_rounding_threshold_angular_ ) )
    {
      ROS_FATAL("SimpleBasePositionController:: youbot_base/use_velocity_limits is set to true but not all necessary configurations have been specified. Shutting down node.");
      ros::shutdown();
      return;
    }
    else
    {
      dx_limits_ = std::pair<double,double>(dx_min,dx_max);
      dy_limits_ = std::pair<double,double>(dy_min,dy_max);
      dtheta_limits_ = std::pair<double,double>(dtheta_min,dtheta_max);
      dx_unfeasible_ = std::pair<double,double>(dx_feas_min,dx_feas_max);
      dy_unfeasible_ = std::pair<double,double>(dy_feas_min,dy_feas_max);
      dtheta_unfeasible_ = std::pair<double,double>(dtheta_feas_min,dtheta_feas_max);
    }
  }
  if( !nh_.getParam("/youbot_base/use_space_limits", use_space_limits_ ) )
  {
    ROS_INFO("SimpleBasePositionController couldn't find a youbot_base/use_space_limits parameter on the parameter server. Assuming that no space limits shall be used");
    use_space_limits_ = false;
  }
  else if(use_space_limits_)
  {
    double x_min,x_max,y_min,y_max;
    if( !nh_.getParam("/youbot_base/space_limits/x/min", x_min ) || 
        !nh_.getParam("/youbot_base/space_limits/x/max", x_max ) || 
        !nh_.getParam("/youbot_base/space_limits/y/min", y_min ) || 
        !nh_.getParam("/youbot_base/space_limits/y/max", y_max ) )
    {
      ROS_ERROR("Youbot::SimpleBasePositionController:: /youbot_base/space_limits specified that space limits should be used but not all parameters necessary for setup were found on the parameter server. Space limiting is disabled.");
      use_space_limits_ = false;
    }
    else
    {
      if( x_min>x_max || y_min>y_max )
      {
	ROS_ERROR("Youbot::SimpleBasePositionController:: /youbot_base/use_space_limits specified that space limits should be used but the parameter limits specified are invalid, that is at least one minima is larger than the corresponding maxima. Space limiting is disabled.");
	use_space_limits_ = false;
      }
      else
      {
	x_limits_ = std::pair<double,double>(x_min,x_max);
	y_limits_ = std::pair<double,double>(y_min,y_max);
      }
    }
  }
  
  
  //set up the publisher for the velocity command topic
  velocity_commander_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic_, 1000);
  
  // set up publisher for /joint_states topic
  if( publish_state_ )
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  
  last_published_ = ros::Time::now(); // just so that it has a value assigned
  
  init();
}

SimpleBasePositionController::~SimpleBasePositionController()
{
  halt(); // primitive safety
}

void SimpleBasePositionController::start()
{
  ros::Rate rate(50);
  ros::Time last_update = ros::Time::now();
  
  while( nh_.ok() )
  {
    rate.sleep();
    ros::Time now = ros::Time::now();
    ros::Duration time_passed = now - last_update;
    last_update = now;
    update( now, time_passed );
  }
}

void SimpleBasePositionController::setCommand(double _x, double _y, double _theta)
{
  boost::mutex::scoped_lock command_locker(command_protector_);
  command_.position.x = _x;
  command_.position.y = _y;
  command_.position.theta = _theta;
  command_.has_velocity = false;
}

void SimpleBasePositionController::setCommand( PlanarPosition _commanded_pos )
{
  boost::mutex::scoped_lock command_locker(command_protector_);
  command_.position = _commanded_pos;
  command_.has_velocity = false;
}

tf::StampedTransform SimpleBasePositionController::getCurrentTransform()
{
  ros::Time now = ros::Time::now();
  while( !listener_.waitForTransform(position_frame_, base_frame_, now, ros::Duration(1.0)) )
  {
    ROS_WARN_STREAM("Cannot get current state of youbot, missing tf transform from "<<position_frame_<<" to "<<base_frame_<<". Stopping the youbot for safety purposes while waiting.");
    halt();
  }
  
  tf::StampedTransform current_transform;
  // gets the transform that transforms data from base_frame to position_frame
  listener_.lookupTransform(position_frame_, base_frame_, now, current_transform);
  
  return current_transform;
}

SimpleBasePositionController::PlanarPosition SimpleBasePositionController::getBaseState( tf::StampedTransform& _transform )
{
  
  PlanarPosition state;
  state.x = _transform.getOrigin().x();
  state.y = _transform.getOrigin().y();
  state.theta = _transform.inverse().getRotation().getAngle();
  return state;
}

void SimpleBasePositionController::enforceVelocityLimits( PlanarVelocity& _velocity )
{
  if( !use_velocity_limits_ )
    return;
  
  singleVelLimitEnforcer( _velocity.dx, dx_limits_, dx_unfeasible_, unfeasible_rounding_threshold_linear_ );
  singleVelLimitEnforcer( _velocity.dy, dy_limits_, dy_unfeasible_, unfeasible_rounding_threshold_linear_ );
  singleVelLimitEnforcer( _velocity.dtheta, dtheta_limits_, dtheta_unfeasible_, unfeasible_rounding_threshold_angular_ );
}

void SimpleBasePositionController::singleVelLimitEnforcer( double& _value, std::pair<double,double>& _bounds, std::pair<double,double>& _feasibility_limits, double _feasibility_threshold )
{
  if( _value<_bounds.first )
    _value=_bounds.first;
  if( _value>_bounds.second )
    _value=_bounds.second;
  if( _value<0 && _value>_feasibility_limits.first )
  {
    if( _value<-_feasibility_threshold )
      _value=_feasibility_limits.first;
    else
      _value=0;
  }
  else if( _value>0 && _value<_feasibility_limits.second )
  {
    if( _value>_feasibility_threshold )
      _value=_feasibility_limits.second;
    else
      _value = 0;
  }
}

void SimpleBasePositionController::init()
{
  tf::StampedTransform current_transform = getCurrentTransform();
  PlanarPosition current_pos = getBaseState(current_transform);
  enforceSpaceLimits( current_pos );
  setCommand( current_pos );
  pid_x_.reset();
  pid_y_.reset();
  pid_theta_.reset();
}

void SimpleBasePositionController::update( const ros::Time& _time, const ros::Duration& _period )
{
  // get current command
  Commands current_command;
  // possibly critical section
  {
    boost::mutex::scoped_lock command_locker(command_protector_);
    current_command = command_;
  }
  PlanarPosition pos_error;
  PlanarVelocity control_command;
  
  PlanarVelocity vel_error;
  
  tf::StampedTransform current_transform = getCurrentTransform();
  PlanarPosition current_position = getBaseState(current_transform);
  
  // ensure command position is inside limits
  enforceSpaceLimits(current_command.position);
  
  using namespace std;
  cout<<endl<<"x position command:"<<current_command.position.x;
  cout<<endl<<"current x position:"<<current_position.x;
  
  pos_error.x = 0;//current_command.position.x - current_position.x;
  pos_error.y = current_command.position.y - current_position.y;
  pos_error.theta = angles::shortest_angular_distance( current_position.theta, current_command.position.theta );
  
  cout<<endl<<"position error in x is: "<<pos_error.x;
  cout<<endl<<"position error in y is: "<<pos_error.y;
  cout<<endl<<"position error in theta is: "<<pos_error.theta;
  
  // decide which of the two PID computeCommand() methods to call
  if( current_command.has_velocity )
  {
    // not implemented yet
  }
  else
  {
    // transform velocity vector from global frame to robot frame, construct command
    double pid_com_x = pid_x_.computeCommand( pos_error.x, _period );
    double pid_com_y = pid_y_.computeCommand( pos_error.y, _period );
    tf::Vector3 velocity_vec_global( pid_com_x, pid_com_y, 0 );
    tf::Quaternion velocity_vec_robot = current_transform.inverse().getRotation()*velocity_vec_global;
    control_command.dx = velocity_vec_robot.x();
    control_command.dy = velocity_vec_robot.y();
    control_command.dtheta = pid_theta_.computeCommand( pos_error.theta, _period );
      
    cout<<endl<<"pid computed commanded speed in x is: "<<pid_com_x;
    cout<<endl<<"pid computed commanded speed in y is: "<<pid_com_y;
    cout<<endl<<"pid computed commanded speed in theta is: "<<control_command.dtheta;
  }
  
  cout<<endl<<"transformed commanded speed in x is: "<<control_command.dx;
  cout<<endl<<"transformed commanded speed in y is: "<<control_command.dy;
  cout<<endl<<"transformed commanded speed in theta is: "<<control_command.dtheta;
  // enforce velocity _bounds
  enforceVelocityLimits(control_command);
  
  geometry_msgs::Twist control_output;
  control_output.linear.x = control_command.dx;
  control_output.linear.y = control_command.dy;
  control_output.linear.z = 0;
  control_output.angular.x = 0;
  control_output.angular.y = 0;
  control_output.angular.z = control_command.dtheta;
  
  publishCommand( control_output );
  
  // publish state if set
  if( publish_state_ && _time>(last_published_+publish_period_) )
  {
    sensor_msgs::JointState state;
    state.header.stamp = _time;
    state.header.frame_id = position_frame_;
    
    state.name.resize(3);
    state.position.resize(3);
    
    state.name[0] = base_x_name_;
    state.name[1] = base_y_name_;
    state.name[2] = base_theta_name_;
    state.position[0] = current_position.x;
    state.position[1] = current_position.y;
    state.position[2] = current_position.theta;
    
    state_publisher_.publish(state);
    
    last_published_ = _time;
  }
}

void SimpleBasePositionController::enforceSpaceLimits( PlanarPosition& _position )
{
  if( !use_space_limits_ )
    return;
  
  if( _position.x<x_limits_.first )
    _position.x = x_limits_.first;
  else if( _position.x>x_limits_.second )
    _position.x = x_limits_.second;
  if( _position.y<y_limits_.first )
    _position.y = y_limits_.first;
  else if( _position.y>y_limits_.second )
    _position.y = y_limits_.second;
}

void SimpleBasePositionController::publishCommand( geometry_msgs::Twist _base_velocity )
{
  velocity_commander_.publish( _base_velocity );
}

void SimpleBasePositionController::halt()
{
  geometry_msgs::Twist control_output;
  control_output.linear.x = 0;
  control_output.linear.y = 0;
  control_output.linear.z = 0;
  control_output.angular.x = 0;
  control_output.angular.y = 0;
  control_output.angular.z = 0;
  
  publishCommand( control_output );
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
    velocity_commander_.publish(base_cmd);
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
      velocity_commander_.publish(base_cmd);
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
    velocity_commander_.publish(base_cmd);
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
