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
  
  if( !nh_.getParam("/youbot_base/base_controller_ns", base_controller_ns_ ) )
  {
    ROS_INFO("SimpleBasePositionController couldn't find youbot_base/base_controller_ns parameter on the parameter server. The default (base_controller) will be used.");
    base_controller_ns_ = "base_controller";
  }
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
  else
  {
    base_x_name_ = base_name_+"/x";
    base_y_name_ = base_name_+"/y";
    base_theta_name_ = base_name_+"/theta";
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
  if( !pid_pos_.init( ros::NodeHandle(nh_, "/youbot_base/pid_pos") ) ||
      !nh_.getParam("/youbot_base/pid_pos/p", p_pos ) ||
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
    double dx_min, dx_max, dtheta_min, dtheta_max, dx_feas_min, dx_feas_max, dtheta_feas_min, dtheta_feas_max;
    if( !nh_.getParam("/youbot_base/velocity_limits/upper/linear/min", dx_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/linear/max", dx_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/angular/min", dtheta_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/upper/angular/max", dtheta_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/linear/neg", dx_feas_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/linear/pos", dx_feas_max ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/angular/neg", dtheta_feas_min ) || 
        !nh_.getParam("/youbot_base/velocity_limits/unfeasible/angular/pos", dtheta_feas_max ) ||
        !nh_.getParam("/youbot_base/unfeasible_rounding_threshold/linear", unfeasible_rounding_threshold_linear_ ) || 
        !nh_.getParam("/youbot_base/unfeasible_rounding_threshold/angular", unfeasible_rounding_threshold_angular_ ) )
    {
      ROS_FATAL("SimpleBasePositionController:: youbot_base/use_velocity_limits is set to true but not all necessary configurations have been specified. Shutting down node.");
      ros::shutdown();
      return;
    }
    else
    {
      dlin_limits_ = std::pair<double,double>(dx_min,dx_max);
      dang_limits_ = std::pair<double,double>(dtheta_min,dtheta_max);
      dlin_unfeasible_ = std::pair<double,double>(dx_feas_min,dx_feas_max);
      dang_unfeasible_ = std::pair<double,double>(dtheta_feas_min,dtheta_feas_max);
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
  
  // start position_command subscription
  position_commands_ = nh_.subscribe( "/"+base_controller_ns_+"/position_command", 1, &SimpleBasePositionController::positionCommandCallback, this );
}

SimpleBasePositionController::~SimpleBasePositionController()
{
  halt(); // primitive safety
}

void SimpleBasePositionController::start()
{
  ros::Rate rate(50);
  init();
  
  while( nh_.ok() )
  {
    rate.sleep();
    doUpdate();
    ros::spinOnce();
  }
}

void SimpleBasePositionController::doUpdate()
{
  ros::Time now = ros::Time::now();
  update( now );
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
  //state.theta = _transform.inverse().getRotation().getAngle(); // transform probably finds the shorter
  tf::Quaternion rot = _transform.getRotation();
  if( rot.z()>=0 )
    state.theta = 2*acos(rot.w());
  else
    state.theta = 2*acos(-rot.w());
  return state;
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
  last_update_ = ros::Time::now();
}

void SimpleBasePositionController::update( const ros::Time& _time, const ros::Duration& _period )
{
  
  ros::Duration period;
  if( _period==ros::Duration() )
    period = _time-last_update_;
  else
    period = _period;
  
  last_update_ = _time;
  
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
  cout<<endl<<"y position command:"<<current_command.position.y;
  cout<<endl<<"theta position command:"<<current_command.position.theta;
  cout<<endl<<"current x position:"<<current_position.x;
  cout<<endl<<"current y position:"<<current_position.y;
  cout<<endl<<"current theta position:"<<current_position.theta;
  
  pos_error.x = current_command.position.x - current_position.x; // in global (e.g. odometry) frame
  pos_error.y = current_command.position.y - current_position.y;
  pos_error.theta = angles::shortest_angular_distance( current_position.theta, current_command.position.theta );
  
  cout<<endl<<"position error in x is: "<<pos_error.x;
  cout<<endl<<"position error in y is: "<<pos_error.y;
  cout<<endl<<"position error in theta is: "<<pos_error.theta;
  
  // decide which of the two PID computeCommand() methods to call
  if( current_command.has_velocity )
  {
    // not implemented (yet?)
  }
  else
  {
    // direction to move to: needs to be transformed from global frame to local robot frame
    tf::Vector3  velocity_dir_global( pos_error.x, pos_error.y, 0 );
    tf::Transform R_rb = current_transform.inverse(); // transforming base entities to robot entities
    R_rb.setOrigin( tf::Vector3(0,0,0) ); // want only the rotation
    
    tf::Vector3 velocity_dir_robot = R_rb*velocity_dir_global;
    
    
    velocity_dir_robot.normalize();
    
    // use pid to calculate velocities
    double dist_error = velocity_dir_global.length();
    double lin_speed_command = fabs(pid_pos_.computeCommand( dist_error, period ));
    double ang_speed_command = pid_theta_.computeCommand( pos_error.theta, period );
        
    cout<<endl<<"transformed commanded speed in pos is: "<<lin_speed_command;
    cout<<endl<<"transformed commanded speed in theta is: "<<ang_speed_command;
    
    if( use_velocity_limits_ ) // enforce velocity limits
    {
      singleVelLimitEnforcer( lin_speed_command, dlin_limits_, dlin_unfeasible_, unfeasible_rounding_threshold_linear_ );
      singleVelLimitEnforcer( ang_speed_command, dang_limits_, dang_unfeasible_, unfeasible_rounding_threshold_angular_ );
    }
      
    cout<<endl<<"transformed commanded limited speed in pos is: "<<lin_speed_command;
    cout<<endl<<"transformed commanded limited speed in theta is: "<<ang_speed_command;
    
    velocity_dir_robot *= lin_speed_command;
    
    //double pid_com_x = pid_x_.computeCommand( pos_error.x, _period );
    //double pid_com_y = pid_y_.computeCommand( pos_error.y, _period );
    //tf::Vector3 velocity_vec_global( pid_com_x, pid_com_y, 0 );
    //tf::Quaternion velocity_vec_robot = current_transform.inverse().getRotation()*velocity_vec_global;
    
    control_command.dx = velocity_dir_robot.x();
    control_command.dy = velocity_dir_robot.y();
    control_command.dtheta = ang_speed_command;
    
    
    cout<<endl<<"transformed commanded limited speed in x is: "<<control_command.dx;
    cout<<endl<<"transformed commanded limited speed in y is: "<<control_command.dy;
    cout<<endl<<"transformed commanded limited speed in theta is: "<<ang_speed_command;
    
  }
  
  
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

void SimpleBasePositionController::positionCommandCallback( const geometry_msgs::Pose2DConstPtr& _position_command )
{
  setCommand( _position_command->x, _position_command->y, _position_command->theta );
}