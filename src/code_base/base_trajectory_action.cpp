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

/*
 * originally based on the pr2::BaseTrajectoryAction by 
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 */

#include <youbot_controllers/base_trajectory_action.h>
#include <geometry_msgs/Pose2D.h>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>

#include <control_msgs/FollowBaseTrajectoryActionResult.h>


BaseTrajectoryAction::BaseTrajectoryAction( ros::NodeHandle& _nh, std::string _base_link_name, std::string _base_control_ns ) :
  use_position_commands_(true),ignore_time_(true),no_interpolation_(true), nh_(_nh),  base_link_name_(_base_link_name), base_control_ns_(_base_control_ns)
{
    setFrequency(50);
    commander_ = nh_.advertise<geometry_msgs::Pose2D>("/"+base_control_ns_+"/position_command", 1);
    
    // starts up a simple trajectory server instance
    trajectory_server_ = boost::shared_ptr<Server>( new Server("/"+base_control_ns_+"/follow_joint_trajectory", boost::bind(&BaseTrajectoryAction::executeActionServer, this, _1, trajectory_server_), false) );
    trajectory_server_->start();
}

BaseTrajectoryAction::~BaseTrajectoryAction()
{

}

void BaseTrajectoryAction::executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& _goal, Server* _as)
{
  execute(goal, trajectory_server_);
}

void BaseTrajectoryAction::execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{

    /* unused: unnecessary without error calculations, reenable if using controlLoop/calculateVelocity
    current_state.name = goal->trajectory.joint_names;
    current_state.position.resize(current_state.name.size());
    current_state.velocity.resize(current_state.name.size());
    current_state.effort.resize(current_state.name.size());
    */

    sensor_msgs::JointState angle1;
    /*angle1.name = goal->trajectory.joint_names; // unused
    angle1.position.resize(angle1.name.size());
    angle1.velocity.resize(angle1.name.size());
    angle1.effort.resize(angle1.name.size());
*/
    sensor_msgs::JointState angle2;
    /*angle2.name = goal->trajectory.joint_names; // unused
    angle2.position.resize(angle2.name.size());
    angle2.velocity.resize(angle2.name.size());
    angle2.effort.resize(angle2.name.size());
*/
    const uint numberOfJoints = goal->trajectory.joint_names.size();//current_state.name.size();
    const double dt = 1.0 / getFrequency();
    
    if( false&&!no_interpolation_ ) //interpolation has not been matched to base joint
    {
      
      KDL::Trajectory_Composite trajectoryComposite[numberOfJoints];

      angle2.position = goal->trajectory.points.at(0).positions;
      for (uint i = 1; i < goal->trajectory.points.size(); i++)
      {
	  angle1.position = angle2.position;
	  angle2.position = goal->trajectory.points.at(i).positions;

	  double duration = (goal->trajectory.points.at(i).time_from_start -
		  goal->trajectory.points.at(i - 1).time_from_start).toSec();

	  for (uint j = 0; j < numberOfJoints; ++j)
	  {
	      setTargetTrajectory(angle1.position.at(j),
				  angle2.position.at(j),
				  duration,
				  trajectoryComposite[j]);

	  }
      }

      std::vector<double> target_values; // position or velocity, little weird like that but don't want to rewrite whole code
      ros::Time startTime = ros::Time::now();

      for (double time = 0; time <= trajectoryComposite[0].Duration(); time = time + dt)
      {
	  /* // uh-uh (imho control loops are the task of the PID controller!)
	  controlLoop(current_state.position,
		      current_state.velocity,
		      trajectoryComposite,
		      numberOfJoints,
		      startTime,
		      target_values);
	  */
	  
	  // using velocity commands
	  if( !use_position_commands_ )
	  {
	    getAllCurrentVelocities( trajectoryComposite, numberOfJoints, startTime, time, target_values );
		    
	    brics_actuator::JointVelocities command;
	    std::vector <brics_actuator::JointValue> armJointVelocities;
	    armJointVelocities.resize(numberOfJoints);

	    for (uint j = 0; j < numberOfJoints; j++)
	    {
		armJointVelocities[j].joint_uri = goal->trajectory.joint_names[j];//current_state.name.at(j);
		armJointVelocities[j].value = target_values[j];
		armJointVelocities[j].unit = boost::units::to_string(boost::units::si::radian_per_second);
	    }

	    command.velocities = armJointVelocities;
	    jointStateObserver->updateVelocity(command);
	  }
	  else // using position commands
	  {
	    getAllCurrentPositions( trajectoryComposite, numberOfJoints, startTime, time, target_values );
	    
	    brics_actuator::JointPositions command;
	    std::vector <brics_actuator::JointValue> armJointPositions;
	    armJointPositions.resize(numberOfJoints);

	    for (uint j = 0; j < numberOfJoints; j++)
	    {
		armJointPositions[j].joint_uri = goal->trajectory.joint_names[j];//current_state.name.at(j);
		armJointPositions[j].value = target_values[j];
		armJointPositions[j].unit = boost::units::to_string(boost::units::si::radian);
	    }
	    using namespace std;
	    command.positions = armJointPositions;
	    
	    //if( time<(4*dt) || time>(trajectoryComposite[0].Duration()-4*dt) )
	    //  cout<<endl<<endl<<"issued position command is:"<<endl<<command<<endl<<endl;
	    jointStateObserver->updatePosition(command);
	  }

	  ros::Duration(dt).sleep();
      }
    }
    else
    {
      unsigned int next_trajectory_point = 0;
      for (double time = 0; time <= (goal->trajectory.points.back().time_from_start.toSec()+dt); time = time + dt)
      {
	if( time >= goal->trajectory.points[next_trajectory_point].time_from_start.toSec() ) // about time to send next position command -> can never follow the set time since this point should at least already be reached now
	{
	  
	  geometry_msgs::Pose2D command;
	  
	  
	  for (uint j = 0; j < numberOfJoints; j++)
	  {
	    if( goal->trajectory.joint_names[j]==(base_link_name_+"/x") )
	    {
	      command.x = goal->trajectory.points[next_trajectory_point].positions[j];
	    }
	    else if( goal->trajectory.joint_names[j]==(base_link_name_+"/y") )
	    {
	      command.y = goal->trajectory.points[next_trajectory_point].positions[j];
	    }
	    else if( goal->trajectory.joint_names[j]==(base_link_name_+"/theta") )
	    {
	      command.theta = goal->trajectory.points[next_trajectory_point].positions[j];
	    }
	  }
	  commander_.publish(command);
	  
	  //using namespace std;
	  //cout<<endl<<endl<<"issued position command is:"<<endl<<command<<endl<<endl;
	  
	  next_trajectory_point++;
	  if( next_trajectory_point >= goal->trajectory.points.size() )
	    break;
	}
	
	ros::Duration(dt).sleep();
      }
    }

    // to ensure that at the end of the trajectory the robot is at the goal state, they issue a position command for that last state (especially for velocity control)
    sensor_msgs::JointState goal_state;
    goal_state.name = goal->trajectory.joint_names;
    goal_state.position = goal->trajectory.points.back().positions;

    geometry_msgs::Pose2D command;

    for (uint j = 0; j < numberOfJoints; j++)
    {
        if( goal->trajectory.joint_names[j]==(base_link_name_+"/x") )
	{
	  command.x = goal->trajectory.points[next_trajectory_point].positions[j];
	}
	else if( goal->trajectory.joint_names[j]==(base_link_name_+"/y") )
	{
	  command.y = goal->trajectory.points[next_trajectory_point].positions[j];
	}
	else if( goal->trajectory.joint_names[j]==(base_link_name_+"/theta") )
	{
	  command.theta = goal->trajectory.points[next_trajectory_point].positions[j];
	}
    }

    commander_.publish(command);
        
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    as->setSucceeded(result);
    return;    
}

void BaseTrajectoryAction::setFrequency(double frequency)
{
    this->frequency = frequency;
}

double BaseTrajectoryAction::getFrequency() const
{
    return frequency;
}

void BaseTrajectoryAction::usePositionCommands()
{
  use_position_commands_ = true;
}

void BaseTrajectoryAction::useVelocityCommands()
{
  use_position_commands_ = false;
}

void BaseTrajectoryAction::dontIgnoreTime()
{
  ignore_time_ = false;
}

void BaseTrajectoryAction::ignoreTime()
{
  ignore_time_ = true;
}

double BaseTrajectoryAction::getVelocityAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec)
{

    double velocity = 0;

    if (trajectoryComposite.Duration() > 0 && elapsedTimeInSec <= trajectoryComposite.Duration())
    {
        double actualTime = elapsedTimeInSec;
        double desiredVelocity = trajectoryComposite.Vel(actualTime).vel.x();
	velocity = desiredVelocity;
    }

    return velocity;
}

double BaseTrajectoryAction::getPositionAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec)
{

    double position = 0;

    if (trajectoryComposite.Duration() > 0 && elapsedTimeInSec <= trajectoryComposite.Duration())
    {
        double actualTime = elapsedTimeInSec;
        double desiredAngle = trajectoryComposite.Pos(actualTime).p.x();
	position = desiredAngle;
	
	return position;
    }
    else
    {
      throw std::runtime_error("BaseTrajectoryAction::getPositionAtTime:: Time given exceeded the time available in the trajectory.");
      return position; // (never called)
    }

}

void BaseTrajectoryAction::getAllCurrentVelocities( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& velocities)
{

    velocities.clear();
    
    double elapsedTime;
    if( !ignore_time_ )
      elapsedTime = ros::Duration(ros::Time::now() - startTime).toSec();
    else
      elapsedTime = currentTime;

    for (int i = 0; i < numberOfJoints; ++i)
    {
        double velocity = getVelocityAtTime(trajectory[i],
                                            elapsedTime);

        velocities.push_back(velocity);

    }

}

void BaseTrajectoryAction::getAllCurrentPositions( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& positions)
{

    positions.clear();
    
    double elapsedTime;
    if( !ignore_time_ )
      elapsedTime = ros::Duration(ros::Time::now() - startTime).toSec();
    else
      elapsedTime = currentTime;

    for (int i = 0; i < numberOfJoints; ++i)
    {
	double position;
	
	try
	{
	  position = getPositionAtTime(trajectory[i],
                                            elapsedTime);
	}
	catch( std::runtime_error )
	{
	  return;
	}

        positions.push_back(position);

    }

}

void BaseTrajectoryAction::setTargetTrajectory(double angle1,
                                                double angle2,
                                                double duration,
                                                KDL::Trajectory_Composite& trajectoryComposite)
{

    KDL::Frame pose1(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle1, 0, 0));
    KDL::Frame pose2(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle2, 0, 0));

    KDL::Path_Line* path = new KDL::Path_Line(pose1, pose2, new KDL::RotationalInterpolation_SingleAxis(), 0.001);
    KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();

    velprof->SetProfileDuration(0, path->PathLength(),
                                duration);

    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(path, velprof);
    trajectoryComposite.Add(trajectorySegment);
}

void BaseTrajectoryAction::jointStateCallback(const sensor_msgs::JointState& joint_state)
{
  return;
    int k = joint_state.name.size();
    
    if( k>current_state.name.size() )
    {
      current_state.name.resize(k);
      current_state.position.resize(k);
      current_state.velocity.resize(k);
    }
    //int k = current_state.name.size();
    
    if (k != 0)
    {
        for (uint i = 0; i < joint_state.name.size(); i++)
        {
            for (uint j = 0; j < current_state.name.size(); j++)
            {
                if (current_state.name.at(j) == joint_state.name.at(i))
                {
                    current_state.position[j] = joint_state.position.at(i);
		    current_state.velocity[j] = joint_state.velocity.at(i);
                    k--;
                    break;
                }
            }
            if (k == 0)
            {
                break;
            }
        }
    }
}

