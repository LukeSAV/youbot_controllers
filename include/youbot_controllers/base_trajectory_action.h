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
 * partly based on the pr2/youbot::BaseTrajectoryAction by 
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 */

/** loads base_controller namespace and node name from parameter server, just as the simple base position controller
 */

#pragma once

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <actionlib/server/action_server.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

// still quite a mess, this is just a fast fix to get the job done

/*namespace KDL
{
class Trajectory_Composite;
}*/


class BaseTrajectoryAction
{
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> Server;
typedef Server::GoalHandle GoalHandle;

public:
    BaseTrajectoryAction( ros::NodeHandle& _nh );
    virtual ~BaseTrajectoryAction();

    void goalCallback(GoalHandle _goal);
    
    void cancelCallback(GoalHandle _goal);

    void execute( const control_msgs::FollowJointTrajectoryGoalConstPtr& goal );


    void setFrequency(double frequency);
    double getFrequency() const;

    // tells the BaseTrajectoryAction class to use position commands when following a trajectory: velocity and/or accelerations in the trajectory are not considered
    void usePositionCommands();
    
    // tells the BaseTrajectoryAction class to use velocity commands when following a trajectory: currently the velocities are being recalculated using the KDL library and the positions in the trajectory while discarding velocities and/or accelerations, you might want to change that and do interpolations using the velocities in the trajectory if you want to use the class with velocity commands (Velocity and torque commands weren't necessary for the task for which the class has been revised since timing was of no interest, thus this wasn't done) -> check the getAllCurrentVelocities(...) function
    void useVelocityCommands();
    
    // sets ignore_time_ to false
    void dontIgnoreTime();
    // sets ignore_time to true
    void ignoreTime();
private:
    ros::NodeHandle nh_;
    
    bool has_active_goal_;
    GoalHandle active_goal_;
    boost::shared_ptr<Server> trajectory_server_;
    std::string base_link_name_; /// name of the base link
    std::string base_control_ns_; /// name of the ns in which the position_command topic exists
    ros::Publisher commander_; /// commands 
  
    bool use_position_commands_; /// if true then only the positions of the trajectory are used, otherwise the velocities will be used - default is true
    bool ignore_time_; /// if true then each point in the trajectory will be commanded with no regard to the the time in the trajectory. Currently defaults to true because accurate trajectory following was desired and timing was of no interest for the task at hand for which this class was revised
    bool no_interpolation_; /// don't interpolate positions of received trajectory, only resend what is received
    
    double frequency;
    
    
    /** returns the velocity given in the trajectory trajectoryComposite at time elapsedTimeInSec. If the elapsed time exceeds the time inside the trajectory, velocity zero is returned - unused, comes from model
     */
    /*double getVelocityAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);*/
    
    /** returns the position given in the trajectory trajectoryComposite at time elapsedTimeInSec - unused, comes from model
     * @throws std::runtime_error If the elapsed time exceeds the time available in the trajectory
     */
    /*double getPositionAtTime( const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);*/
    
    /** gets all velocities for the current time (using ros::Time::now() ) as given in the trajectory - unused, comes from model
     * @param startTime the time at which trajectory execution started (only used if ignore_time_ is false)
     * @param currentTime the time for which the velocites are wanted (only used if ignore_time_ is true)
     * @return writes into the vector velocities
     */
    /*void getAllCurrentVelocities( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& velocities);*/
    
    /** gets all positions for the current time (using ros::Time::now() ) as given in the trajectory - unused, comes from model
     * @param startTime the time at which trajectory execution started (only used if ignore_time_ is false)
     * @param currentTime the time for which the velocites are wanted (only used if ignore_time_ is true)
     * @return writes into the vector positions
     */
    /*void getAllCurrentPositions( const KDL::Trajectory_Composite* trajectory,
                     int numberOfJoints,
                     ros::Time startTime,
		     double currentTime,
                     std::vector<double>& positions);*/

    /*void setTargetTrajectory(double angle1,
                             double angle2,
                             double duration,
                             KDL::Trajectory_Composite& trajectoryComposite);*/

};

