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
#include <iostream>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

/** node that expects 2d pose input on the command line from the user and sends it to a base_position_controller */

int main(int argc, char** argv)
{
  using namespace std;
  
  //init the ROS node
  ros::init(argc, argv, "trajectory_test");
  ros::NodeHandle nh;
  
  Client client("base_controller/follow_joint_trajectory", true);
  client.waitForServer();
  
  control_msgs::FollowJointTrajectoryGoal goal;
  
  goal.trajectory.joint_names.push_back("youbot_base/x");
  goal.trajectory.joint_names.push_back("youbot_base/y");
  goal.trajectory.joint_names.push_back("youbot_base/theta");
  
  trajectory_msgs::JointTrajectoryPoint eins, zwei, drei, vier;
  
  eins.positions.push_back(0);
  eins.positions.push_back(1);
  eins.positions.push_back(0);
  eins.time_from_start = ros::Duration(0);
  
  zwei.positions.push_back(1);
  zwei.positions.push_back(1);
  zwei.positions.push_back(1.57);
  zwei.time_from_start = ros::Duration(10);
  
  drei.positions.push_back(2);
  drei.positions.push_back(1);
  drei.positions.push_back(3.14159);
  drei.time_from_start = ros::Duration(15);
  
  vier.positions.push_back(2);
  vier.positions.push_back(2);
  vier.positions.push_back(4.71);
  vier.time_from_start = ros::Duration(25);
  
  goal.trajectory.points.push_back(eins);
  goal.trajectory.points.push_back(zwei);
  goal.trajectory.points.push_back(drei);
  goal.trajectory.points.push_back(vier);
  
  client.sendGoal(goal);
  
  client.waitForResult(ros::Duration(40.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  
  return 0;
  
}