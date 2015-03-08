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
#include <youbot_controllers/base_trajectory_action.h>

/** sets up a trajectory action server that can interface with the base_position_controller*/

int main(int argc, char** argv)
{
  using namespace std;
  
  //init the ROS node
  ros::init(argc, argv, "base_trajectory_action_server");
  ros::NodeHandle nh;
    
  BaseTrajectoryAction server(nh);
  
  ROS_INFO("BaseTrajectoryAction is up and running.");
  
  ros::spin();
  
  return 0;
}