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
#include <geometry_msgs/Pose2D.h>

/** node that expects 2d pose input on the command line from the user and sends it to a base_position_controller */

int main(int argc, char** argv)
{
  using namespace std;
  
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  
  ros::Publisher commander = nh.advertise<geometry_msgs::Pose2D>("/base_controller/position_command",1);
  
  double x,y,theta;
  
  while( nh.ok() )
  {
    cout<<endl<<endl<<"Please type in the target position (in m and rad) in the following format: x y theta"<<endl;
    
    cin>>x;
    cin>>y;
    cin>>theta;
    
    geometry_msgs::Pose2D command;
    command.x=x;
    command.y=y;
    command.theta=theta;
    commander.publish(command);
  }
  
  return 0;
}