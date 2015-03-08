 
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

/// node starts up an instance of the SimpleBasePositionController
#include "youbot_controllers/simple_base_position_controller.h"
int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "base_position_controller");
  ros::NodeHandle nh;
  
  SimpleBasePositionController driver(nh);
  
  driver.start();
  
  return 0;
}