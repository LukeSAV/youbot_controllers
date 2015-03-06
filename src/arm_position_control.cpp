
/* Copyright (c) 2014,2015 , Stefan Isler, islerstefan@bluewin.ch
*
This file is part of youbot_controllers, a ROS package providing some additonal controllers for the KUKA youbot,

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

#include "youbot_controllers/youbot_arm_controller.h"

/** ros node for more convenient manual control of the youbot arm */
int main(int argc, char **argv) {

	ros::init(argc, argv, "arm_position_control");
	ros::NodeHandle n;
	
	if( n.ok() )
	{
	  YoubotArmController armcontroller(&n);
	  armcontroller.run();
	}
	
	return 0;
}
