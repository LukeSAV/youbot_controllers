 
 
/******************************************************************************
* Copyright (c) 2014
*
* Author:
* Stefan Isler
*
*
* Class provides an interface for controlling the youbot arm.
*
******************************************************************************/

#include "youbot_controllers/YoubotArmController.hpp"


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
