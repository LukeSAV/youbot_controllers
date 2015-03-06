
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


#include <iostream>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointValue.h"
#include "sensor_msgs/JointState.h"

using namespace std;

class YoubotArmController
{
  public:
    YoubotArmController( ros::NodeHandle* _n );
    ~YoubotArmController();
    
    /** starts the arm controller */
    void run();

    void stateUpdate( const sensor_msgs::JointStateConstPtr& _newState );
  private:
    double joint5Even_; // the position at which joint 5 is even
    vector< pair<double,double> > jointLimits_; // lower/upper joint limits
    
    ros::Publisher armStatePublisher_;
    ros::Subscriber armStateListener_;
    
    void retrieveNewData();
    
    /** publishes the current command set */
    void publishCommand();
    /** stops the robot's arm movement by setting the commanded positions to the current values*/
    void stopArm();
    /** sets new current joint, ensuring the range */
    void setActiveJoint( int _jointId );
    
    //predefined positions (sets internal command parameter and publishes the command)
    void resetPos(); // sets all joints to lower limit values
    void upPos();
    void midPos();
    void downPos();
    void skyPos();
    void baseForward();
    void baseBackward();
    void baseLeft();
    void baseRight();
    
    // prints help to console
    void printHelp();
    
    ros::NodeHandle* parentNode_;
    sensor_msgs::JointState armState_; // last received state of the youbot arm
    vector<string> jointNames_;
    vector<int> joint_; // stores the position each joint has in the transmitted arrays
    vector<double> commandedPositions_; // currently commanded joint positions
    int joint(int _i); // translates the joint numbering from the set (1,2,3,...)[_i] to (0,1,2,...)[internally] and returns the id of the joint inside the array
    double& commandedPositions( int _pos ); //access interface to commandedPositions_ vector where _pos=1 is 0, _pos=2 is 1, etc (internally)
    int currentJoint_; //currently controlled joint
    double currentStepSize_; //currently used step size
    bool newDataReceived_;
    
    static double pi;
};