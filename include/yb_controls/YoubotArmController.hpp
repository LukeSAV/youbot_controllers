  
 
/******************************************************************************
* Copyright (c) 2014
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch
*
*
* Class providing an interface for controlling the youbot arm.
*
******************************************************************************/

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
    
    static double pi;
};