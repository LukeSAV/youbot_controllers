 
 
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

#include "yb_controls/YoubotArmController.hpp"


YoubotArmController::YoubotArmController( ros::NodeHandle* _n )
{
  cout<<endl<<"Initializing Youbot arm controller.."<<endl;
  
  parentNode_ = _n;
  
  armStatePublisher_ = _n->advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",10);
  armStateListener_ = _n->subscribe("/joint_states",1,&YoubotArmController::stateUpdate, this);
  
  jointNames_.push_back("arm_joint_1");
  jointNames_.push_back("arm_joint_2");
  jointNames_.push_back("arm_joint_3");
  jointNames_.push_back("arm_joint_4");
  jointNames_.push_back("arm_joint_5");
  
  joint_.resize( jointNames_.size(),-1 );
  commandedPositions_.resize( jointNames_.size() );
  
  // data limits from (calibrated) hardware, rounded up and down to next rad/1000
  jointLimits_.push_back( pair<double,double>(0.011, 5.840) ); //joint 1
  jointLimits_.push_back( pair<double,double>(0.011, 2.617) ); //joint 2
  jointLimits_.push_back( pair<double,double>(-5.026, -0.016) ); //joint 3
  jointLimits_.push_back( pair<double,double>(0.023, 3.429) ); //joint 4
  jointLimits_.push_back( pair<double,double>(0.111, 5.641) ); //joint 5
  
  currentJoint_ = 1;
  currentStepSize_ = 0.1; //rad
  
  joint5Even_ = 2.9;
  return;
}

YoubotArmController::~YoubotArmController()
{
  
}

void YoubotArmController::stateUpdate( const sensor_msgs::JointStateConstPtr& _newState )
{
  armState_ = *_newState;
  return;
}


void YoubotArmController::run()
{
  while( armState_.name.empty() )
  {
    //wait until youbot arm state data arrived
    ros::spinOnce();
    ros::Rate rate(10); //Hz
    rate.sleep();
    if( !parentNode_->ok() ) return;
    if( armState_.name.empty() ) cout<<endl<<"Waiting for youbot arm to publish data...";
  }
  // initialize command with current position and find position of joint states in transmitted arrays
  for( int i=0; i<armState_.name.size(); i++ )
  {
    for( int j=0; j<jointNames_.size(); j++ )
    {
      if( armState_.name[i]==jointNames_[j] ) joint_[j]=i;
    }
  }
  for( int i=0; i<5; i++ )
  {
    if( joint_[i]==-1 )
    {
      cout<<endl<<"YoubotArmController::run()::ERROR occured: joint with name "<<jointNames_[i]<<" wasn't found in transmitted data. Shutting down arm controller."<<endl;
      return;
    }
  }
  //resetPos(); // initializes commanded positions to lower joint limits
  stopArm(); // initializes commanded positions to current arm positions
  
  cout<<endl<<"Controller is ready."<<endl;
  char userInput;
  
  printHelp();
  
  while( parentNode_->ok() )
  {
    
      cout<<"Controlling joint "<<currentJoint_<<"."<<endl<<"Enter command."<<endl;
      
      cin >> userInput;
      switch(userInput)
      {
	case '?':
	  printHelp();
	  break;
	case ',':
	  setActiveJoint( currentJoint_-1 );
	  break;
	case '.':
	  setActiveJoint( currentJoint_+1 );
	  break;
	case '+':
	  commandedPositions(currentJoint_) += currentStepSize_;
	  publishCommand();
	  break;
	case '-':
	  commandedPositions(currentJoint_) -= currentStepSize_;
	  publishCommand();
	  break;
	case 'y':
	  currentStepSize_-=0.05;
	  if( currentStepSize_<0 ) currentStepSize_=0;
	  cout<<endl<<"New step size: "<<currentStepSize_<<" rad."<<endl;
	  break;
	case 'x':
	  currentStepSize_+=0.05;
	  if( currentStepSize_<0 ) currentStepSize_=0;
	  cout<<endl<<"New step size: "<<currentStepSize_<<" rad."<<endl;
	  break;
	case 't': //enter
	  cout<<endl<<"Enter position to move to. Current position is "<<commandedPositions(currentJoint_)<<" rad."<<endl;
	  double newPos;
	  cin >> newPos;
	  commandedPositions(currentJoint_) = newPos;
	  publishCommand();
	  break;
	case 'z': //position reset
	  resetPos();
	  break;
	case 'u':
	  upPos();
	  break;
	case 'm':
	  midPos();
	  break;
	case 'd':
	  downPos();
	  break;
	case 's':
	  skyPos();
	  break;
	case 'f':
	  baseForward();
	  break;
	case 'b':
	  baseBackward();
	  break;
	case 'l':
	  baseLeft();
	  break;
	case 'r':
	  baseRight();
	  break;
	case 'h':
	  stopArm();
	  break;
	case 'q':
	  return;
	default:
	  if( userInput>=49 && userInput<=57 ) // joints 1 to 9
	  {
	    setActiveJoint( (int)userInput-48 );
	  }
	  break;
      }
      cin.ignore(cin.gcount());
      ros::spinOnce(); // not really necessary: don't need to retrieve any data for the current functionality...
    
  }
  return;
}


void YoubotArmController::publishCommand()
{
  brics_actuator::JointPositions command;
  command.poisonStamp.originator = "YoubotArmController";
  
  for(int i=0;i<jointNames_.size();i++)
  {
    brics_actuator::JointValue toAdd;
    toAdd.timeStamp = ros::Time::now();
    toAdd.joint_uri = jointNames_[i];
    toAdd.unit = "rad";
    double posCommand = commandedPositions_[i];
    if( posCommand<jointLimits_[i].first )
    {
      cout<<endl<<"Entered value "<<posCommand<<" is lower than limit, using limit: "<<jointLimits_[i].first<<endl;
      posCommand = jointLimits_[i].first;
    }
    else if( posCommand>jointLimits_[i].second )
    {
      cout<<endl<<"Entered value "<<posCommand<<" exceeds joint limit, using limit: "<<jointLimits_[i].second<<endl;
      posCommand = jointLimits_[i].second;
    }
    toAdd.value = posCommand;
    
    command.positions.push_back(toAdd);
  }
  
  cout<<endl<<"publishing command"<<endl;
  armStatePublisher_.publish(command);
  return;
}


void YoubotArmController::stopArm()
{
  ros::spinOnce(); // retrieve newest data...
  for( int i=0; i<jointNames_.size(); i++ )
  {
    commandedPositions_[i] = armState_.position[joint_[i]];
  }
  publishCommand();
  return;
}

void YoubotArmController::setActiveJoint( int _jointId )
{
  if( _jointId<=0 ) currentJoint_=1;
  else if( _jointId>jointNames_.size() ) currentJoint_=jointNames_.size();
  else currentJoint_=_jointId;
  return;
}


void YoubotArmController::resetPos()
{
  for( int i=0; i<jointNames_.size(); i++ )
  {
    commandedPositions_[i] = jointLimits_[i].first;
  }
  publishCommand();
  return;
}

void YoubotArmController::upPos()
{
  commandedPositions(2) = 1.15;
  commandedPositions(3) = -2.6;
  commandedPositions(4) = 3.4;
  commandedPositions(5) = joint5Even_;
  
  publishCommand();
  return;
}

void YoubotArmController::midPos()
{
  commandedPositions(2) = 0.6;
  commandedPositions(3) = -1.3;
  commandedPositions(4) = 2.6;
  commandedPositions(5) = joint5Even_;
  
  publishCommand();
  return;
}

void YoubotArmController::downPos()
{
  commandedPositions(2) = 0.011;
  commandedPositions(3) = -0.016;
  commandedPositions(4) = 1.9;
  commandedPositions(5) = joint5Even_;
  
  publishCommand();
  return;
}

void YoubotArmController::skyPos()
{
  commandedPositions(2) = 1.15;
  commandedPositions(3) = -2.6;
  commandedPositions(4) = 1.88;
  commandedPositions(5) = joint5Even_;
  
  publishCommand();
  return;
}

void YoubotArmController::baseForward()
{
  commandedPositions(1) = 2.95;
  publishCommand();
  return;
}

void YoubotArmController::baseBackward()
{
  commandedPositions(1) = 2.95-pi;
  publishCommand();
  return;
}

void YoubotArmController::baseLeft()
{
  commandedPositions(1) = 1.35;
  publishCommand();
  return;
}

void YoubotArmController::baseRight()
{
  commandedPositions(1) = 4.5;
  publishCommand();
  return;
}


void YoubotArmController::printHelp()
{
  cout<<"You can switch through joints by entering either a number or through ','/'.'.\nMove the current joint relative to its position using arrow +/-.\nMove faster using y/x\nEnter a new joint position numerically by pressing t>>type position\nMove to predefined positions with:\nz(zero position) u(up) m(mid) d(down) s(skywatch)\nf(base points forwards) l(base points left) r(base points right) b(base points back)\nStop the movement (hold position): h, quit program: q, print this text:?"<<endl<<endl;
  return;
}


int YoubotArmController::joint(int _i)
{
  if( _i<=0 ) return joint_[0];
  
  return joint_[_i-1];
}


double& YoubotArmController::commandedPositions( int _pos )
{
  if( _pos<0 ) return commandedPositions_[0];
  else if( _pos>= commandedPositions_.size() ) return commandedPositions_[commandedPositions_.size()-1];
  
   return commandedPositions_[_pos-1];
}

double YoubotArmController::pi = 3.14159265358979;