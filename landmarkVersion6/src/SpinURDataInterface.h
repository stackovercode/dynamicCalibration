/*
#pragma once

/////////////////////////////////////////////////////////////////////////////
// Includes

// ROS
//#include <ros/ros.h>
#include <RTDEInterface.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <spin_msgs/RobotTime.h>
#include <spin_msgs/RobotForce.h>
#include <spin_msgs/RobotPose.h>
#include <spin_msgs/RobotVelocity.h>
#include <spin_msgs/RobotStatus.h>

// Utilities
#include <iostream>
#include <boost/shared_ptr.hpp>
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Defines
#define DEFAULT_SPIN_RATE                           10
#define DEFAULT_IP_ADDRESS                          "127.0.0.1"
#define DEFAULT_TIMEOUT_RECONNECT                   5

#define MOVING_SPEED_THRESHOLD                      0.001
#define QUEUE_SIZE                                  1000

#define NODE_SPIN_ROBOT_DATA_INTERFACE              (std::string)"/spin_robot_data_interface/"
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespace
namespace spin_rpi
{
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Class
class SpinURDataInterface
{
public:
    // Constructors
    SpinURDataInterface();
    ~SpinURDataInterface() { }

private:
    // Functions
    void rosLoop();
    void handleTopics();
    bool checkConnection();
    void publishTopicConnected(bool connected);

    // Variables
    std::string _className;
    ros::NodeHandle _nodeHandle;

    std::string _nodeName;

    int _spinRate;
    std::string _ipAddress;
    int _timeoutReconnect;

    boost::shared_ptr<RTDEInterface> _rtde;
    bool _robotConnected = false;
    double _robotTime = -1;

    ros::Publisher _topicForce;
    ros::Publisher _topicPose;
    ros::Publisher _topicStatus;
    ros::Publisher _topicTime;
    ros::Publisher _topicVelocity;
    ros::Publisher _topicConnected;
};
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespace - End
};
/////////////////////////////////////////////////////////////////////////////
*/
