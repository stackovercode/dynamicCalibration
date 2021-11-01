/*
/////////////////////////////////////////////////////////////////////////////
// Includes
#include "SpinURDataInterface.h"
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespaces
using namespace spin_rpi;
using namespace std;
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "spin_ur_data_interface");

    SpinURDataInterface spinUrDataInterface;

    return 0;
}
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Class functions
SpinURDataInterface::SpinURDataInterface()
{
    // Get class name
    _className = __func__;

    // Get node name
    _nodeName = ros::this_node::getName();

    // Get node parameters
    _nodeHandle.param(NODE_SPIN_ROBOT_DATA_INTERFACE + "/spin_rate", _spinRate, DEFAULT_SPIN_RATE);
    _nodeHandle.param(NODE_SPIN_ROBOT_DATA_INTERFACE + "/ip_address", _ipAddress, std::string(DEFAULT_IP_ADDRESS));
    _nodeHandle.param(NODE_SPIN_ROBOT_DATA_INTERFACE + "/timeout_reconnect", _timeoutReconnect,
                      DEFAULT_TIMEOUT_RECONNECT);

    // Advertise topics
    // TODO: Test on real robot if these publish what we expect.
    // TODO: Robot Pose/Flange, RobotTime/current time?
    _topicConnected = _nodeHandle.advertise<std_msgs::Bool>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/connected", QUEUE_SIZE,
                                                            true);
    _topicForce = _nodeHandle.advertise<spin_msgs::RobotForce>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/force", QUEUE_SIZE);
    _topicPose = _nodeHandle.advertise<spin_msgs::RobotPose>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/pose", QUEUE_SIZE);
    _topicStatus = _nodeHandle.advertise<spin_msgs::RobotStatus>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/status",
                                                                 QUEUE_SIZE);
    _topicTime = _nodeHandle.advertise<spin_msgs::RobotTime>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/time", QUEUE_SIZE);
    _topicVelocity = _nodeHandle.advertise<spin_msgs::RobotVelocity>(NODE_SPIN_ROBOT_DATA_INTERFACE + "/velocity",
                                                                     QUEUE_SIZE);

    // Start RTDE
    _rtde.reset(new RTDEInterface(_ipAddress, MOVING_SPEED_THRESHOLD));

    // Setup services
    ros::ServiceServer srvSetFreedrive = _nodeHandle.advertiseService(NODE_SPIN_ROBOT_DATA_INTERFACE + "/set_freedrive",
                                                                      &RTDEInterface::setFreedrive, _rtde);

    // Publish
    publishTopicConnected(false);

    // ROS loop
    rosLoop();
}

void SpinURDataInterface::rosLoop()
{
    // The desired rate to run at in Hz
    ros::Rate loopRate(_spinRate);

    // Loop
    while (ros::ok())
    {
        ///////////////////////////////////////////////////////////////////////
        // Check connection
        if (checkConnection())
        {
            // Just connected
            if (_robotConnected == false)
            {
                // Publish
                publishTopicConnected(true);
            }
        } else
        {
            // Just disconnected
            if (_robotConnected)
            {
                // Publish
                publishTopicConnected(false);
            }
        }
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        // Only if connected to robot
        if (_robotConnected)
            handleTopics();
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ros::spinOnce();

        // Sleep for more time if not connected
        if (_robotConnected == false)
        {
            ROS_ERROR_STREAM(_className << ": Trying to reconnect in " << _timeoutReconnect << " seconds!");
            ros::Duration(_timeoutReconnect).sleep();
        } else
        {
            loopRate.sleep();
        }
        ///////////////////////////////////////////////////////////////////////
    }
}

void SpinURDataInterface::handleTopics()
{
    if (_topicForce.getNumSubscribers())
    {
        spin_msgs::RobotForce robotForce;
        if (_rtde->getForce(robotForce))
            _topicForce.publish(robotForce);
    }

    if (_topicPose.getNumSubscribers())
    {
        spin_msgs::RobotPose robotPose;
        if (_rtde->getPose(robotPose))
            _topicPose.publish(robotPose);
    }

    if (_topicStatus.getNumSubscribers())
    {
        spin_msgs::RobotStatus robotStatus;
        if (_rtde->getStatus(robotStatus))
            _topicStatus.publish(robotStatus);
    }

    if (_topicTime.getNumSubscribers())
    {
        spin_msgs::RobotTime robotTime;
        if (_rtde->getTime(robotTime))
            _topicTime.publish(robotTime);
    }

    if (_topicVelocity.getNumSubscribers())
    {
        spin_msgs::RobotVelocity robotVelocity;
        if (_rtde->getVelocity(robotVelocity))
            _topicVelocity.publish(robotVelocity);
    }
}

bool SpinURDataInterface::checkConnection()
{
    _rtde->kickWatchdog();
    if (_rtde->isConnected())
        return true;
    else
    {
        _robotConnected = false;
        publishTopicConnected(false);
        return _rtde->connect();
    }

}

void SpinURDataInterface::publishTopicConnected(bool connected)
{
    // Publish
    std_msgs::Bool message;
    message.data = connected;
    _topicConnected.publish(message);

    // Save
    _robotConnected = connected;
}
/////////////////////////////////////////////////////////////////////////////
*/
