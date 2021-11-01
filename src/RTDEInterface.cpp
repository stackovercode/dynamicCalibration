/*
/////////////////////////////////////////////////////////////////////////////
// Includes
#include "RTDEInterface.h"
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespaces
using namespace spin_rpi;
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Class functions
RTDEInterface::RTDEInterface(std::string ip, double movingSpeedThreshold) :
        _ip(ip),
        _movingVelocityThreshold(movingSpeedThreshold)
{
    // Get class name
    _className = __func__;
}

RTDEInterface::~RTDEInterface()
{
    if (isConnected())
        _receiver->disconnect();
}

bool RTDEInterface::connect()
{
    try
    {
        if (_receiver == NULL)
        {
            ROS_INFO_STREAM("Resetting rtde");
            _receiver.reset(new ur_rtde::RTDEReceiveInterface(_ip));
            _ioControl.reset(new ur_rtde::RTDEIOInterface(_ip));
        } else
        {
            ROS_INFO_STREAM("Reconnecting rtde");
            _receiver->disconnect();
            _receiver->reconnect();
            _ioControl->reconnect();
        }
        ROS_INFO_STREAM("Done reconnecting rtde");
        ros::Duration(0.05).sleep();
        _receiver->getActualTCPPose().at(0);

        return true;
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM(_className << ": Could not connect to robot: " << e.what());
    }
    return false;
}

bool RTDEInterface::isConnected()
{
    if (_receiver == NULL)
    {
        ROS_ERROR_STREAM("Connection fail 1");
        return false;
    }
    if (_receiver->isConnected() == false)
    {
        ROS_ERROR_STREAM("Connection fail 2");
        return false;
    }
    if (_watchdog == false)
    {
        ROS_ERROR_STREAM("Connection fail 3");
        return false;
    }
    return true;
}

void RTDEInterface::kickWatchdog()
{
    if (_receiver != NULL)
    {
        try
        {
            double now = _receiver->getTimestamp();
            // if time is not updated
            if (_robotTime == now)
            {
                _watchdog = false;
            } else
            {
                _robotTime = now;
                _watchdog = true;
            }
        }
        catch (std::exception &e)
        {
            _watchdog = false;
            ROS_ERROR_STREAM(_className << ": " << e.what());
        }
    }
}

bool RTDEInterface::getTime(spin_msgs::RobotTime &robotTime)
{
    if (isConnected() == false) return false;

    robotTime.uptime.data.sec = _receiver->getTimestamp();

    return true;
}

bool RTDEInterface::getPose(spin_msgs::RobotPose &robotPose)
{
    if (isConnected() == false) return false;

    robotPose.poseBaseToTcp = getPoseMsgFromURPose(_receiver->getActualTCPPose());

    return true;
}

bool RTDEInterface::getStatus(spin_msgs::RobotStatus &robotStatus)
{
    if (isConnected() == false) return false;

    std::bitset<32> safetyStatusBits(_receiver->getSafetyStatusBits());
    std::bitset<32> statusBits(_receiver->getRobotStatus());

    robotStatus.emergencyStop = safetyStatusBits[_receiver->IS_EMERGENCY_STOPPED];
    robotStatus.securityStop = safetyStatusBits[_receiver->IS_STOPPED_DUE_TO_SAFETY];
    robotStatus.ready = safetyStatusBits[_receiver->IS_NORMAL_MODE];
    robotStatus.powerOn = statusBits[0];
    robotStatus.running = statusBits[1];
    robotStatus.freeDrive = statusBits[2];

    // Get velocity
    spin_msgs::RobotVelocity robotVelocity;
    if (getVelocity(robotVelocity) == false) return false;

    robotStatus.moving = (_movingVelocityThreshold < robotVelocity.tcpSpeed);

    return true;
}

bool RTDEInterface::getVelocity(spin_msgs::RobotVelocity &robotVelocity)
{
    if (isConnected() == false) return false;

    const std::vector<double> urVel = _receiver->getActualTCPSpeed();
    const tf2::Vector3 linearTcpVelocityInBaseFrame = {urVel.at(0), urVel.at(1), urVel.at(2)};
    const tf2::Vector3 angularTcpVelocityInBaseFrame = {urVel.at(3), urVel.at(4), urVel.at(5)};
    const tf2::Vector3 linearTcpVelocityInTcpFrame = rotateVectorFromBaseToTcp(linearTcpVelocityInBaseFrame);

    robotVelocity.tcpVelocityInBaseFrame.linear = tf2::toMsg(linearTcpVelocityInBaseFrame);
    robotVelocity.tcpVelocityInBaseFrame.angular = tf2::toMsg(angularTcpVelocityInBaseFrame);
    robotVelocity.tcpVelocityInToolFrame = tf2::toMsg(linearTcpVelocityInTcpFrame);
    robotVelocity.tcpSpeed = linearTcpVelocityInBaseFrame.length();

    return true;
}

bool RTDEInterface::getForce(spin_msgs::RobotForce &robotForce)
{
    if (isConnected() == false) return false;

    const std::vector<double> urForce = _receiver->getActualTCPForce();
    const tf2::Vector3 tcpForceInBaseFrame = {urForce.at(0), urForce.at(1), urForce.at(2)};
    const tf2::Vector3 tcpForceInToolFrame = rotateVectorFromBaseToTcp(tcpForceInBaseFrame);

    robotForce.tcpForceInToolFrame = tf2::toMsg(tcpForceInToolFrame);
    robotForce.tcpForce = tcpForceInToolFrame.length();

    return true;
}

bool RTDEInterface::setFreedrive(std_srvs::SetBool::Request &req, std_srvs::SetBoolResponse &resp)
{
    if (isConnected() == false)
    {
        resp.success = false;
        resp.message = "ERROR: Not connected to the robot!";
        return false;
    }

    spin_msgs::RobotStatus robotStatus;

    if (req.data)
    {
        // Get status
        if (getStatus(robotStatus) == false) return false;

        if (
                robotStatus.running == false &&
                robotStatus.powerOn &&
                robotStatus.ready
                )
        {
            // Set freedrive
            _ioControl->setInputBitRegister64(true);

            // Wait to check if success
            ros::Duration(0.01).sleep();

            // Get status
            if (getStatus(robotStatus) == false) return false;

            // Check
            if (robotStatus.freeDrive)
            {
                // Return
                resp.success = true;
                resp.message = "Robot in freedrive mode.";
                return true;
            } else
            {
                // Clear again
                _ioControl->setInputBitRegister64(false);

                // Return
                resp.success = false;
                resp.message = "ERROR: Robot could not be set into freedrive mode.";
                return false;
            }
        } else
        {
            // Return
            resp.success = false;
            resp.message = "ERROR: Cannot be set in freedrive in this mode!";
            return false;
        }
    } else
    {
        // End freedrive
        _ioControl->setInputBitRegister64(false);

        // Wait to check if success
        ros::Duration(0.01).sleep();

        // Get status
        if (getStatus(robotStatus) == false) return false;

        // Check
        if (robotStatus.freeDrive == false)
        {
            // Return
            resp.success = true;
            resp.message = "Robot not in freedrive mode anymore.";
            return true;
        } else
        {
            // Return
            resp.success = false;
            resp.message = "ERROR: Robot may still be in freedrive mode.";
            return false;
        }
    }
}

geometry_msgs::Pose RTDEInterface::getPoseMsgFromURPose(const std::vector<double> &urPose)
{
    geometry_msgs::Pose result;
    result.position.x = urPose.at(0);
    result.position.y = urPose.at(1);
    result.position.z = urPose.at(2);

    tf2::Quaternion q;
    tf2::Vector3 urAxisAngle(urPose.at(3), urPose.at(4), urPose.at(5));
    q.setRotation(urAxisAngle.normalize(), urAxisAngle.length());
    result.orientation = tf2::toMsg(q);

    return result;
}

tf2::Vector3 RTDEInterface::rotateVectorFromBaseToTcp(const tf2::Vector3 &vec)
{
    // Get velocity
    spin_msgs::RobotPose robotPose;
    getPose(robotPose);

    // Get the tcp to base rotation from the actual tcp orientation
    tf2::Quaternion tcpToBase;
    tf2::fromMsg(robotPose.poseBaseToTcp.orientation, tcpToBase);

    // Invert it to base to tcp rotation
    tf2::Quaternion baseToTcp = tf2::inverse(tcpToBase);

    // Rotate the linear velocity vector from base to tcp frame
    return tf2::quatRotate(baseToTcp, vec);
}
/////////////////////////////////////////////////////////////////////////////
*/
