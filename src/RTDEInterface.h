/*
#pragma once

/////////////////////////////////////////////////////////////////////////////
// Includes

// ROS
//#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

// RTDE
#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_io_interface.h"

// ROS Messages
#include "spin_msgs/RobotStatus.h"
#include "spin_msgs/RobotVelocity.h"
#include "spin_msgs/RobotPose.h"
#include "spin_msgs/RobotTime.h"
#include "spin_msgs/RobotForce.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Utilities
#include <bitset>
#include <chrono>
#include <thread>
#include <utility>
#include <boost/shared_ptr.hpp>
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespace
namespace spin_rpi
{
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Class
    class RTDEInterface
    {

    public:
        // Constructors
        RTDEInterface(std::string ip, double movingSpeedThreshold);

        ~RTDEInterface();

        // Functions
        bool isConnected();

        // Must be called only once every loop cycle.
        void kickWatchdog();

        bool connect();

        bool getStatus(spin_msgs::RobotStatus &robotStatus);

        bool getVelocity(spin_msgs::RobotVelocity &robotVelocity);

        bool getPose(spin_msgs::RobotPose &robotPose);

        bool getForce(spin_msgs::RobotForce &robotForce);

        bool
        getTime(spin_msgs::RobotTime &robotTime); // Returns robot uptime (seconds from last time the controller was restarted)

        bool setFreedrive(std_srvs::SetBool::Request &req, std_srvs::SetBoolResponse &resp);

    private:
        // Functions
        geometry_msgs::Pose getPoseMsgFromURPose(const std::vector<double> &urPose);

        tf2::Vector3 rotateVectorFromBaseToTcp(const tf2::Vector3 &vec);

        // Variables
        std::string _className;

        std::string _ip;
        double _movingVelocityThreshold;
        double _robotTime = -1.0;
        bool _watchdog = false;

        boost::shared_ptr<ur_rtde::RTDEReceiveInterface> _receiver;
        boost::shared_ptr<ur_rtde::RTDEIOInterface> _ioControl;
    };
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Namespace - End
};
/////////////////////////////////////////////////////////////////////////////
*/
