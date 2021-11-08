#ifndef MOVEARM_H
#define MOVEARM_H
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
//#include "client.h"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cameraCalibration.h"
#include "camera.h"


class MoveArm
{
public:

    MoveArm();

    ~MoveArm();


    void initialize(ur_rtde::RTDEReceiveInterface& reciver, ur_rtde::RTDEControlInterface &controller);

    std::vector<double> receivePose(ur_rtde::RTDEReceiveInterface& reciver);

    std::vector<double> moveCalibrate(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity = 0.5, double acceleration = 0.5, int positionStatus = 0);

    void getToCheckerboard(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, cv::Point3f position, double velocity, double acceleration);
    void getToJob(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Point3f position, int progress, double velocity, double acceleration);


    void poseSwift(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity, double acceleration);

    std::vector<std::vector<double>> throwBall(ur_rtde::RTDEControlInterface &rtde_control, ur_rtde::RTDEReceiveInterface &reciver,  double velocity= 0.5, double acceleration = 0.5);

    std::string readVector(std::vector<double> result);

    void writeFileRobotPoses(std::vector<double> robotPoses);

    void closeURConnection(ur_rtde::RTDEControlInterface controller);

private:
    char mPoseTemp;
    bool lastPose;
    std::vector<std::vector<double>> mRobotPose;
    std::vector<std::vector<double>> mRobotjointData;

    //double mXtcp, mYtcp, mZtcp;
    //double mRXtcp, mRYtcp, mRZtcp;
    std::vector<std::string> mTempRobotPose;
    std::vector<std::vector<std::string>> mRobotPosesFromFile;
    double m2mm = 0.001;
    double deg2rad = 3.14159/180; // pi/360
    double rad2deg = 180/3.14159; // pi/360

};

#endif // MOVEARM_H
