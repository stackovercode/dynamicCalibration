#include "moveRobot.h"
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>
#include <iostream>
#include <bits/stdc++.h>
#include <algorithm>

using namespace ur_rtde;
using namespace std;

MoveRobot::MoveRobot(){

}

MoveRobot::~MoveRobot(){}

bool MoveRobot::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller){
    std::vector<std::vector<double>> path_q;
    double velocity = 0.15, acceleration = 0.15;
    std::vector<double> startPos_q1 = {-0.896468,-1.4284,-1.2074,-3.65303,-1.2375,1.57324, velocity, acceleration, 0.0};
    path_q.push_back(startPos_q1);
    controller.moveJ(path_q, false);
    return true;
}

std::vector<double> MoveRobot::getToCheckerboard(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, int type, cv::Vec6d position, double velocity, double acceleration){
    std::cout << "Moving to checker postion" << std::endl;
    ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
    std::vector<double> baseFrame = receiverNew.getTargetTCPPose();
    std::vector<double> jointBaseFrame = receiverNew.getTargetQ();
    std::cout << "Move frame: " << readVector(baseFrame) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<double> featureFrame = {position[0],position[1],position[2],position[3],position[4],position[5]};
    std::vector<double> moveFrame = controller.poseTrans(baseFrame,featureFrame);
    std::cout << "Move frame: " << readVector(moveFrame) << std::endl;
    controller.moveL({moveFrame}, velocity, acceleration);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    controller.stopL();
    return jointBaseFrame;
}
std::vector<double> MoveRobot::getLiveEstimation(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, int type, cv::Vec6d position, double velocity, double acceleration){
    std::cout << "Moving to checker postion" << std::endl;
    ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
    std::vector<double> baseFrame = receiverNew.getTargetTCPPose();
    std::vector<double> jointBaseFrame = receiverNew.getTargetQ();
    std::cout << "Move frame: " << readVector(baseFrame) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<double> featureFrame = {position[0],position[1],position[2],position[3],position[4],position[5]};
    std::vector<double> moveFrame = controller.poseTrans(baseFrame,featureFrame);
    std::cout << "Move frame: " << readVector(moveFrame) << std::endl;
    controller.moveL({moveFrame}, velocity, acceleration);
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    controller.stopL();
    return jointBaseFrame;
}

std::vector<double> MoveRobot::getToPoseEstimation(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, int type, cv::Vec6d position, double velocity, double acceleration){
    std::cout << "Moving to checker postion" << std::endl;
    ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
    std::vector<double> baseFrame = receiverNew.getTargetTCPPose();
    std::cout << "Move frame: " << readVector(baseFrame) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<double> featureFrame = {position[0],position[1],position[2],baseFrame[3],baseFrame[4],baseFrame[5]};
    std::vector<double> featureFrame2 = {0,0,0,position[3],position[4],position[5]};
    std::vector<double> moveFrame = controller.poseTrans(baseFrame,featureFrame);
    //std::cout << "Move frame: " << readVector(moveFrame) << std::endl;
    featureFrame = {position[0],position[1],position[2],moveFrame[3],moveFrame[4],moveFrame[5]};
    //std::vector<double> featureFrame = {position[0],position[1],position[2],position[3],position[4],position[5]};
    controller.moveL({featureFrame}, velocity, acceleration);
    //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    controller.moveL({baseFrame}, velocity, acceleration);
    controller.stopL();
    return featureFrame;
}


void MoveRobot::getToJob(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6d position, std::vector<double> baseFrame, int progress, double velocity, double acceleration){
    if (progress < 4) {
        std::cout << "Moving to job postion: " << progress << std::endl;
    }
    std::vector<double> targetJobPose1 = {0.0658822,-0.363385,-0.0169811,2.56832,-1.80919,-0.0107452};
    std::vector<double> targetJobPose2 = {0.157756,-0.326576,-0.0173523,2.56832,-1.80919,-0.0107452};
    std::vector<double> targetJobPose3 = {0.250143,-0.287226,-0.0177096,2.56832,-1.80919,-0.0107452};
    //std::vector<double> baseFrame = {-0.0423311,-0.38472,0.274039,2.56832,-1.80919,-0.0107453};
    std::vector<double> featureFrame = {position[0],position[1],position[2],position[3],position[4],position[5]};
    std::vector<double> startFrame;
    std::vector<double> endFrame;
    std::vector<double> targetEndFrame;
    //WorkspaceCalibration p;
    if (progress >= 4) {
        std::cout << "Done" << std::endl;
    } else {
        switch (progress) {
        case 0:
            std::cout << "Inside case 0. start job." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            startFrame = controller.poseTrans(baseFrame,featureFrame);
            std::cout << "Featureframe: " << readVector(featureFrame) << std::endl;
            targetEndFrame = targetPointTransform(startFrame, targetJobPose1, position[5]);
            endFrame = controller.poseTrans(startFrame, targetEndFrame);
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({startFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({endFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        case 1:
            std::cout << "Inside case 1. start job." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            startFrame = controller.poseTrans(baseFrame,featureFrame);
            std::cout << "Featureframe: " << readVector(featureFrame) << std::endl;
            targetEndFrame = targetPointTransform(startFrame, targetJobPose1, position[5]);
            endFrame = controller.poseTrans(startFrame, targetEndFrame);
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({startFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({endFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        case 2:
            std::cout << "Inside case 2. start job." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            startFrame = controller.poseTrans(baseFrame,featureFrame);
            std::cout << "Featureframe: " << readVector(featureFrame) << std::endl;
            targetEndFrame = targetPointTransform(startFrame, targetJobPose2, position[5]);
            endFrame = controller.poseTrans(startFrame, targetEndFrame);
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({startFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({endFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        case 3:
            std::cout << "Inside case 3. start job." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            startFrame = controller.poseTrans(baseFrame,featureFrame);
            std::cout << "Featureframe: " << readVector(featureFrame) << std::endl;
            targetEndFrame = targetPointTransform(startFrame, targetJobPose3, position[5]);
            endFrame = controller.poseTrans(startFrame, targetEndFrame);
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({startFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({endFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            controller.moveL({baseFrame}, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        default:
            std::cout << "Error: Went to default in switch statement" << std::endl;
            break;
        }

    }

}

std::vector<double> MoveRobot::receivePose(ur_rtde::RTDEReceiveInterface &reciver){
    std::vector<double> jointPose = reciver.getActualQ();
    return jointPose;
}

cv::Vec6f MoveRobot::receiveJPose(ur_rtde::RTDEReceiveInterface &reciver){
    std::vector<double> jointPose = reciver.getActualQ();
    cv::Vec6f jPose;
    for (size_t i = 0; i < jointPose.size(); i++) {
        jPose[i] = jointPose[i];
    }
    return jPose;
}

std::vector<double> MoveRobot::poseSwift(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity, double acceleration, int positionStatus, std::vector<double> initPose, int mNumberOfCalibrationImages, bool largeCheckerboardSize){
    std::cout << "Moving to checker postion" << std::endl;

    if(positionStatus == 1){// && !largeCheckerboardSize){
    std::vector<std::vector<double>> path_q;
    std::vector<double> startPos_q1 = {-0.899286,-1.30822,-1.8855,-3.09498,-1.23777,1.57271, velocity, acceleration, 0.0};
    //std::vector<double> startPos_q1 = {-0.912752,-1.20274,-1.76052,-0.133582,1.31342,-1.5769, velocity, acceleration, 0.0};
    //std::vector<double> startPos_q1 = {-0.906876,-1.28316,-1.37612,-0.437507,1.30899,-1.57704, velocity, acceleration, 0.0};
    path_q.push_back(startPos_q1);
    controller.moveJ(path_q, false);
    writeFileRobotTCPPosesFlagChoice = true;
    }

    ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
    std::vector<double> pose = receiverNew.getActualTCPPose();
    double temp = pose[2];
    pose[2] = 0.453179;

    mRobotTCPPoses.push_back(pose);
    writeFileRobotTCPPoses(writeFileRobotTCPPosesFlagChoice, mRobotTCPPoses);
    pose[2] = temp;

    //std::vector<double> currentPose = {0.0146505+0.001518,0.0136401+0.004175,0.0,0.0,0.0,0.0};
    std::vector<double> currentPose = {0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<double> change_0_0 = controller.poseTrans(pose,currentPose);

    if (positionStatus == 1) {
        initPose = change_0_0;
    }

    std::vector<double> change_1_1 = {0.0,0.10,0.0,0.35,0.0,0.0};
    std::vector<double> change_1_2 = {0.0,0.05,0.0,0.175,0.0,0.0};
    std::vector<double> change_1_3 = {0.0,-0.05,0.0,-0.175,0.0,0.0};
    std::vector<double> change_1_4 = {0.0,-0.10,0.0,-0.35,0.0,0.0};
    std::vector<double> pose_1_1 = controller.poseTrans(initPose,change_1_1);
    std::vector<double> pose_1_2 = controller.poseTrans(initPose,change_1_2);
    std::vector<double> pose_1_3 = controller.poseTrans(initPose,change_1_3);
    std::vector<double> pose_1_4 = controller.poseTrans(initPose,change_1_4);

    std::vector<double> change_9_1 = {0.025,0.10,0.0,0.35,-0.087,0.0};
    std::vector<double> change_9_2 = {0.0125,0.05,0.0,0.175,-0.044,0.0};
    std::vector<double> change_9_3 = {-0.0125,-0.05,0.0,-0.175,0.044,0.0};
    std::vector<double> change_9_4 = {-0.025,-0.10,0.0,-0.35,0.087,0.0};
    std::vector<double> pose_9_1 = controller.poseTrans(initPose,change_9_1);
    std::vector<double> pose_9_2 = controller.poseTrans(initPose,change_9_2);
    std::vector<double> pose_9_3 = controller.poseTrans(initPose,change_9_3);
    std::vector<double> pose_9_4 = controller.poseTrans(initPose,change_9_4);

    std::vector<double> change_2_1 = {0.05,0.10,0.0,0.35,-0.175,0.0};
    std::vector<double> change_2_2 = {0.025,0.05,0.0,0.175,-0.087,0.0};
    std::vector<double> change_2_3 = {-0.025,-0.05,0.0,-0.175,0.087,0.0};
    std::vector<double> change_2_4 = {-0.05,-0.10,0.0,-0.35,0.175,0.0};
    std::vector<double> pose_2_1 = controller.poseTrans(initPose,change_2_1);
    std::vector<double> pose_2_2 = controller.poseTrans(initPose,change_2_2);
    std::vector<double> pose_2_3 = controller.poseTrans(initPose,change_2_3);
    std::vector<double> pose_2_4 = controller.poseTrans(initPose,change_2_4);

    std::vector<double> change_10_1 = {0.075,0.10,0.0,0.35,-0.26,0.0}; // Change to -0.26 from -0.13
    std::vector<double> change_10_2 = {0.0375,0.05,0.0,0.175,-0.13,0.0}; // Change to -0.13 from -0.065
    std::vector<double> change_10_3 = {-0.0375,-0.05,0.0,-0.175,0.13,0.0}; // Change to 0.13 from 0.065
    std::vector<double> change_10_4 = {-0.075,-0.10,0.0,-0.35,0.26,0.0}; // Change to 0.26 from 0.13
    std::vector<double> pose_10_1 = controller.poseTrans(initPose,change_10_1);
    std::vector<double> pose_10_2 = controller.poseTrans(initPose,change_10_2);
    std::vector<double> pose_10_3 = controller.poseTrans(initPose,change_10_3);
    std::vector<double> pose_10_4 = controller.poseTrans(initPose,change_10_4);

    std::vector<double> change_7_1 = {0.10,0.10,0.0,0.35,-0.35,0.0};
    std::vector<double> change_7_2 = {0.05,0.05,0.0,0.175,-0.175,0.0};
    std::vector<double> change_7_3 = {-0.05,-0.05,0.0,-0.175,0.175,0.0};
    std::vector<double> change_7_4 = {-0.10,-0.10,0.0,-0.35,0.35,0.0};
    std::vector<double> pose_7_1 = controller.poseTrans(initPose,change_7_1);
    std::vector<double> pose_7_2 = controller.poseTrans(initPose,change_7_2);
    std::vector<double> pose_7_3 = controller.poseTrans(initPose,change_7_3);
    std::vector<double> pose_7_4 = controller.poseTrans(initPose,change_7_4);

    std::vector<double> change_11_1 = {0.10,0.075,0.0,0.26,-0.35,0.0};
    std::vector<double> change_11_2 = {0.05,0.0375,0.0,0.13,-0.175,0.0};
    std::vector<double> change_11_3 = {-0.05,-0.0375,0.0,-0.13,0.175,0.0};
    std::vector<double> change_11_4 = {-0.10,-0.075,0.0,-0.26,0.35,0.0};
    std::vector<double> pose_11_1 = controller.poseTrans(initPose,change_11_1);
    std::vector<double> pose_11_2 = controller.poseTrans(initPose,change_11_2);
    std::vector<double> pose_11_3 = controller.poseTrans(initPose,change_11_3);
    std::vector<double> pose_11_4 = controller.poseTrans(initPose,change_11_4);

    std::vector<double> change_3_1 = {0.10,0.05,0.0,0.175,-0.35,0.0};
    std::vector<double> change_3_2 = {0.05,0.025,0.0,0.087,-0.175,0.0};
    std::vector<double> change_3_3 = {-0.05,-0.025,0.0,-0.087,0.175,0.0};
    std::vector<double> change_3_4 = {-0.10,-0.05,0.0,-0.175,0.35,0.0};
    std::vector<double> pose_3_1 = controller.poseTrans(initPose,change_3_1);
    std::vector<double> pose_3_2 = controller.poseTrans(initPose,change_3_2);
    std::vector<double> pose_3_3 = controller.poseTrans(initPose,change_3_3);
    std::vector<double> pose_3_4 = controller.poseTrans(initPose,change_3_4);

    std::vector<double> change_12_1 = {0.10,0.025,0.0,0.087,-0.35,0.0};
    std::vector<double> change_12_2 = {0.05,0.0125,0.0,0.044,-0.175,0.0};
    std::vector<double> change_12_3 = {-0.05,-0.0125,0.0,-0.044,0.175,0.0};
    std::vector<double> change_12_4 = {-0.10,-0.025,0.0,-0.087,0.35,0.0};
    std::vector<double> pose_12_1 = controller.poseTrans(initPose,change_12_1);
    std::vector<double> pose_12_2 = controller.poseTrans(initPose,change_12_2);
    std::vector<double> pose_12_3 = controller.poseTrans(initPose,change_12_3);
    std::vector<double> pose_12_4 = controller.poseTrans(initPose,change_12_4);

    std::vector<double> change_4_1 = {0.10,0.0,0.0,0.0,-0.35,0.0};
    std::vector<double> change_4_2 = {0.05,0.0,0.0,0.0,-0.175,0.0};
    std::vector<double> change_4_3 = {-0.05,0.0,0.0,0.0,0.175,0.0};
    std::vector<double> change_4_4 = {-0.10,0.0,0.0,0.0,0.35,0.0};
    std::vector<double> pose_4_1 = controller.poseTrans(initPose,change_4_1);
    std::vector<double> pose_4_2 = controller.poseTrans(initPose,change_4_2);
    std::vector<double> pose_4_3 = controller.poseTrans(initPose,change_4_3);
    std::vector<double> pose_4_4 = controller.poseTrans(initPose,change_4_4);

    std::vector<double> change_13_1 = {0.10,-0.025,0.0,-0.087,-0.35,0.0};
    std::vector<double> change_13_2 = {0.05,-0.0125,0.0,-0.044,-0.175,0.0};
    std::vector<double> change_13_3 = {-0.05,0.0125,0.0,0.044,0.175,0.0};
    std::vector<double> change_13_4 = {-0.10,0.025,0.0,0.087,0.35,0.0};
    std::vector<double> pose_13_1 = controller.poseTrans(initPose,change_13_1);
    std::vector<double> pose_13_2 = controller.poseTrans(initPose,change_13_2);
    std::vector<double> pose_13_3 = controller.poseTrans(initPose,change_13_3);
    std::vector<double> pose_13_4 = controller.poseTrans(initPose,change_13_4);


    std::vector<double> change_5_1 = {0.10,-0.05,0.0,-0.175,-0.35,0.0};
    std::vector<double> change_5_2 = {0.05,-0.025,0.0,-0.087,-0.175,0.0};
    std::vector<double> change_5_3 = {-0.05,0.025,0.0,0.087,0.175,0.0};
    std::vector<double> change_5_4 = {-0.10,0.05,0.0,0.175,0.35,0.0};
    std::vector<double> pose_5_1 = controller.poseTrans(initPose,change_5_1);
    std::vector<double> pose_5_2 = controller.poseTrans(initPose,change_5_2);
    std::vector<double> pose_5_3 = controller.poseTrans(initPose,change_5_3);
    std::vector<double> pose_5_4 = controller.poseTrans(initPose,change_5_4);

    std::vector<double> change_14_1 = {0.10,-0.075,0.0,-0.26,-0.35,0.0};
    std::vector<double> change_14_2 = {0.05,-0.0375,0.0,-0.13,-0.175,0.0};
    std::vector<double> change_14_3 = {-0.05,0.0375,0.0,0.13,0.175,0.0};
    std::vector<double> change_14_4 = {-0.10,0.075,0.0,0.26,0.35,0.0};
    std::vector<double> pose_14_1 = controller.poseTrans(initPose,change_14_1);
    std::vector<double> pose_14_2 = controller.poseTrans(initPose,change_14_2);
    std::vector<double> pose_14_3 = controller.poseTrans(initPose,change_14_3);
    std::vector<double> pose_14_4 = controller.poseTrans(initPose,change_14_4);


    std::vector<double> change_8_1 = {0.10,-0.10,0.0,-0.35,-0.35,0.0};
    std::vector<double> change_8_2 = {0.05,-0.05,0.0,-0.175,-0.175,0.0};
    std::vector<double> change_8_3 = {-0.05,0.05,0.0,0.175,0.175,0.0};
    std::vector<double> change_8_4 = {-0.10,0.10,0.0,0.35,0.35,0.0};
    std::vector<double> pose_8_1 = controller.poseTrans(initPose,change_8_1);
    std::vector<double> pose_8_2 = controller.poseTrans(initPose,change_8_2);
    std::vector<double> pose_8_3 = controller.poseTrans(initPose,change_8_3);
    std::vector<double> pose_8_4 = controller.poseTrans(initPose,change_8_4);

    std::vector<double> change_15_1 = {0.075,-0.10,0.0,-0.35,-0.26,0.0};
    std::vector<double> change_15_2 = {0.0375,-0.05,0.0,-0.175,-0.13,0.0};
    std::vector<double> change_15_3 = {-0.0375,0.05,0.0,0.175,0.13,0.0};
    std::vector<double> change_15_4 = {-0.075,0.10,0.0,0.35,0.26,0.0};
    std::vector<double> pose_15_1 = controller.poseTrans(initPose,change_15_1);
    std::vector<double> pose_15_2 = controller.poseTrans(initPose,change_15_2);
    std::vector<double> pose_15_3 = controller.poseTrans(initPose,change_15_3);
    std::vector<double> pose_15_4 = controller.poseTrans(initPose,change_15_4);


    std::vector<double> change_6_1 = {0.05,-0.10,0.0,-0.35,-0.175,0.0};
    std::vector<double> change_6_2 = {0.025,-0.05,0.0,-0.175,-0.087,0.0};
    std::vector<double> change_6_3 = {-0.025,0.05,0.0,0.175,0.087,0.0};
    std::vector<double> change_6_4 = {-0.05,0.10,0.0,0.35,0.175,0.0};
    std::vector<double> pose_6_1 = controller.poseTrans(initPose,change_6_1);
    std::vector<double> pose_6_2 = controller.poseTrans(initPose,change_6_2);
    std::vector<double> pose_6_3 = controller.poseTrans(initPose,change_6_3);
    std::vector<double> pose_6_4 = controller.poseTrans(initPose,change_6_4);

    std::vector<double> change_16_1 = {0.025,-0.10,0.0,-0.35,-0.087,0.0};
    std::vector<double> change_16_2 = {0.0125,-0.05,0.0,-0.175,-0.044,0.0};
    std::vector<double> change_16_3 = {-0.0125,0.05,0.0,0.175,0.044,0.0};
    std::vector<double> change_16_4 = {-0.025,0.10,0.0,0.35,0.087,0.0};
    std::vector<double> pose_16_1 = controller.poseTrans(initPose,change_16_1);
    std::vector<double> pose_16_2 = controller.poseTrans(initPose,change_16_2);
    std::vector<double> pose_16_3 = controller.poseTrans(initPose,change_16_3);
    std::vector<double> pose_16_4 = controller.poseTrans(initPose,change_16_4);


    switch (positionStatus) {
    case 1:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_1_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (mNumberOfCalibrationImages == 5) {
                //controller.moveL(pose_1_1, velocity, acceleration);
                controller.moveL(pose_1_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                if (mNumberOfCalibrationImages == 1) {
                    controller.moveL(initPose, velocity, acceleration);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }else{
                    controller.moveL(pose_1_2, velocity, acceleration);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        break;
    case 2:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_7_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (mNumberOfCalibrationImages == 5) {
                //controller.moveL(pose_4_1, velocity, acceleration);
                controller.moveL(pose_4_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                controller.moveL(pose_9_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        break;
    case 3:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_4_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (mNumberOfCalibrationImages == 5) {
                //controller.moveL(pose_1_4, velocity, acceleration);
                controller.moveL(pose_1_3, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                controller.moveL(pose_2_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        break;
    case 4:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_8_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (mNumberOfCalibrationImages == 5) {
                //controller.moveL(pose_4_4, velocity, acceleration);
                controller.moveL(pose_4_3, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                controller.moveL(pose_10_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        break;
    case 5:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_1_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (mNumberOfCalibrationImages == 5) {
                controller.moveL(initPose, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                controller.moveL(pose_7_2, velocity, acceleration);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        break;
    case 6:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_7_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_11_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 7:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_4_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_3_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 8:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_8_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_12_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 9:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_8_4, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_4_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 10:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_1_1, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_13_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 11:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_7_1, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_5_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 12:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_4_1, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_14_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 13:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_8_1, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_8_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 14:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_1_4, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_15_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 15:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_7_4, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_6_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 16:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(pose_4_4, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_16_2, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 17:
        if (mNumberOfCalibrationImages == 17) {
            controller.moveL(initPose, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            controller.moveL(pose_1_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 18:
        controller.moveL(pose_9_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 19:
        controller.moveL(pose_2_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 20:
        controller.moveL(pose_10_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 21:
        controller.moveL(pose_7_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 22:
        controller.moveL(pose_11_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 23:
        controller.moveL(pose_3_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 24:
        controller.moveL(pose_12_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 25:
        if(mNumberOfCalibrationImages == 25){
            controller.moveL(initPose, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }else{
            controller.moveL(pose_4_3, velocity, acceleration);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;
    case 26:
        controller.moveL(pose_13_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 27:
        controller.moveL(pose_5_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 28:
        controller.moveL(pose_14_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 29:
        controller.moveL(pose_8_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 30:
        controller.moveL(pose_15_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 31:
        controller.moveL(pose_6_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 32:
        controller.moveL(pose_16_3, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 33:
        controller.moveL(pose_16_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 34:
        controller.moveL(pose_1_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;

    case 35:
        controller.moveL(pose_9_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;

    case 36:
        controller.moveL(pose_2_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 37:
        controller.moveL(pose_10_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 38:
        controller.moveL(pose_7_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 39:
        controller.moveL(pose_11_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 40:
        controller.moveL(pose_3_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 41:
        controller.moveL(pose_12_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 42:
        controller.moveL(pose_4_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 43:
        controller.moveL(pose_13_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 44:
        controller.moveL(pose_5_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 45:
        controller.moveL(pose_14_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 46:
        controller.moveL(pose_8_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 47:
        controller.moveL(pose_15_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 48:
        controller.moveL(pose_6_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 49:
        controller.moveL(pose_16_1, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 50:
        controller.moveL(pose_1_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 51:
        controller.moveL(pose_9_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 52:
        controller.moveL(pose_2_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 53:
        controller.moveL(pose_10_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 54:
        controller.moveL(pose_7_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 55:
        controller.moveL(pose_11_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 56:
        controller.moveL(pose_3_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 57:
        controller.moveL(pose_12_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 58:
        controller.moveL(pose_4_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 59:
        controller.moveL(pose_13_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 60:
        controller.moveL(pose_5_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 61:
        controller.moveL(pose_14_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 62:
        controller.moveL(pose_8_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 63:
        controller.moveL(pose_15_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 64:
        controller.moveL(pose_6_4, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    case 65:
        controller.moveL(initPose, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    default:
        controller.moveL(initPose, velocity, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Error: Went to default in poseswift" << std::endl;
        break;
    }

    return initPose;

}

std::vector<double> MoveRobot::moveCalibrate(RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity, double acceleration, int positionStatus)
{

    std::cout << "Moving to checker postion" << std::endl;
    double blend = 0.0;

    for (int i = positionStatus; i <= positionStatus+1; i++) {
        std::vector<std::vector<double>> path_q;
        std::vector<std::vector<double>> tempPath_q;

        //Dataset 2
        std::vector<double> startPos_q1 = {-1.36689,-1.28005,-1.90308,-3.10318,-1.70538,1.5708, velocity, acceleration, blend};
        path_q.push_back(startPos_q1);
        std::vector<double> startPos_q2 = {-1.36736,-1.20107,-1.98242,-3.01794,-1.70544,1.57076, velocity, acceleration, blend};
        path_q.push_back(startPos_q2);
        std::vector<double> startPos_q3 = {-1.36748,-1.02789,-2.10625,-2.95188,-1.70543,1.57074, velocity, acceleration, blend};
        path_q.push_back(startPos_q3);
        std::vector<double> startPos_q4 = {-1.36685,-1.51952,-1.68411,-3.22387,-1.70548,1.57073, velocity, acceleration, blend};
        path_q.push_back(startPos_q4);
        std::vector<double> startPos_q5 = {-1.36624,-1.66543,-1.52339,-3.34693,-1.70547,1.57084, velocity, acceleration, blend};
        path_q.push_back(startPos_q5);
        std::vector<double> startPos_q6 = {-1.58481,-1.48048,-1.75088,-3.06395,-1.90983,1.75687, velocity, acceleration, blend};
        path_q.push_back(startPos_q6);
        std::vector<double> startPos_q7 = {-1.72776,-1.57459,-1.67291,-3.03949,-2.0756,1.82734, velocity, acceleration, blend};
        path_q.push_back(startPos_q7);
        std::vector<double> startPos_q8 = {-1.14939,-1.23728,-1.94058,-3.10791,-1.52694,1.44021, velocity, acceleration, blend};
        path_q.push_back(startPos_q8);
        std::vector<double> startPos_q9 = {-1.05218,-1.21415,-1.95861,-3.11494,-1.52624,1.38868, velocity, acceleration, blend};
        path_q.push_back(startPos_q9);
        std::vector<double> startPos_q10 = {-1.61608,-1.28124,-1.94645,-2.91638,-1.98955,1.70305, velocity, acceleration, blend};
        path_q.push_back(startPos_q10);
        std::vector<double> startPos_q11 = {-1.7892,-1.26425,-1.96668,-2.88363,-2.13456,1.79808, velocity, acceleration, blend};
        path_q.push_back(startPos_q11);
        std::vector<double> startPos_q12 = {-1.89567,-1.39632,-1.87204,-2.88146,-2.1338,1.92503, velocity, acceleration, blend};
        path_q.push_back(startPos_q12);
        std::vector<double> startPos_q13 = {-1.77157,-1.43253,-1.83683,-2.91425,-2.06406,1.92522, velocity, acceleration, blend};
        path_q.push_back(startPos_q13);
        std::vector<double> startPos_q14 = {-1.54479,-1.67822,-1.53244,-3.21055,-1.79602,1.76034, velocity, acceleration, blend};
        path_q.push_back(startPos_q14);
        std::vector<double> startPos_q15 = {-1.59852,-1.81873,-1.36047,-3.27728,-1.77842,1.76018, velocity, acceleration, blend};
        path_q.push_back(startPos_q15);
        std::vector<double> startPos_q16 = {-1.51729,-1.80706,-1.36871,-3.2634,-1.73008,1.65627, velocity, acceleration, blend};
        path_q.push_back(startPos_q16);
        std::vector<double> startPos_q17 = {-1.53057,-1.70923,-1.48565,-3.25013,-1.73047,1.65628, velocity, acceleration, blend};
        path_q.push_back(startPos_q17);
        std::vector<double> startPos_q18 = {-1.22665,-1.52012,-1.68223,-3.26594,-1.70628,1.4149, velocity, acceleration, blend};
        path_q.push_back(startPos_q18);
        std::vector<double> startPos_q19 = {-1.13722,-1.58361,-1.61766,-3.28707,-1.70608,1.41497, velocity, acceleration, blend};
        path_q.push_back(startPos_q19);
        std::vector<double> startPos_q20 = {-1.08198,-1.46827,-1.73255,-3.24727,-1.7059,1.41472, velocity, acceleration, blend};
        path_q.push_back(startPos_q20);
        std::vector<double> startPos_q21 = {-1.1731,-1.46489,-1.74459,-3.1827,-1.70584,1.41476, velocity, acceleration, blend};
        path_q.push_back(startPos_q21);
        std::vector<double> startPos_q22 = {-1.15132,-1.06494,-2.06857,-3.04253,-1.5263,1.43971, velocity, acceleration, blend};
        path_q.push_back(startPos_q22);
        std::vector<double> startPos_q23 = {-1.07175,-0.968296,-2.11721,-3.06948,-1.5267,1.43971, velocity, acceleration, blend};
        path_q.push_back(startPos_q23);
        std::vector<double> startPos_q24 = {-1.24796,-0.951413,-2.15698,-2.94865,-1.56068,1.52204, velocity, acceleration, blend};
        path_q.push_back(startPos_q24);
        std::vector<double> startPos_q25 = {-1.24799,-1.12373,-2.04463,-2.98072,-1.56071,1.52225, velocity, acceleration, blend};
        path_q.push_back(startPos_q25);
        std::vector<double> startPos_q26 = {-1.36689,-1.28005,-1.90308,-3.10318,-1.70538,1.5708, velocity, acceleration, blend};
        //std::vector<double> newStartPos_q26 = {-78.1*deg2rad, -97.3*deg2rad, -26.1*deg2rad, -236.8*deg2rad, -97.8*deg2rad, 90*deg2rad, velocity, acceleration, blend};
        //std::vector<double> newStartPos_q26 = {-1.3631, -1.6982, -0.4555, -4.1329, -1.7069, -1.5707, velocity, acceleration, blend};
        path_q.push_back(startPos_q26);
        //path_q.push_back(newStartPos_q26);


        tempPath_q.push_back(path_q[i-1]);

        if (i == positionStatus+1) {
            controller.moveJ(tempPath_q, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            controller.stopJ(0.2);
            ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
            //receivePose(receiverNew);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::vector<double> pose = receiverNew.getActualTCPPose();
            std::vector<double> jointPose = receiverNew.getActualQ();
            for (int i = 0; i < 3; i++) {
                pose[i] = pose[i]/m2mm;
            }

            writeFileRobotPoses(jointPose);

            return pose;
            break;
        }
    }

}


std::string MoveRobot::readVector(std::vector<double> result)
{
    std::ostringstream vts;
    if (!result.empty()){
    std::copy(result.begin(), result.end()-1,
        std::ostream_iterator<double>(vts, ","));
        vts << result.back();
    }
    return vts.str();
}



void MoveRobot::writeFileRobotPoses(std::vector<double> robotPoses){
    std::ofstream myfile;
    myfile.open ("../Detection/RobotjointData.txt", std::ios::app);
        myfile << readVector(robotPoses) << ";" << std::endl;
    myfile.close();
}


void MoveRobot::closeURConnection(RTDEControlInterface controller)
{
    controller.servoStop();
}

std::vector<double> MoveRobot::targetPointTransform(std::vector<double> startPoint, std::vector<double> targetPoint, double zRotation){

    cv::Mat startPointRvec = (cv::Mat_<double>(3,1) << startPoint[3], startPoint[4], startPoint[5]);
    cv::Mat startPointRM;
    cv::Mat startPointTvec = (cv::Mat_<double>(3,1) << startPoint[0], startPoint[1], startPoint[2]);

    Rodrigues(startPointRvec, startPointRM);

    cv::Mat startPointTransform = (cv::Mat_<double>(4, 4) <<
            startPointRM.at<double>(0,0), startPointRM.at<double>(0,1), startPointRM.at<double>(0,2), startPointTvec.at<double>(0,0),
            startPointRM.at<double>(1,0), startPointRM.at<double>(1,1), startPointRM.at<double>(1,2), startPointTvec.at<double>(0,1),
            startPointRM.at<double>(2,0), startPointRM.at<double>(2,1), startPointRM.at<double>(2,2), startPointTvec.at<double>(0,2),
            0, 0, 0, 1);

    cv::Mat targetPointRvec = (cv::Mat_<double>(3,1) << targetPoint[3], targetPoint[4], targetPoint[5]);
    cv::Mat targetPointRM;
    cv::Mat targetPointTvec = (cv::Mat_<double>(3,1) << targetPoint[0], targetPoint[1], targetPoint[2]);

    Rodrigues(targetPointRvec, targetPointRM);

    cv::Mat targetTransform = (cv::Mat_<double>(4, 4) <<
            targetPointRM.at<double>(0,0), targetPointRM.at<double>(0,1), targetPointRM.at<double>(0,2), targetPointTvec.at<double>(0,0),
            targetPointRM.at<double>(1,0), targetPointRM.at<double>(1,1), targetPointRM.at<double>(1,2), targetPointTvec.at<double>(0,1),
            targetPointRM.at<double>(2,0), targetPointRM.at<double>(2,1), targetPointRM.at<double>(2,2), targetPointTvec.at<double>(0,2),
            0, 0, 0, 1);

    cv::Mat endTransform = startPointTransform.inv() * targetTransform;

    cv::Mat endPointRM = (cv::Mat_<double>(3,3) <<
                      endTransform.at<double>(0,0), endTransform.at<double>(0,1), endTransform.at<double>(0,2),
                      endTransform.at<double>(1,0), endTransform.at<double>(1,1), endTransform.at<double>(1,2),
                      endTransform.at<double>(2,0), endTransform.at<double>(2,1), endTransform.at<double>(2,2));

    cv::Mat endPointRVec;

    Rodrigues(endPointRM, endPointRVec);

    cv::Mat endPointTvec = (cv::Mat_<double>(3,1) << endTransform.at<double>(0,3), endTransform.at<double>(1,3), endTransform.at<double>(2,3));

    std::vector<double> endPoint = {endPointTvec.at<double>(0,0), endPointTvec.at<double>(0,1), endPointTvec.at<double>(0,2), endPointRVec.at<double>(0,0), endPointRVec.at<double>(0,1), endPointRVec.at<double>(0,2) + zRotation};

    return endPoint;
}

void MoveRobot::writeFileRobotTCPPoses(bool flagChoice, std::vector<std::vector<double>> robotPoses){
    if (flagChoice) {
        std::ofstream myfile;
        myfile.open ("../Detection/RobotposeData.txt", std::ios::app);
        myfile.clear();
        for (int i = 0; i < robotPoses.size(); i++) {
            myfile << readVector(robotPoses[i]) << ";" <<  std::endl;
        }
        myfile.close();
        writeFileRobotTCPPosesFlagChoice = false;
    }else{
    std::ofstream myfile;
    myfile.open ("../Detection/RobotposeData.txt", std::ios::app);
    for (int i = 0; i < robotPoses.size(); i++) {
        myfile << readVector(robotPoses[i]) << ";" <<  std::endl;
    }
    myfile.close();
    }
}

