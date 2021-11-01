#include "moveArm.h"
#include "algorithm"
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>
#include <iostream>


using namespace ur_rtde;
using namespace std;

MoveArm::MoveArm(){

}

MoveArm::~MoveArm(){}

void MoveArm::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller){
    //MoveArm process
    //std::vector<double> hej = controller.getTCPOffset();
    //std::cout << "text: " << readVector(hej) << std::endl;

    std::vector<double> baseFrame = reciver.getActualTCPPose();
    std::cout << "move Base: " << readVector(baseFrame) << std::endl;
    std::vector<double> featureFrame = {0.1,0.1,0.1,2.56827,-1.80924,-0.0107754};
    //ur_rtde::RTDEControlInterface rtde_control("192.168.100.50", 30004);
    std::vector<double> moveFrame;

    moveFrame = controller.poseTrans(baseFrame,featureFrame);

    std::cout << "move Move: " << readVector(moveFrame) << std::endl;
    std::cout << controller.isConnected() << std::endl;
    //controller.moveL(moveFrame, 0.25,1.2);

    //process.receivePose(reciver);
    //process.getPoseFile("../Detection/RobotposeData.txt");


}

void MoveArm::getToCheckerboard(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller){
    std::vector<double> baseFrame = reciver.getTargetTCPPose();
    std::cout << "move Base: " << readVector(baseFrame) << std::endl;
    std::vector<double> featureFrame = {0.055,0.073,0.25,0.0,0.0,0.0};
    std::vector<double> moveFrame = controller.poseTrans(baseFrame,featureFrame);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "move Move: " << readVector(moveFrame) << std::endl;
    controller.moveL({moveFrame}, 0.10,0.10);

}

std::vector<double> MoveArm::receivePose(ur_rtde::RTDEReceiveInterface &reciver){
    std::vector<double> jointPose = reciver.getActualQ();
//    for (int i = 0; i < jointPose.size(); i++) {
//       jointPose[i] = jointPose[i]*rad2deg;
//    }
    return jointPose;
}

void MoveArm::poseSwift(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity, double acceleration){
    std::cout << "Moving to checker postion" << std::endl;

    //std::vector<double> startingPose = reciver.getTargetTCPPose();

    double blend = 0.0;

    std::vector<std::vector<double>> path_q;

    //Dataset 2
    std::vector<double> startPos_q6 = {-1.58481,-1.48048,-1.75088,-3.06395,-1.90983,1.75687, velocity, acceleration, blend};
    path_q.push_back(startPos_q6);
    std::vector<double> startPos_q1 = {-1.36689,-1.28005,-1.90308,-3.10318,-1.70538,1.5708, velocity, acceleration, blend};
    path_q.push_back(startPos_q1);

    controller.moveJ(path_q, false);
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //controller.stopJ(0.20);
    //controller.stopScript();
    ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
    //receivePose(receiverNew);
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<double> pose = receiverNew.getActualTCPPose();
    //std::cout << readVector(pose) << std::endl;

    path_q.clear();
    path_q.push_back(startPos_q6);
    controller.moveJ(path_q, false);

    //controller.moveL(pose, velocity, acceleration);

    std::vector<double> startPos_q2 = {-78*deg2rad,-73*deg2rad,-109*deg2rad,-177*deg2rad,-97*deg2rad,89*deg2rad, velocity, acceleration, blend};

    path_q.clear();
    path_q.push_back(startPos_q2);
    controller.moveJ(path_q, false);

}

std::vector<double> MoveArm::moveCalibrate(RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface& controller, double velocity, double acceleration, int positionStatus)
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
        path_q.push_back(startPos_q26);

        /*//Dataset 1
        std::vector<double> startPos_q1 = {-65*deg2rad, -89*deg2rad, -90*deg2rad, -179*deg2rad, 0*deg2rad, 89*deg2rad, velocity, acceleration, blend};
        path_q.push_back(startPos_q1);
        std::vector<double> startPos_q2 = {-65*deg2rad,-80*deg2rad,-99*deg2rad,-179*deg2rad,0*deg2rad,95*deg2rad, velocity, acceleration, blend};
        path_q.push_back(startPos_q2);
        std::vector<double> startPos_q3 = {-65*deg2rad,-70*deg2rad,-108*deg2rad,-179*deg2rad,0*deg2rad,106*deg2rad, velocity, acceleration, blend};
        path_q.push_back(startPos_q3);
        std::vector<double> startPos_q4 = {-67*deg2rad,-102*deg2rad,-75*deg2rad,-171*deg2rad,5*deg2rad,67*deg2rad, velocity, acceleration, blend};
        path_q.push_back(startPos_q4);
        std::vector<double> startPos_q5 = {-1.17525,-1.9928,-1.14322,-3.08425,-0.0265792,1.15823, velocity, acceleration, blend};
        path_q.push_back(startPos_q5);
        std::vector<double> startPos_q6 = {-1.67486,-1.71752,-1.54214,-3.11933,-1.95247,1.82232, velocity, acceleration, blend};
        path_q.push_back(startPos_q6);
        std::vector<double> startPos_q7 = {-1.92125,-1.88368,-1.31198,-3.19834,-2.13113,1.9989, velocity, acceleration, blend};
        path_q.push_back(startPos_q7);
        std::vector<double> startPos_q8 = {-0.939013,-1.1956,-2.02712,-3.08006,-1.2746,1.33868, velocity, acceleration, blend};
        path_q.push_back(startPos_q8);
        std::vector<double> startPos_q9 = {-0.669973,-1.28826,-1.86764,-3.17137,-0.973059,1.24124, velocity, acceleration, blend};
        path_q.push_back(startPos_q9);
        std::vector<double> startPos_q10 = {-1.02239,-1.68582,-1.50754,-2.90805,-0.147546,1.15198, velocity, acceleration, blend};
        path_q.push_back(startPos_q10);
        std::vector<double> startPos_q11 = {-1.00707,-1.89985,-1.32997,-2.92559,-0.258037,1.01791, velocity, acceleration, blend};
        path_q.push_back(startPos_q11);
        std::vector<double> startPos_q12 = {-1.40569,-1.28339,-1.89509,-3.18822,-0.44815,1.95826, velocity, acceleration, blend};
        path_q.push_back(startPos_q12);
        std::vector<double> startPos_q13 = {-1.62346,-0.997068,-2.15913,-3.17817,-0.627815,2.21539, velocity, acceleration, blend};
        path_q.push_back(startPos_q13);
        std::vector<double> startPos_q14 = {-1.97083,-1.5071,-1.6907,-3.03317,-1.80962,2.05435, velocity, acceleration, blend};
        path_q.push_back(startPos_q14);
        std::vector<double> startPos_q15 = {-2.2579,-1.71241,-1.46222,-3.03268,-2.17991,2.22559, velocity, acceleration, blend};
        path_q.push_back(startPos_q15);
        std::vector<double> startPos_q16 = {-0.87396,-1.45053,-1.67357,-3.24048,-1.02501,1.25937, velocity, acceleration, blend};
        path_q.push_back(startPos_q16);
        std::vector<double> startPos_q17 = {-0.673228,-1.64072,-1.49262,-3.24053,-0.655377,1.1822, velocity, acceleration, blend};
        path_q.push_back(startPos_q17);
        std::vector<double> startPos_q18 = {-0.425662,-1.10118,-1.9831,-3.22662,-0.973275,1.14836, velocity, acceleration, blend};
        path_q.push_back(startPos_q18);
        std::vector<double> startPos_q19 = {-0.242841,-1.03813,-2.07674,-3.22216,-0.730345,1.12017, velocity, acceleration, blend};
        path_q.push_back(startPos_q19);
        std::vector<double> startPos_q20 = {-1.88274,-1.92474,-1.22369,-3.10531,-2.62499,2.0271, velocity, acceleration, blend};
        path_q.push_back(startPos_q20);
        std::vector<double> startPos_q21 = {-1.98976,-2.07412,-1.04277,-3.12221,-2.62734,2.18131, velocity, acceleration, blend};
        path_q.push_back(startPos_q21);
        std::vector<double> startPos_q22 = {-1.33825,-1.87563,-1.2207,-2.99749,0.236173,1.19899, velocity, acceleration, blend};
        path_q.push_back(startPos_q22);
        std::vector<double> startPos_q23 = {-1.40842,-2.09429,-0.931311,-3.1257,0.360255,1.16669, velocity, acceleration, blend};
        path_q.push_back(startPos_q23);
        std::vector<double> startPos_q24 = {-0.975167,-1.17637,-1.98857,-3.07356,0.654298,1.92787, velocity, acceleration, blend};
        path_q.push_back(startPos_q24);
        std::vector<double> startPos_q25 = {-0.695715,-1.04212,-2.0269,-3.0741,1.20606,2.04704, velocity, acceleration, blend};
        path_q.push_back(startPos_q25);
        std::vector<double> startPos_q26 = {-65*deg2rad, -89*deg2rad, -90*deg2rad, -179*deg2rad, 0*deg2rad, 89*deg2rad, velocity, acceleration, blend};
        path_q.push_back(startPos_q26);
        */

        tempPath_q.push_back(path_q[i-1]);

        if (i == positionStatus+1) {
//            if (positionStatus == 25) {
//                lastPose = true;
//            }
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


std::string MoveArm::readVector(std::vector<double> result)
{
    std::ostringstream vts;
    if (!result.empty()){
    std::copy(result.begin(), result.end()-1,
        std::ostream_iterator<double>(vts, ","));
        vts << result.back();
    }
    return vts.str();
}



void MoveArm::writeFileRobotPoses(std::vector<double> robotPoses){
    std::ofstream myfile;
    myfile.open ("../Detection/RobotjointData.txt", std::ios::app);
        myfile << readVector(robotPoses) << ";" << std::endl;
    myfile.close();
}


void MoveArm::closeURConnection(RTDEControlInterface controller)
{
    controller.servoStop();
}
