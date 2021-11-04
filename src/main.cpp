#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/cvdef.h"
#include <iostream>
#include <pylon/PylonIncludes.h>
#include "cameraCalibration.h"
#include "workspaceCalibration.h"
#include "detectionObject.h"
#include "detectionMarker.h"
#include "moveArm.h"
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_export.h>
#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde_utility.h>
#include "myHTTPClient.h"



using namespace cv;
using namespace std;


int main(int argc, char* argv[]){

    /* Simulate */
    bool runWhitGUI = false;
    /* HTTP Server variables */
    std::string httpServerIP    = "10.126.33.179";
    int httpServerPort          = 33333;

    // working on HTTP Com for database
    std::string ur5IP = "192.168.100.50";

    ur_rtde::RTDEIOInterface rtde_io(ur5IP);
    ur_rtde::RTDEReceiveInterface rtde_receive(ur5IP);
    ur_rtde::RTDEControlInterface rtde_control(ur5IP, 30004);
    /* RTDE handler object */
    MoveArm ur5arm;
    // HTTP
    MyHTTPClient httpClient(httpServerIP,httpServerPort);


    // I made something
    bool runCalibrateCameraSekvens = false;
    bool runCalibrateWorkSpaceSekvens = false;
    bool runDetectionMarker = true;
    bool runComToRobot = false;
    bool runMainSekvens = false;
    bool runTransSekvens = false;

    /* Camera variabler */
    int lengthXROImm     = 599;  // Lengt of width in cammera region of interrest in [mm] on tabele
    int lengthYROImm     = 959;  // Lengt of height in cammera region of interrest in [mm] on tabele

    /* cammera object */
    CameraSettings cameraSettings; /* Width default parameter*/
    DetectionObject checkerboard(cameraSettings); /* Width default parameter*/
    CameraCalibration cameraCalibrate(cameraSettings); /* Width default parameter*/
    WorkspaceCalibration workspaceCalibrate(cameraSettings, checkerboard);
    DetectionMarker detectMarker(cameraSettings);
    MoveArm robotCom;



    //////////// Calibrate camera sekvens //////////////
    if (runCalibrateCameraSekvens)
    {

        std::cout << "/* Calibrate camera sekvens */" << std::endl;
        cameraCalibrate.initialize(rtde_receive, rtde_control);
        //httpClient.sendCameraCalibrationMatrix(cameraCalibrate.cameraCalibrationToString());
    }

    //////////// Transformation sekvens //////////////
    if (runTransSekvens)
    {
        std::cout << "/* Transformation sekvens */" << std::endl;
        workspaceCalibrate.initialize(rtde_receive, rtde_control, lengthXROImm, lengthYROImm);

    }

    /////////// Calibrate workspace sekvens /////////////
     if (runCalibrateWorkSpaceSekvens)
     {
         std::cout << "/* Calibrate workspace sekvens */" << std::endl;
         workspaceCalibrate.initialize(rtde_receive,rtde_control,lengthXROImm, lengthYROImm);
     }
     //////////// Detection marker sekvens //////////////
     if (runDetectionMarker)
     {
         std::cout << "/* Detection marker sekvens */" << std::endl;
         detectMarker.initialize(rtde_receive, rtde_control);

     }
     //////////// Communikation robot sekvens //////////////
     if (runComToRobot)
     {

         std::cout << "/* Communikation robot sekvens */" << std::endl;
         //robotCom.initialize(rtde_receive, rtde_control);

//         std::vector<double> baseFrame = rtde_receive.getTargetTCPPose();
//        // std::cout << "move Base: " << ur5arm.readVector(baseFrame) << std::endl;
//         //std::vector<double> base = {-0.0423306,-0.384721,0.27404,2.56833,-1.80919,-0.0107535};
//         std::vector<double> featureFrame = {0.039,0.030,0.240,0.0,0.0,0.0};
//         //ur_rtde::RTDEControlInterface rtde_control("192.168.100.50", 30004);
//         std::vector<double> moveFrame;

//         moveFrame = rtde_control.poseTrans(baseFrame,featureFrame);

//         std::cout << "move Move: " << ur5arm.readVector(moveFrame) << std::endl;
//         //std::cout << controller.isConnected() << std::endl;
//         //base[2] += 0.1;
//         rtde_control.moveL({moveFrame}, 0.10,0.10);

         //robotCom.getToCheckerboard(rtde_receive, rtde_control);

         //process.receivePose(reciver);
         //process.getPoseFile("../Detection/RobotposeData.txt");




     }


     //////////// Main robot sekvens //////////////
     if (runMainSekvens){
         std::cout << "/* Main robot sekvens */" << std::endl;

         // 1. Move to Checkerboard Via workspace vector point to center
         // 2. Run Calibrate
         // 3. Run Workspace that get origo point fra vector
         // 4. Run Move to target with movearm func.



         ur5arm.poseSwift(rtde_receive, rtde_control, 0.5, 0.5);
//         while(true){
//             try
//             {
//                 ur5arm.poseSwift(rtde_receive, rtde_control, 0.5, 0.5);
//                //ur5arm.moveCalibrate(rtde_receive,rtde_control, 0.5, 0.5);

//             }
//             catch (std::exception e )
//             {
//                 e.what();
//                 std::cout << e.what() << " + FUUUUUUCK " << std::endl;
//                 break;
//             }

//         }


     }


         return 0;
}
