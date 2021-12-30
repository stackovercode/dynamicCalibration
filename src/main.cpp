#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/cvdef.h"
#include <iostream>
#include <pylon/PylonIncludes.h>
#include "cameraCalibration.h"
#include "workspaceCalibration.h"
#include "detectionMarker.h"
#include "moveArm.h"
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_export.h>
#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde_utility.h>
#include <QtCore/QCoreApplication>
#include "socket.h"
#include "mytcpserver.h"
#include <math.h>
#include <visp3/vision/vpHandEyeCalibration.h>
//#include <rw/rw.hpp>



using namespace cv;
using namespace std;


int main(int argc, char* argv[]){
    /* Universal robot IP-Adress */
    std::string ur5IP = "192.168.100.50";

    /* RTDE handler object */
    ur_rtde::RTDEIOInterface rtde_io(ur5IP);
    ur_rtde::RTDEReceiveInterface rtde_receive(ur5IP);
    ur_rtde::RTDEControlInterface rtde_control(ur5IP, 30004);

    /* Movement object */
    MoveArm ur5arm;

    // I made something
    bool runFinalSekvens = true;
    bool runCalibrateCameraSekvens = false;
    bool runCalibrateWorkSpaceSekvens = false;
    bool runDetectionMarker = false;
    bool runComToRobot = false;
    bool runMainSekvens = false;
    bool runTransSekvens = false;

    /* Confirgurations objects */
    CameraSettings cameraSettings; /* Width default parameter*/
    CameraCalibration cameraCalibrate(cameraSettings); /* Width default parameter*/
    WorkspaceCalibration workspaceCalibrate;
    DetectionMarker detectMarker(cameraSettings);
    MoveArm robotCom;


    //////////// Final sekvens //////////////
    if (runFinalSekvens){
        std::cout << "/* Final sekvens */" << std::endl;
        /* Varibles */
        bool beginProces = false;
        int procesState = 0;
        bool procesVerification;
        std::vector<double> jointBase = {0,0,0,0,0,0};
        cv::Vec6d poses;

        /* Process states: */
        /*
            * 1:....Complete the safety check and begin proces state.
            * 2:....Robotarm has reach the overview position.
            * 3:....
            * 4:....
            * 5:....
            * 6:....
            * 7:....
            * 8:....
            * 9:....
            * 10:...
        */


        /* Stoprobot for Safty if any script is runnning on the robots */
        try {
            if (!(rtde_control.isProgramRunning())) {
                /* Safety Aproved */
                std::cout << "Safety Aproved" << std::endl;
            }
        } catch (exception& e) {
            /* Safety Declined */
            std::cout << "Safety Declined" << std::endl;
            std::cout << "Exception"
                         ": " << std::endl;
        }



            std::cout << "Press Y to begin: ";

            char keyPressed;
            std::cin>>keyPressed;

            if (keyPressed == 'y'|| keyPressed == 'Y' ) {
                beginProces = true;
                procesState = 1;
            } else {
                std::cout << "Wrong command, not ready to begin." << std::endl;
            }


        while (beginProces) {

            if (procesState == 1) {
                procesVerification = robotCom.initialize(rtde_receive, rtde_control);
                //detectMarker.initialize(rtde_receive, rtde_control, true);
                if (procesVerification) {
                    procesState = 2;
                }
            }
            if (procesState == 2) {
                detectMarker.initialize(rtde_receive, rtde_control, poses, true);
                procesVerification = true;
                if (procesVerification) {
                    procesState = 3;
                }
            }
//            if (procesState == 3) {
//                cameraCalibrate.initialize(rtde_receive, rtde_control);
//                procesVerification = detectMarker.mainProcesState;
//                if (procesVerification) {
//                    procesState = 4;
//                }
//            }
            if (procesState == 3) {
                ur_rtde::RTDEReceiveInterface receiverNew("192.168.100.50");
                jointBase = receiverNew.getTargetQ();
                for (size_t i = 0; i < jointBase.size(); i++) {
                    poses[i] = jointBase[i];
                }
                detectMarker.initialize(rtde_receive, rtde_control, poses, true);
                procesVerification = detectMarker.mainProcesState;
                if (procesVerification) {
                    procesState = 4;
                    break;
                }
            }


}

//        cameraCalibrate.initialize(rtde_receive, rtde_control);
//        detectMarker.initialize(rtde_receive, rtde_control, false);
//        cv::Vec6d point = detectMarker.mRobotPoint3d;
//        std::vector<double> baseFrame = detectMarker.moveFrame;
//        robotCom.getToJob(rtde_receive, rtde_control, point, baseFrame, 0, 0.05, 0.05);
    }

    //////////// Calibrate camera sekvens //////////////
    if (runCalibrateCameraSekvens)
    {

        std::cout << "/* Calibrate camera sekvens */" << std::endl;
        //cameraCalibrate.initialize(rtde_receive, rtde_control,);
        //httpClient.sendCameraCalibrationMatrix(cameraCalibrate.cameraCalibrationToString());
    }

    //////////// Transformation sekvens //////////////
    if (runTransSekvens)
    {
        std::cout << "/* Transformation sekvens */" << std::endl;
        //workspaceCalibrate.initialize(rtde_receive, rtde_control, lengthXROImm, lengthYROImm);
        std::cout << "HandEye: " <<  workspaceCalibrate.getTransformationFlange2CameraHandEye(23,1) << std::endl;

    }

    /////////// Calibrate workspace sekvens /////////////
     if (runCalibrateWorkSpaceSekvens)
     {
         std::cout << "/* Calibrate workspace sekvens */" << std::endl;
         //workspaceCalibrate.initialize(rtde_receive,rtde_control,lengthXROImm, lengthYROImm);
     }
     //////////// Detection marker sekvens //////////////
     if (runDetectionMarker)
     {
         std::cout << "/* Detection marker sekvens */" << std::endl;


         //detectMarker.initialize(rtde_receive, rtde_control, true);
         //cv::Vec6d point = detectMarker.mRobotPoint3d;
         //std::cout << "featureFrame: " << point << std::endl;

     }
     //////////// Communikation robot sekvens //////////////
     if (runComToRobot){

         QCoreApplication a(argc, argv);

         Socket Connection;
         MyTcpServer server;
         Connection.Running();

         return a.exec();



//         std::cout << "/* Communikation robot sekvens */" << std::endl;
//         //robotCom.initialize(rtde_receive, rtde_control);
//         int progress = 1;
//         double velocity = 0.02;
//         double acceleration = 0.02;
//         //detectMarker.initialize(rtde_receive, rtde_control);
//         //cv::Point3d point = {0.0146505, 0.0136401, 0.0};
//         cv::Vec6d point = detectMarker.mRobotPoint3d;
//         while(progress < 4 ){
//             std::cout << "Inside while loop. start Progress: " << progress << std::endl;
//             //robotCom.getToJob(rtde_receive, rtde_control, point, , progress, velocity, acceleration);
//             progress++;
//         }

     }


     //////////// Main robot sekvens //////////////
     if (runMainSekvens){
         std::cout << "/* Main robot sekvens */" << std::endl;


//         const std::string my_script =
//                 "def script_test():\n"
//                    "\tglobal featureFrame=p[-0.00417425,-0.00252985,0,0,0,-0.0371178]\n"
//                 "end\n";
//         bool my_result = rtde_control.sendCustomScript(my_script);

//         std::cout << "Result: " << my_result << std::endl;





//         const std::string inline_script =
//                    "def script_test():\n"
//                            "\tdef test():\n"
//                                    "textmsg(\"test1\")\n"
//                                    "textmsg(\"test2\")\n"
//                            "\tend\n"
//                            "\twrite_output_integer_register(0, 1)\n"
//                            "\ttest()\n"
//                            "\ttest()\n"
//                            "\twrite_output_integer_register(0, 2)\n"
//                    "end\n"
//                    "run program\n";
//              bool result = rtde_control.sendCustomScript(inline_script);

//              std::cout << "Result: " << result << std::endl;


         // 1. Move to Checkerboard Via workspace vector point to center
         // 2. Run Calibrate
         // 3. Run Workspace that get origo point fra vector
         // 4. Run Move to target with movearm func.



        std::cout << "test: " << ur5arm.readVector(ur5arm.receivePose(rtde_receive)) << std::endl;

//         bool test = true;
//         char keyPressed;
//         while(test){
//             keyPressed = cv::waitKey(1);
//             if(keyPressed == 'n'|| keyPressed == 'N' ){
//                 std::cout << "N" << std::endl;
//              } else if(keyPressed == 'q'|| keyPressed == 'Q' ){
//                 std::cout << "Q" << std::endl;
//                 test = false;
//             }

            //ur5arm.poseSwift(rtde_receive, rtde_control, 0.1, 0.1, 1, {0.0,0.0,0.0,0.0,0.0,0.0}, 25, true);

//         }


     }


         return 0;
}
