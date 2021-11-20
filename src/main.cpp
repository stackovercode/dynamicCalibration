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
#include <QtCore/QCoreApplication>
#include "socket.h"
//#include <rw/rw.hpp>



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
    bool runFinalSekvens = false;
    bool runCalibrateCameraSekvens = false;
    bool runCalibrateWorkSpaceSekvens = false;
    bool runDetectionMarker = false;
    bool runComToRobot = true;
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


    //////////// Final sekvens //////////////
    if (runFinalSekvens){
        std::cout << "/* Final sekvens */" << std::endl;
        robotCom.initialize(rtde_receive, rtde_control);
        detectMarker.initialize(rtde_receive, rtde_control, true);
        cameraCalibrate.initialize(rtde_receive, rtde_control);
        detectMarker.initialize(rtde_receive, rtde_control, false);
        cv::Vec6d point = detectMarker.mRobotPoint3d;
        std::vector<double> baseFrame = detectMarker.moveFrame;
        robotCom.getToJob(rtde_receive, rtde_control, point, baseFrame, 0, 0.05, 0.05);
    }

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


         detectMarker.initialize(rtde_receive, rtde_control, true);
         cv::Vec6d point = detectMarker.mRobotPoint3d;
         std::cout << "featureFrame: " << point << std::endl;

     }
     //////////// Communikation robot sekvens //////////////
     if (runComToRobot){

         QCoreApplication a(argc, argv);

         Socket Connection;
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


         const std::string my_script =
                 "def script_test():\n"
                    "\tglobal featureFrame=p[-0.00417425,-0.00252985,0,0,0,-0.0371178]\n"
                 "end\n";
         bool my_result = rtde_control.sendCustomScript(my_script);

         std::cout << "Result: " << my_result << std::endl;





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



       // std::cout << "test: " << ur5arm.readVector(ur5arm.receivePose(rtde_receive)) << std::endl;

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

//            // ur5arm.poseSwift(rtde_receive, rtde_control, 0.02, 0.02, 1, {0.0,0.0,0.0,0.0,0.0,0.0}, 25, true);

//         }


     }


         return 0;
}
