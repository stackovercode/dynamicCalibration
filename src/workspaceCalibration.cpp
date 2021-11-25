#include "workspaceCalibration.h"

/********************************************************************************
*                               CONSTRUCTOR                                     *
********************************************************************************/

WorkspaceCalibration::WorkspaceCalibration(){

}

WorkspaceCalibration::~WorkspaceCalibration(){}



//WorkspaceCalibration::WorkspaceCalibration(CameraSettings& cameraSetting, DetectionObject& detectionObject)
//    : Camera(cameraSetting), mDetectionObject{detectionObject}{
//    ////////// Redding ind data //////////////
//    getCalibrationData("../Detection/CalibrationData.txt");
//    loadFileTranRot("../Detection/MarkertransposeData.txt");
//    loadFileRobotJoint("../Detection/RobotjointData.txt");
//    loadFileRobotTCP("../Detection/RobotposeData.txt");
//    loadFileImagePoints("../Detection/CalibrationData.txt");
//    /////////// Creating rectifying Maps //////////////
//    cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, mCamerasettings.getResolution(), CV_32FC1, mMapX, mMapY);
//}

cv::Mat WorkspaceCalibration::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, double lengthXmm, double lengthYmm, cv::Vec6f robotJointAngels){
    //cv::Vec6f robotJointAngles = {-1.36689,-1.28005,-1.90308,-3.10318,-1.70538,1.5708};
//    cv::Vec6f robotJointAngles = mJointPose[24];
//    cv::Mat testMatrix = getTransformationMatrixBase2Cam(robotJointAngles);

//    cv::Point2f centerPoint = mCenterPoint;
//    cv::Point2f diaPoint = mDiaPoint;

//    cv::Point2i frameCenter = {1920/2, 1200/2};

//    cv::Mat OrigoMat = (cv::Mat_<double>(4, 1) << vectorBetween2Points(imageFramePoints[0],frameCenter).x, vectorBetween2Points(imageFramePoints[0],frameCenter).y, getDistance2Object(centerPoint, diaPoint)/getPixelPermm(centerPoint, diaPoint), 1);
//    cv::Mat OrigoPoint = getTransformationMatrixImage2Camera(mR_target2cam[0], mT_target2cam[0]) * OrigoMat;
//    OrigoPoint.at<double>(0,0) = (OrigoMat.at<double>(0,0) * getPixelPermm(centerPoint, diaPoint))/1000;
//    OrigoPoint.at<double>(1,0) = (OrigoMat.at<double>(1,0) * getPixelPermm(centerPoint, diaPoint))/1000;
//    OrigoPoint.at<double>(2,0) = (OrigoMat.at<double>(2,0) * getPixelPermm(centerPoint, diaPoint))/1000;

//    cv::Mat robotPoint = testMatrix * OrigoPoint;

//    std::cout << "Point: " << robotPoint << std::endl;
//    std::cout << "Transformation: " << testMatrix << std::endl;

    cv::Mat robotTransformationMatrix = getRobotTransformationMatrix(robotJointAngels) * getTransformationFlange2EndEffector() * getTransformationEndEffector2Camera();


    return robotTransformationMatrix;

}

void WorkspaceCalibration::action(Pylon::CInstantCamera& camera, ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller)
{

//    Pylon::CImageFormatConverter formatConverter;
//    formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
//    Pylon::CGrabResultPtr ptrGrabResult;
//    Pylon::CPylonImage pylonImage;
//    cv::Mat openCvImage, undistortedImage;

//    for (int i = 0; i < mWorkspaceCorners.size(); ++i) {
//        bool isGrabbingPosition = false;
//        cv::Vec3d vecObjectPosition;
//        std::vector<double> objectVectorX;
//        std::vector<double> objectVectorY;
//        std::vector<double> objectVectorZ;

//        int noDetection = 0;
//        int grabbingIterator = 0;
//        int frame = 1;
//        while ( camera.IsGrabbing())
//        {
//            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

//            if (ptrGrabResult->GrabSucceeded())
//            {
//                formatConverter.Convert(pylonImage, ptrGrabResult);
//                openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

//                cv::remap(openCvImage, undistortedImage, mMapX, mMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

////                bool isObjectFound = mDetectionObject.locate(vecObjectPosition, undistortedImage);
////                if ( !isObjectFound )
////                {
////                    ++noDetection;
////                    std::cerr << "Error: object not detected:" << noDetection << " times."<< std::endl;
////                }

//                //Textbox counter in the bottom of video feed
//                std::stringstream ss;
//                ss << "[ " << grabbingIterator << " images of 10" <<" ]";
//                //cv::putText(undistortedImage, ss.str(), cv::Point(600,30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255),2);

//                // Create an OpenCV display window.
//                std::stringstream window;
//                window << "Video feed:" << ( ( !isGrabbingPosition ) ? "Press G to grab Object Center" : " "  )
//                                        << ( ( i==0 ) ? " At top left corner in ROI" : " "  )
//                                        << ( ( i==1 ) ? " At top right corner in ROI" : " "  )
//                                        << ( ( i==2 ) ? " At buttom left corner in ROI" : " "  )
//                                        << ( ( i==3 ) ? " At buttom right corner in ROI" : " "  );
//                cv::namedWindow( window.str() , cv::WINDOW_AUTOSIZE);
//                cv::imshow( window.str(), undistortedImage);

//                // Detect 'key' is pressed
//                int keyPressed = cv::waitKey(1);
//                if(keyPressed == 'g'|| keyPressed == 'G' || isGrabbingPosition)
//                {
//                    isGrabbingPosition = true;

//                    if (grabbingIterator < 10 && isObjectFound){
//                        objectVectorX.push_back(vecObjectPosition[0]);
//                        objectVectorY.push_back(vecObjectPosition[1]);
//                        objectVectorZ.push_back(vecObjectPosition[2]);

//                        grabbingIterator++;

//                    } else if(grabbingIterator == 10) {
//                        std::sort(objectVectorX.begin(), objectVectorX.end());
//                        std::sort(objectVectorY.begin(), objectVectorY.end());
//                        std::sort(objectVectorZ.begin(), objectVectorZ.end());

//                        cv::Point meanObjectPosition(objectVectorX[5], objectVectorY[5]);
//                        mWorkspaceCorners[i] = meanObjectPosition;
//                        std::cout << "Mean object position =\n " << meanObjectPosition << std::endl;

//                        std::cout << "Destroying Window..." << std::endl;
//                        cv::destroyAllWindows();

//                        std::stringstream vindue;
//                        vindue << "Result of grab";
//                        cv::namedWindow( vindue.str() , cv::WINDOW_AUTOSIZE);
//                        cv::imshow( vindue.str(), undistortedImage);
//                        cv::waitKey(0);
//                        break;
//                    }
//                }

//                frame++;
//            }
//            else
//            {
//                std::cerr << "[ Fail ]:  " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
//                exit(-1);
//            }
//        }
//    }
//    std::cout << "Shutting down camera..." << std::endl;
//    camera.Close();
//    std::cout << "Camera successfully closed." << std::endl;

//    std::cout << "point1 " << mWorkspaceCorners[0] << std::endl
//              << "point2 " << mWorkspaceCorners[1] << std::endl
//              << "point3 " << mWorkspaceCorners[2] << std::endl
//              << "point4 " << mWorkspaceCorners[3] << std::endl;
}

void WorkspaceCalibration::getCalibrationData(std::string fileLocation)
{
    std::ifstream ifs;
    cv::Matx21f centerPointVec;
    cv::Matx21f diaPointVec;
    cv::Matx31d rvecV;
    cv::Matx31d tvecV;
    cv::Mat rvecMat;
    cv::Mat tvecMat;

    cv::Point2f CenterPoint;
    cv::Point2f DiaPoint;

    ifs.open(fileLocation);
    if (ifs.fail())
    {
        std::cerr <<"[ Error ]: ifs.fail "<< fileLocation << " File not fund. /n"
                 <<"function: Detection::getCalibrationData" << std::endl;
        exit(1);
    }
    else
    {
        std::string trashString;
        char trashChar;
        float tempFloat;
        int tempInt;

        std::getline(ifs,trashString, '=');
        ifs >> mErrorRMS;

        std::getline(ifs,trashString, '[');
        for (int index = 0; index < 9; ++index)
        {
            ifs >> tempFloat;
            mCameraMatrix.val[index] =tempFloat;
            ifs >> trashChar;
        }

        std::getline(ifs,trashString, '[');
        for (int index = 0; index < 5; ++index)
        {
            ifs >> tempFloat;
            mDistortionCoefficient[index] = tempFloat;
            ifs >> trashChar;
        }

        // Her skal center og diagonal hentes.
        for (int index = 0; index <2; index++){

              ifs >> tempFloat;
              centerPointVec.val[index] = tempFloat;
              ifs >> trashChar;
          }
          CenterPoint.x = centerPointVec.val[0];
          CenterPoint.y = centerPointVec.val[1];
          //cout << CenterPoint <<endl;

          getline(ifs, trashString, '=');
          for (int index = 0; index <2; index++){

              ifs >> tempFloat;
              diaPointVec.val[index] = tempFloat;
              ifs >> trashChar;
          }
          DiaPoint.x = diaPointVec.val[0];
          DiaPoint.y = diaPointVec.val[1];
          //cout << DiaPoint <<endl;
        // Her slutter center og diagonal

        std::getline(ifs,trashString, '[');
        for (int i = 0; i < 3; ++i)
        {
            ifs >> tempFloat;
            mRotMatrix.push_back(tempFloat);
            ifs >> trashChar;
        }
        std::getline(ifs,trashString, '[');
        for (int i = 0; i < 3; ++i)
        {
            ifs >> tempFloat;
            mTransMatrix.push_back(tempFloat);
            ifs >> trashChar;
        }


        for (int i = 0; i < 3; i++) {
            std::cout << mRotMatrix[i] << std::endl;
        }
        for (int i = 0; i < 3; i++) {
            std::cout << mTransMatrix[i] << std::endl;
        }

    }
}


std::string WorkspaceCalibration::transformationMatrixToString()
{
    std::stringstream ss;
    ss << mTransformationMatrix;
    std::string result = ss.str();
    return result;
}


cv::Mat WorkspaceCalibration::getTransformationMatrixBase2Cam(cv::Vec6f robotJointAngles){
    return getRobotTransformationMatrix(robotJointAngles) * getTransformationFlange2EndEffector() * getTransformationEndEffector2Camera();
}

cv::Mat WorkspaceCalibration::getRobotTransformationMatrix(cv::Vec6f robotJointAngles){

    //double deg2rad = 3.14159/180;

    cv::Mat T0_1 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[0] ), 0, sin(robotJointAngles[0]  ), 0,
        sin(robotJointAngles[0]  ), 0, -cos(robotJointAngles[0]  ), 0,
        0, 1, 0, 0.1625,
        0, 0, 0, 1);

    cv::Mat T1_2 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[1]  ), -sin(robotJointAngles[1]  ), 0, -0.425*cos(robotJointAngles[1]  ),
        sin(robotJointAngles[1]  ), cos(robotJointAngles[1]  ), 0, -0.425*sin(robotJointAngles[1]  ),
        0, 0, 1, 0,
        0, 0, 0, 1);

    cv::Mat T2_3 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[2]  ), -sin(robotJointAngles[2]  ), 0, -0.3922*cos(robotJointAngles[2]  ),
        sin(robotJointAngles[2]  ), cos(robotJointAngles[2]  ), 0, -0.3922*sin(robotJointAngles[2]  ),
        0, 0, 1, 0,
        0, 0, 0, 1);

    cv::Mat T3_4 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[3]  ), 0, sin(robotJointAngles[3]  ), 0,
        sin(robotJointAngles[3]  ), 0, -cos(robotJointAngles[3]  ), 0,
        0, 1, 0, 0.1333,
        0, 0, 0, 1);

    cv::Mat T4_5 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[4]  ), 0, -sin(robotJointAngles[4]  ), 0,
        sin(robotJointAngles[4]  ), 0, cos(robotJointAngles[4]  ), 0,
        0, -1, 0, 0.0997,
        0, 0, 0, 1);

    cv::Mat T5_6 = (cv::Mat_<double>(4, 4) <<
        cos(robotJointAngles[5]  ), -sin(robotJointAngles[5]  ), 0, 0,
        sin(robotJointAngles[5]  ), cos(robotJointAngles[5]  ), 0, 0,
        0, 0, 1, 0.0996,
        0, 0, 0, 1);

    cv::Mat T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6;

    return T0_6;

}


cv::Mat WorkspaceCalibration::getTransformationFlange2EndEffector(){

    cv::Mat TTFEE = (cv::Mat_<double>(4, 4) <<
        0, 0, -1, -0.18,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1);

    return TTFEE;
}


cv::Mat WorkspaceCalibration::getTransformationEndEffector2Camera(){

    cv::Mat TEECAM = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0.021,
        0, 1, 0, 0.043,
        0, 0, 1, -0.129,
        0, 0, 0, 1);

    return TEECAM;
}

cv::Mat WorkspaceCalibration::getTransformationMatrixImage2Camera(cv::Mat rvec, cv::Mat tvec){
    cv::Mat RRodriguesMatrix;

    cv::Rodrigues(rvec, RRodriguesMatrix);

    cv::Mat TCAMIMG = (cv::Mat_<double>(4, 4) <<
            RRodriguesMatrix.at<double>(0,0), RRodriguesMatrix.at<double>(0,1), RRodriguesMatrix.at<double>(0,2), tvec.at<double>(0,0),
            RRodriguesMatrix.at<double>(1,0), RRodriguesMatrix.at<double>(1,1), RRodriguesMatrix.at<double>(1,2), tvec.at<double>(0,1),
            RRodriguesMatrix.at<double>(2,0), RRodriguesMatrix.at<double>(2,1), RRodriguesMatrix.at<double>(2,2), tvec.at<double>(0,2),
            0, 0, 0, 1);
    return TCAMIMG;
}

double WorkspaceCalibration::lineLength(double sX, double sY, double eX, double eY){
    double v1Coordinate, v2Coordinate;

    v1Coordinate = eX-sX;
    v2Coordinate = eY-sY;

    double vLength = sqrt(pow(v1Coordinate, 2)+pow(v2Coordinate, 2));

    return vLength;
}

double WorkspaceCalibration::getDistance2Object(cv::Point2f origo, cv::Point2f dia){
    double focalL = 3316.188; //f = (pixel*distance)/width = (536.6*395.52/64)
    double widthObj = 64.03;

    double distanceObj = (widthObj*focalL)/ WorkspaceCalibration::lineLength(origo.x, origo.y, dia.x, dia.y);

    return distanceObj;
}

double WorkspaceCalibration::getPixelPermm(cv::Point2f origo, cv::Point2f dia){
    double widthObj = 64.03;

    double pixelPmm = widthObj / WorkspaceCalibration::lineLength(origo.x, origo.y, dia.x, dia.y);

    return pixelPmm;
}

void WorkspaceCalibration::writeFileTranRot (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec){
    std::ofstream myfile;
    myfile.open ("../Detection/MarkertransposeData.txt");
    for (int i = 0; i < tempRvec.size(); i++) {
        myfile << "Rvec: " << i+1 << " = \n" << tempRvec[i] << std::endl
                  << "Tvec: " << i+1 << " = \n" << tempRvec[i] << std::endl;
    }
    myfile.close();

}

void WorkspaceCalibration::loadFileTranRot(std::string fileLocation){

    cv::Matx31d rvec;
    cv::Matx31d tvec;

    cv::Mat rvecMat;
    cv::Mat tvecMat;

    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> T_target2cam;

    std::ifstream ifs;
    ifs.open(fileLocation);
    if (ifs.fail())
    {
        std::cerr <<"[ Error ]: ifs.fail "<< fileLocation << " File not fund. /n"
                 <<"function: Detection::getCalibrationData" << std::endl;
        exit(1);
    }
    else
    {
        while(ifs){

            std::string trashString;

            char trashChar;
            double tempFloat;

            std::getline(ifs,trashString, '=');

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 3; ++index)
            {
                ifs >> tempFloat;
                rvec.val[index] =tempFloat;
                ifs >> trashChar;
            }
            rvecMat = (cv::Mat_<double>(3,1) << rvec.val[0], rvec.val[1], rvec.val[2]);
            R_target2cam.push_back(rvecMat);


            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 3; ++index)
            {
                ifs >> tempFloat;
                tvec.val[index] = tempFloat;
                ifs >> trashChar;
            }
            tvecMat = (cv::Mat_<double>(3,1) << tvec.val[0], tvec.val[1], tvec.val[2]);
            T_target2cam.push_back(tvecMat);
        }
    }
    mR_target2cam = R_target2cam;
    mT_target2cam = T_target2cam;
}

void WorkspaceCalibration::loadFileRobotJoint(std::string fileLocation){
    cv::Matx61f jointPose;
    cv::Mat jointPoseMat;
    std::vector<cv::Mat> jointPoseVec;

    std::ifstream ifs;
    ifs.open(fileLocation);
    if (ifs.fail())
    {
        std::cerr <<"[ Error ]: ifs.fail "<< fileLocation << " File not fund. /n"
                 <<"function: Detection::getCalibrationData" << std::endl;
        exit(1);
    }
    else
    {
        while(ifs){

            std::string trashString;

            char trashChar;
            float tempFloat;

            for (int index = 0; index <6; index++){
                ifs >> tempFloat;
                jointPose.val[index] = tempFloat;
                ifs >> trashChar;
            }
            jointPoseMat = (cv::Mat_<float>(6,1) << jointPose.val[0], jointPose.val[1], jointPose.val[2], jointPose.val[3], jointPose.val[4], jointPose.val[5]);
            jointPoseVec.push_back(jointPoseMat);
        }
    }
    mJointPose = jointPoseVec;
}

void WorkspaceCalibration::loadFileRobotTCP(std::string fileLocation){

    cv::Matx61f tcpPose;

    cv::Mat tcpPoseMat;

    std::vector<cv::Mat> tcpPoseVec;

    std::ifstream ifs;
    ifs.open(fileLocation);
    if (ifs.fail())
    {
        std::cerr <<"[ Error ]: ifs.fail "<< fileLocation << " File not fund. /n"
                 <<"function: Detection::getCalibrationData" << std::endl;
        exit(1);
    }
    else
    {
        while(ifs){

            std::string trashString;

            char trashChar;
            float tempFloat;


            for (int index = 0; index <6; index++){

                ifs >> tempFloat;
                tcpPose.val[index] = tempFloat;
                ifs >> trashChar;
            }
            tcpPoseMat = (cv::Mat_<float>(6,1) << tcpPose.val[0], tcpPose.val[1], tcpPose.val[2], tcpPose.val[3], tcpPose.val[4], tcpPose.val[5]);
            tcpPoseVec.push_back(tcpPoseMat);
        }
    }
    mTcpPose = tcpPoseVec;
}
void WorkspaceCalibration::loadFileImagePoints(std::string fileLocation){

    std::ifstream ifs;
    ifs.open(fileLocation);
    if (ifs.fail())
    {
        std::cerr <<"[ Error ]: ifs.fail "<< fileLocation << " File not fund. /n"
                 <<"function: Detection::getCalibrationData" << std::endl;
        exit(1);
    }
    else
    {
        while(ifs){

            std::string trashString;
            char trashChar;
            float tempFloat;


            std::getline(ifs,trashString, '=');
            for (int index = 0; index < 1; ++index)
            {
                ifs >> tempFloat;
                mErrorRMS = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 9; ++index)
            {
                ifs >> tempFloat;
                mCameraMatrix.val[index] = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 5; ++index)
            {
                ifs >> tempFloat;
                mDistortionCoefficient[index] = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 2; ++index){

                ifs >> tempFloat;
                mCenterPoint[index] = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 2; ++index){

                ifs >> tempFloat;
                mDiaPoint[index] = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 3; ++index){

                ifs >> tempFloat;
                mRvec.val[index] = tempFloat;
                ifs >> trashChar;
            }

            std::getline(ifs,trashString, '[');
            for (int index = 0; index < 3; ++index){

                ifs >> tempFloat;
                //srcout<< tempFloat <<endl;
                mTvec.val[index] = tempFloat;
                //cout<< mTvec.val[index] <<endl;
                ifs >> trashChar;
            }
            ifs >> trashString;
        }
    }
}

cv::Point2f WorkspaceCalibration::vectorBetween2Points(cv::Point2f startPoint, cv::Point2f endPoint){

    cv::Point2f pointVector = {(endPoint.x - startPoint.x), (endPoint.y - startPoint.y)};

    return pointVector;
}

cv::Mat WorkspaceCalibration::getTransformationEndEffector2CameraHandEye(){

    std::vector<cv::Mat> R_gripper2baseRM;
    std::vector<cv::Mat> R_gripper2baseRV;
    std::vector<cv::Mat> T_gripper2base;
    std::vector<cv::Mat> R_target2camRM;
    std::vector<cv::Mat> R_target2camRV;
    std::vector<cv::Mat> T_target2cam;
    cv::Mat R_cam2gripper = (cv::Mat_<float>(3, 3));
    cv::Mat T_cam2gripper = (cv::Mat_<float>(3, 1));

    cv::Mat R_base2world = (cv::Mat_<float>(3, 3));
    cv::Mat T_base2world = (cv::Mat_<float>(3, 1));
    cv::Mat R_gripper2cam = (cv::Mat_<float>(3, 3));
    cv::Mat T_gripper2cam = (cv::Mat_<float>(3, 1));

    cv::Vec3f robotPose1Rvec{2.56833,-1.80919,-0.0107535};
    cv::Mat robotPose1RM;
    Rodrigues(robotPose1Rvec, robotPose1RM);
    R_gripper2baseRM.push_back(robotPose1RM);
    cv::Vec3f robotPose2Rvec{2.51982,-1.77544,0.0851194};
    cv::Mat robotPose2RM;
    Rodrigues(robotPose2Rvec, robotPose2RM);
    R_gripper2baseRM.push_back(robotPose2RM);
    cv::Vec3f robotPose3Rvec{2.45034,-1.72646,0.20952};
    cv::Mat robotPose3RM;
    Rodrigues(robotPose3Rvec, robotPose3RM);
    R_gripper2baseRM.push_back(robotPose3RM);
    cv::Vec3f robotPose4Rvec{2.48553,-1.75135,-0.167425};
    cv::Mat robotPose4RM;
    Rodrigues(robotPose4Rvec, robotPose4RM);
    R_gripper2baseRM.push_back(robotPose4RM);
    cv::Vec3f robotPose5Rvec{2.41831,-1.70364,-0.281098};
    cv::Mat robotPose5RM;
    Rodrigues(robotPose5Rvec, robotPose5RM);
    R_gripper2baseRM.push_back(robotPose5RM);
    cv::Vec3f robotPose6Rvec{2.42166,-1.73313,0.144227};
    cv::Mat robotPose6RM;
    Rodrigues(robotPose6Rvec, robotPose6RM);
    R_gripper2baseRM.push_back(robotPose6RM);
    cv::Vec3f robotPose7Rvec{2.39505,-1.67246,0.206131};
    cv::Mat robotPose7RM;
    Rodrigues(robotPose7Rvec, robotPose7RM);
    R_gripper2baseRM.push_back(robotPose7RM);
    cv::Vec3f robotPose8Rvec{2.51064,-1.69561,-0.120229};
    cv::Mat robotPose8RM;
    Rodrigues(robotPose8Rvec, robotPose8RM);
    R_gripper2baseRM.push_back(robotPose8RM);
    cv::Vec3f robotPose9Rvec{2.54838,-1.54705,-0.152988};
    cv::Mat robotPose9RM;
    Rodrigues(robotPose9Rvec, robotPose9RM);
    R_gripper2baseRM.push_back(robotPose9RM);
    cv::Vec3f robotPose10Rvec{2.59225,-1.74308,0.222385};
    cv::Mat robotPose10RM;
    Rodrigues(robotPose10Rvec, robotPose10RM);
    R_gripper2baseRM.push_back(robotPose10RM);
    cv::Vec3f robotPose11Rvec{2.55247,-1.75462,0.29355};
    cv::Mat robotPose11RM;
    Rodrigues(robotPose11Rvec, robotPose11RM);
    R_gripper2baseRM.push_back(robotPose11RM);
    cv::Vec3f robotPose12Rvec{2.35181,-1.80303,0.383283};
    cv::Mat robotPose12RM;
    Rodrigues(robotPose12Rvec, robotPose12RM);
    R_gripper2baseRM.push_back(robotPose12RM);
    cv::Vec3f robotPose13Rvec{2.36465,-1.72092,0.364604};
    cv::Mat robotPose13RM;
    Rodrigues(robotPose13Rvec, robotPose13RM);
    R_gripper2baseRM.push_back(robotPose13RM);
    cv::Vec3f robotPose14Rvec{2.27269,-1.77914,0.0350043};
    cv::Mat robotPose14RM;
    Rodrigues(robotPose14Rvec, robotPose14RM);
    R_gripper2baseRM.push_back(robotPose14RM);
    cv::Vec3f robotPose15Rvec{2.18543,-1.84727,0.013624};
    cv::Mat robotPose15RM;
    Rodrigues(robotPose15Rvec, robotPose15RM);
    R_gripper2baseRM.push_back(robotPose15RM);
    cv::Vec3f robotPose16Rvec{2.30029,-1.86181,-0.0816545};
    cv::Mat robotPose16RM;
    Rodrigues(robotPose16Rvec, robotPose16RM);
    R_gripper2baseRM.push_back(robotPose16RM);
    cv::Vec3f robotPose17Rvec{2.28458,-1.87354,-0.0856454};
    cv::Mat robotPose17RM;
    Rodrigues(robotPose17Rvec, robotPose17RM);
    R_gripper2baseRM.push_back(robotPose17RM);
    cv::Vec3f robotPose18Rvec{2.66713,-1.58452,-0.355344};
    cv::Mat robotPose18RM;
    Rodrigues(robotPose18Rvec, robotPose18RM);
    R_gripper2baseRM.push_back(robotPose18RM);
    cv::Vec3f robotPose19Rvec{2.73547,-1.46166,-0.378677};
    cv::Mat robotPose19RM;
    Rodrigues(robotPose19Rvec, robotPose19RM);
    R_gripper2baseRM.push_back(robotPose19RM);
    cv::Vec3f robotPose20Rvec{2.75107,-1.379,-0.324053};
    cv::Mat robotPose20RM;
    Rodrigues(robotPose20Rvec, robotPose20RM);
    R_gripper2baseRM.push_back(robotPose20RM);
    cv::Vec3f robotPose21RVec{2.66634,-1.49978,-0.256885};
    cv::Mat robotPose21RM;
    Rodrigues(robotPose21RVec, robotPose21RM);
    R_gripper2baseRM.push_back(robotPose21RM);
    cv::Vec3f robotPose22RVec{2.45641,-1.67622,0.0201043};
    cv::Mat robotPose22RM;
    Rodrigues(robotPose22RVec, robotPose22RM);
    R_gripper2baseRM.push_back(robotPose22RM);
    cv::Vec3f robotPose23RVec{2.51148,-1.57303,0.0569501};
    cv::Mat robotPose23RM;
    Rodrigues(robotPose23RVec, robotPose23RM);
    R_gripper2baseRM.push_back(robotPose23RM);
    cv::Vec3f robotPose24RVec{2.39316,-1.74172,0.224974};
    cv::Mat robotPose24RM;
    Rodrigues(robotPose24RVec, robotPose24RM);
    R_gripper2baseRM.push_back(robotPose24RM);
    cv::Vec3f robotPose25RVec{2.44358,-1.77428,0.115908};
    cv::Mat robotPose25RM;
    Rodrigues(robotPose25RVec, robotPose25RM);
    R_gripper2baseRM.push_back(robotPose25RM);


    const cv::Mat robotPose1TVec = (cv::Mat_<float>(3, 1) << -42.3306,-384.721,274.04);
    T_gripper2base.push_back(robotPose1TVec);
    const cv::Mat robotPose2TVec = (cv::Mat_<float>(3, 1) << -44.0763,-376.486,272.3);
    T_gripper2base.push_back(robotPose2TVec);
    const cv::Mat robotPose3TVec = (cv::Mat_<float>(3, 1) << -51.1709,-341.874,275.016);
    T_gripper2base.push_back(robotPose3TVec);
    const cv::Mat robotPose4TVec = (cv::Mat_<float>(3, 1) << -30.5068,-442.817,272.31);
    T_gripper2base.push_back(robotPose4TVec);
    const cv::Mat robotPose5TVec = (cv::Mat_<float>(3, 1) << -24.2314,-472.719,272.287);
    T_gripper2base.push_back(robotPose5TVec);
    const cv::Mat robotPose6TVec = (cv::Mat_<float>(3, 1) << -74.3224,-430.91,273.445);
    T_gripper2base.push_back(robotPose6TVec);
    const cv::Mat robotPose7TVec = (cv::Mat_<float>(3, 1) << -115.463,-443.106,272.297);
    T_gripper2base.push_back(robotPose7TVec);
    const cv::Mat robotPose8TVec = (cv::Mat_<float>(3, 1) << -2.79579,-386.027,272.305);
    T_gripper2base.push_back(robotPose8TVec);
    const cv::Mat robotPose9TVec = (cv::Mat_<float>(3, 1) << 21.6584,-380.281,272.286);
    T_gripper2base.push_back(robotPose9TVec);
    const cv::Mat robotPose10TVec = (cv::Mat_<float>(3, 1) << -87.9445,-385.063,272.306);
    T_gripper2base.push_back(robotPose10TVec);
    const cv::Mat robotPose11TVec = (cv::Mat_<float>(3, 1) << -124.332,-351.605,272.284);
    T_gripper2base.push_back(robotPose11TVec);
    const cv::Mat robotPose12TVec = (cv::Mat_<float>(3, 1) << -153.497,-371.857,272.288);
    T_gripper2base.push_back(robotPose12TVec);
    const cv::Mat robotPose13TVec = (cv::Mat_<float>(3, 1) << -112.68,-400.536,272.296);
    T_gripper2base.push_back(robotPose13TVec);
    const cv::Mat robotPose14TVec = (cv::Mat_<float>(3, 1) << -64.9959,-489.491,272.307);
    T_gripper2base.push_back(robotPose14TVec);
    const cv::Mat robotPose15TVec = (cv::Mat_<float>(3, 1) << -94.3519,-535.456,272.307);
    T_gripper2base.push_back(robotPose15TVec);
    const cv::Mat robotPose16TVec = (cv::Mat_<float>(3, 1) << -72.9502,-547.814,272.296);
    T_gripper2base.push_back(robotPose16TVec);
    const cv::Mat robotPose17TVec = (cv::Mat_<float>(3, 1) << -81.8275,-503.952,272.278);
    T_gripper2base.push_back(robotPose17TVec);
    const cv::Mat robotPose18TVec = (cv::Mat_<float>(3, 1) << 3.16342,-445.098,272.302);
    T_gripper2base.push_back(robotPose18TVec);
    const cv::Mat robotPose19TVec = (cv::Mat_<float>(3, 1) << 51.7385,-462.24,272.287);
    T_gripper2base.push_back(robotPose19TVec);
    const cv::Mat robotPose20TVec = (cv::Mat_<float>(3, 1) << 59.7246,-425.831,272.288);
    T_gripper2base.push_back(robotPose20TVec);
    const cv::Mat robotPose21TVec = (cv::Mat_<float>(3, 1) << 26.4408,-442.845,272.275);
    T_gripper2base.push_back(robotPose21TVec);
    const cv::Mat robotPose22TVec = (cv::Mat_<float>(3, 1) << -18.5598,-352.518,272.304);
    T_gripper2base.push_back(robotPose22TVec);
    const cv::Mat robotPose23TVec = (cv::Mat_<float>(3, 1) << -4.85337,-326.565,272.296);
    T_gripper2base.push_back(robotPose23TVec);
    const cv::Mat robotPose24TVec = (cv::Mat_<float>(3, 1) << -38.0401,-334.654,272.299);
    T_gripper2base.push_back(robotPose24TVec);
    const cv::Mat robotPose25TVec = (cv::Mat_<float>(3, 1) << -25.5127,-372.064,272.318);
    T_gripper2base.push_back(robotPose25TVec);


    cv::Vec3f camImgPose1RVec{2.11789710722773, 1.963930777925548, 0.2491911990606635};
    cv::Mat camImgPose1RM;
    Rodrigues(camImgPose1RVec, camImgPose1RM);
    R_target2camRM.push_back(camImgPose1RM);
    cv::Vec3f camImgPose2RVec{-2.166339628800446, -2.075565678133577, -0.2548031186570963};
    cv::Mat camImgPose2RM;
    Rodrigues(camImgPose2RVec, camImgPose2RM);
    R_target2camRM.push_back(camImgPose2RM);
    cv::Vec3f camImgPose3RVec{-2.177951649780516, -2.082447029622186, -0.1169292205109279};
    cv::Mat camImgPose3RM;
    Rodrigues(camImgPose3RVec, camImgPose3RM);
    R_target2camRM.push_back(camImgPose3RM);
    cv::Vec3f camImgPose4RVec{2.13698154947643, 1.993872349360031, 0.3625049998384356};
    cv::Mat camImgPose4RM;
    Rodrigues(camImgPose4RVec, camImgPose4RM);
    R_target2camRM.push_back(camImgPose4RM);
    cv::Vec3f camImgPose5RVec{2.098694327410085, 1.954361159678099, 0.4877274103626605};
    cv::Mat camImgPose5RM;
    Rodrigues(camImgPose5RVec, camImgPose5RM);
    R_target2camRM.push_back(camImgPose5RM);
    cv::Vec3f camImgPose6RVec{-2.153599992742323, -2.059269697207191, -0.3991235295861412};
    cv::Mat camImgPose6RM;
    Rodrigues(camImgPose6RVec, camImgPose6RM);
    R_target2camRM.push_back(camImgPose6RM);
    cv::Vec3f camImgPose7RVec{-2.085260107273982, -2.05296201799239, -0.5314222882790234};
    cv::Mat camImgPose7RM;
    Rodrigues(camImgPose7RVec, camImgPose7RM);
    R_target2camRM.push_back(camImgPose7RM);
    cv::Vec3f camImgPose8RVec{2.102857224440608, 2.032456324521735, 0.161508888522923};
    cv::Mat camImgPose8RM;
    Rodrigues(camImgPose8RVec, camImgPose8RM);
    R_target2camRM.push_back(camImgPose8RM);
    cv::Vec3f camImgPose9RVec{1.997499846878869, 2.115249813160311, 0.1041176540023391};
    cv::Mat camImgPose9RM;
    Rodrigues(camImgPose9RVec, camImgPose9RM);
    R_target2camRM.push_back(camImgPose9RM);
    cv::Vec3f camImgPose10Vec{-2.105650729102397, -2.115490026704616, -0.2215221576677011};
    cv::Mat camImgPose10RM;
    Rodrigues(camImgPose10Vec, camImgPose10RM);
    R_target2camRM.push_back(camImgPose10RM);
    cv::Vec3f camImgPose11RVec{-2.046273052064703, -2.043143706725822, -0.3240162018646171};
    cv::Mat camImgPose11RM;
    Rodrigues(camImgPose11RVec, camImgPose11RM);
    R_target2camRM.push_back(camImgPose11RM);
    cv::Vec3f camImgPose12RVec{-2.053958887857421, -1.877536843938971, -0.4381418110068137};
    cv::Mat camImgPose12RM;
    Rodrigues(camImgPose12RVec, camImgPose12RM);
    R_target2camRM.push_back(camImgPose12RM);
    cv::Vec3f camImgPose13RVec{-2.03306929137464, -1.924833897782089, -0.3661307718322906};
    cv::Mat camImgPose13RM;
    Rodrigues(camImgPose13RVec, camImgPose13RM);
    R_target2camRM.push_back(camImgPose13RM);
    cv::Vec3f camImgPose14RVec{2.327068631035941, 1.994965364555241, 0.5912224495312491};
    cv::Mat camImgPose14RM;
    Rodrigues(camImgPose14RVec, camImgPose14RM);
    R_target2camRM.push_back(camImgPose14RM);
    cv::Vec3f camImgPose15RVec{-2.377236107097835, -1.893532618001131, -0.7261297968050794};
    cv::Mat camImgPose15RM;
    Rodrigues(camImgPose15RVec, camImgPose15RM);
    R_target2camRM.push_back(camImgPose15RM);
    cv::Vec3f camImgPose16RVec{2.327975076625156, 1.918617792039849, 0.6049939562272411};
    cv::Mat camImgPose16RM;
    Rodrigues(camImgPose16RVec, camImgPose16RM);
    R_target2camRM.push_back(camImgPose16RM);
    cv::Vec3f camImgPose17RVec{2.345075515305476, 1.93442617117904, 0.6112679297869027};
    cv::Mat camImgPose17RM;
    Rodrigues(camImgPose17RVec, camImgPose17RM);
    R_target2camRM.push_back(camImgPose17RM);
    cv::Vec3f camImgPose18RVec{1.92145497549634, 2.076298080342776, 0.2807811768063092};
    cv::Mat camImgPose18RM;
    Rodrigues(camImgPose18RVec, camImgPose18RM);
    R_target2camRM.push_back(camImgPose18RM);
    cv::Vec3f camImgPose19RVec{1.808447689122425, 2.124037999419507, 0.3063069482437351};
    cv::Mat camImgPose19RM;
    Rodrigues(camImgPose19RVec, camImgPose19RM);
    R_target2camRM.push_back(camImgPose19RM);
    cv::Vec3f camImgPose20RVec{1.760437876741588, 2.177971868536694, 0.255860450556706};
    cv::Mat camImgPose20RM;
    Rodrigues(camImgPose20RVec, camImgPose20RM);
    R_target2camRM.push_back(camImgPose20RM);
    cv::Vec3f camImgPose21RVec{1.900903408685813, 2.148852002690958, 0.2248156159481186};
    cv::Mat camImgPose21RM;
    Rodrigues(camImgPose21RVec, camImgPose21RM);
    R_target2camRM.push_back(camImgPose21RM);
    cv::Vec3f camImgPose22RVec{2.1737899500999, 2.0989183724327, 0.134661246952555};
    cv::Mat camImgPose22RM;
    Rodrigues(camImgPose22RVec, camImgPose22RM);
    R_target2camRM.push_back(camImgPose22RM);
    cv::Vec3f camImgPose23RVec{2.093531266194179, 2.173850598659756, 0.1886039657112569};
    cv::Mat camImgPose23RM;
    Rodrigues(camImgPose23RVec, camImgPose23RM);
    R_target2camRM.push_back(camImgPose23RM);
    cv::Vec3f camImgPose24RVec{-2.214968533966443, -2.048198067153262, -0.05616405839698649};
    cv::Mat camImgPose24RM;
    Rodrigues(camImgPose24RVec, camImgPose24RM);
    R_target2camRM.push_back(camImgPose24RM);
    cv::Vec3f camImgPose25RVec{-2.225545454246284, -2.056649782260117, -0.1834836664737774};
    cv::Mat camImgPose25RM;
    Rodrigues(camImgPose25RVec, camImgPose25RM);
    R_target2camRM.push_back(camImgPose25RM);


    const cv::Mat camImgPose1TVec = (cv::Mat_<float>(3, 1) << -11.70876967160552, -7.829572179824827, 54.37052346728687);
    T_target2cam.push_back(camImgPose1TVec);
    const cv::Mat camImgPose2TVec = (cv::Mat_<float>(3, 1) << -10.16583044691266, -7.686569002717127, 55.60951578434334);
    T_target2cam.push_back(camImgPose2TVec);
    const cv::Mat camImgPose3TVec = (cv::Mat_<float>(3, 1) << -10.27770071994081, -7.849102415334118, 57.40234512349403);
    T_target2cam.push_back(camImgPose3TVec);
    const cv::Mat camImgPose4TVec = (cv::Mat_<float>(3, 1) << -10.18722783836936, -7.653258692478801, 55.04074856119941);
    T_target2cam.push_back(camImgPose4TVec);
    const cv::Mat camImgPose5TVec = (cv::Mat_<float>(3, 1) << -10.49289832467876, -7.729019434273354, 55.27507639859815);
    T_target2cam.push_back(camImgPose5TVec);
    const cv::Mat camImgPose6TVec = (cv::Mat_<float>(3, 1) << -9.019387308739542, -9.143024920986955, 54.90885486869285);
    T_target2cam.push_back(camImgPose6TVec);
    const cv::Mat camImgPose7TVec = (cv::Mat_<float>(3, 1) << -9.022949264122213, -7.063669692499651, 55.47717586157179);
    T_target2cam.push_back(camImgPose7TVec);
    const cv::Mat camImgPose8TVec = (cv::Mat_<float>(3, 1) << -10.11794863170466, -7.764816239677401, 56.53766807229312);
    T_target2cam.push_back(camImgPose8TVec);
    const cv::Mat camImgPose9TVec = (cv::Mat_<float>(3, 1) << -8.702354756724786, -8.892324685061537, 57.82856689578553);
    T_target2cam.push_back(camImgPose9TVec);
    const cv::Mat camImgPose10TVec = (cv::Mat_<float>(3, 1) << -9.326074575618733, -6.263078395970324, 55.90857865294191);
    T_target2cam.push_back(camImgPose10TVec);
    const cv::Mat camImgPose11TVec = (cv::Mat_<float>(3, 1) << -13.18438850400979, -5.823821606858634, 55.09489891698632);
    T_target2cam.push_back(camImgPose11TVec);
    const cv::Mat camImgPose12TVec = (cv::Mat_<float>(3, 1) << -13.10686825460134, -5.960064302413323, 55.30193224508586);
    T_target2cam.push_back(camImgPose12TVec);
    const cv::Mat camImgPose13TVec = (cv::Mat_<float>(3, 1) << -9.811632849636331, -9.670825714291315, 53.74005235941605);
    T_target2cam.push_back(camImgPose13TVec);
    const cv::Mat camImgPose14TVec = (cv::Mat_<float>(3, 1) << -6.93855789155013, -9.310631244304943, 55.42627366044081);
    T_target2cam.push_back(camImgPose14TVec);
    const cv::Mat camImgPose15TVec = (cv::Mat_<float>(3, 1) << -4.463664669343057, -6.363258073800536, 57.44938636013186);
    T_target2cam.push_back(camImgPose15TVec);
    const cv::Mat camImgPose16TVec = (cv::Mat_<float>(3, 1) << -2.493467491086608, -4.48280304882167, 58.33251528984567);
    T_target2cam.push_back(camImgPose16TVec);
    const cv::Mat camImgPose17TVec = (cv::Mat_<float>(3, 1) << -7.061093608922834, -4.516613217227101, 57.15669480676578);
    T_target2cam.push_back(camImgPose17TVec);
    const cv::Mat camImgPose18TVec = (cv::Mat_<float>(3, 1) << -9.048658658963868, -5.927689289016797, 57.17800733597301);
    T_target2cam.push_back(camImgPose18TVec);
    const cv::Mat camImgPose19TVec = (cv::Mat_<float>(3, 1) << -5.220852302773912, -8.83474716849093, 59.08998003844606);
    T_target2cam.push_back(camImgPose19TVec);
    const cv::Mat camImgPose20TVec = (cv::Mat_<float>(3, 1) << -5.760064012146566, -10.91139725878062, 58.46067661580957);
    T_target2cam.push_back(camImgPose20TVec);
    const cv::Mat camImgPose21TVec = (cv::Mat_<float>(3, 1) << -5.483443662368891, -7.540347906420092, 58.24277247418063);
    T_target2cam.push_back(camImgPose21TVec);
    const cv::Mat camImgPose22TVec = (cv::Mat_<float>(3, 1) << -10.60723841142233, -7.677149035693343, 57.33132615701239);
    T_target2cam.push_back(camImgPose22TVec);
    const cv::Mat camImgPose23TVec = (cv::Mat_<float>(3, 1) << -10.89629036648524, -10.55833191929478, 57.42054730019866);
    T_target2cam.push_back(camImgPose23TVec);
    const cv::Mat camImgPose24TVec = (cv::Mat_<float>(3, 1) << -9.860811776222514, -8.432809674566652, 57.66163523074459);
    T_target2cam.push_back(camImgPose24TVec);
    const cv::Mat camImgPose25TVec = (cv::Mat_<float>(3, 1) << -8.706838987158902, -8.381709848234477, 56.64795442574479);
    T_target2cam.push_back(camImgPose25TVec);



    calibrateHandEye(R_gripper2baseRM, T_gripper2base, R_target2camRM, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_ANDREFF);
//CALIB_HAND_EYE_TSAI, CALIB_HAND_EYE_PARK, CALIB_HAND_EYE_HORAUD, CALIB_HAND_EYE_ANDREFF, CALIB_HAND_EYE_DANIILIDIS ////// Andreff


    cv::Mat TEECAMHANDEYE = (cv::Mat_<double>(4, 4) <<
            R_cam2gripper.at<double>(0,0), R_cam2gripper.at<double>(0,1), R_cam2gripper.at<double>(0,2), T_cam2gripper.at<double>(0,0)/1000,
            R_cam2gripper.at<double>(1,0), R_cam2gripper.at<double>(1,1), R_cam2gripper.at<double>(1,2), T_cam2gripper.at<double>(0,1)/1000,
            R_cam2gripper.at<double>(2,0), R_cam2gripper.at<double>(2,1), R_cam2gripper.at<double>(2,2), T_cam2gripper.at<double>(0,2)/1000,
            0, 0, 0, 1);


    return TEECAMHANDEYE;

}

//std::vector<double> WorkspaceCalibration::targetPointTransform(std::vector<double> startPoint, std::vector<double> targetPoint){

//    cv::Mat startPointRvec = (cv::Mat_<double>(3,1) << startPoint[3], startPoint[4], startPoint[5]);
//    cv::Mat startPointRM;
//    cv::Mat startPointTvec = (cv::Mat_<double>(3,1) << startPoint[0], startPoint[1], startPoint[2]);

//    Rodrigues(startPointRvec, startPointRM);

//    cv::Mat startPointTransform = (cv::Mat_<double>(4, 4) <<
//            startPointRM.at<double>(0,0), startPointRM.at<double>(0,1), startPointRM.at<double>(0,2), startPointTvec.at<double>(0,0),
//            startPointRM.at<double>(1,0), startPointRM.at<double>(1,1), startPointRM.at<double>(1,2), startPointTvec.at<double>(0,1),
//            startPointRM.at<double>(2,0), startPointRM.at<double>(2,1), startPointRM.at<double>(2,2), startPointTvec.at<double>(0,2),
//            0, 0, 0, 1);

//    cv::Mat targetPointRvec = (cv::Mat_<double>(3,1) << targetPoint[3], targetPoint[4], targetPoint[5]);
//    cv::Mat targetPointRM;
//    cv::Mat targetPointTvec = (cv::Mat_<double>(3,1) << targetPoint[0], targetPoint[1], targetPoint[2]);

//    Rodrigues(targetPointRvec, targetPointRM);

//    cv::Mat targetTransform = (cv::Mat_<double>(4, 4) <<
//            targetPointRM.at<double>(0,0), targetPointRM.at<double>(0,1), targetPointRM.at<double>(0,2), targetPointTvec.at<double>(0,0),
//            targetPointRM.at<double>(1,0), targetPointRM.at<double>(1,1), targetPointRM.at<double>(1,2), targetPointTvec.at<double>(0,1),
//            targetPointRM.at<double>(2,0), targetPointRM.at<double>(2,1), targetPointRM.at<double>(2,2), targetPointTvec.at<double>(0,2),
//            0, 0, 0, 1);

//    cv::Mat endTransform = startPointTransform.inv() * targetTransform;

//    cv::Mat endPointRM = (cv::Mat_<double>(3,3) <<
//                      endTransform.at<double>(0,0), endTransform.at<double>(0,1), endTransform.at<double>(0,2),
//                      endTransform.at<double>(1,0), endTransform.at<double>(1,1), endTransform.at<double>(1,2),
//                      endTransform.at<double>(2,0), endTransform.at<double>(2,1), endTransform.at<double>(2,2));

//    cv::Mat endPointRVec;

//    Rodrigues(endPointRM, endPointRVec);

//    cv::Mat endPointTvec = (cv::Mat_<double>(3,1) << endTransform.at<double>(0,3), endTransform.at<double>(1,3), endTransform.at<double>(2,3));

//    std::vector<double> endPoint = {endPointTvec.at<double>(0,0), endPointTvec.at<double>(0,1), endPointTvec.at<double>(0,2), endPointRVec.at<double>(0,0), endPointRVec.at<double>(0,1), endPointRVec.at<double>(0,2)};

//    return endPoint;
//}
