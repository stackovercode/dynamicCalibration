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

//    cv::Mat TEECAM = (cv::Mat_<double>(4, 4) <<
//        1, 0, 0, 0.0261,
//        0, 1, 0, 0.0347,
//        0, 0, 1, -0.139,
//        0, 0, 0, 1);

    return TEECAM;
}

cv::Mat WorkspaceCalibration::getTransformationCamera2EndEffector(int numbOfPose, int method){

    cv::Mat TTFCAM = getTransformationFlange2CameraHandEye(numbOfPose, method);

    cv::Mat TCAMEE = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, -TTFCAM.at<double>(0,3),
        0, 1, 0, -TTFCAM.at<double>(1,3),
        0, 0, 1,  (0.18 + TTFCAM.at<double>(2,3)),
        0, 0, 0, 1);

    return TCAMEE;
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

    //vector<Mat> mTcpPose;
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
    //return mTcpPose;
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

cv::Mat WorkspaceCalibration::eulerAnglesToRotationMatrix(cv::Vec3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(theta[0]), -sin(theta[0]),
        0, sin(theta[0]), cos(theta[0])
        );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
        cos(theta[1]), 0, sin(theta[1]),
        0, 1, 0,
        -sin(theta[1]), 0, cos(theta[1])
        );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
        cos(theta[2]), -sin(theta[2]), 0,
        sin(theta[2]), cos(theta[2]), 0,
        0, 0, 1);


    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;

}

cv::Mat WorkspaceCalibration::rvec2RotationMatrix(cv::Vec3f theta){

    float angle = sqrt(pow(theta[0], 2) + pow(theta[1], 2) + pow(theta[2], 2));
    cv::Vec3f unitVector{theta[0]/angle, theta[1]/angle, theta[2]/angle};

    float c = cos(angle);
    float s = sin(angle);
    float C = 1 - sin(angle);


    cv::Mat R = (cv::Mat_<float>(3, 3) <<
                 unitVector[0]*unitVector[0]*C+c, unitVector[0]*unitVector[1]*C-unitVector[2]*s, unitVector[0]*unitVector[2]*C+unitVector[1]*s,
                 unitVector[1]*unitVector[0]*C+unitVector[2]*s, unitVector[1]*unitVector[1]*C+c, unitVector[1]*unitVector[2]*C-unitVector[0]*s,
                 unitVector[2]*unitVector[0]*C-unitVector[1]*s, unitVector[2]*unitVector[1]*C+unitVector[0]*s, unitVector[2]*unitVector[2]*C+c);

    return R;


}

cv::Mat WorkspaceCalibration::getInversMatrix(cv::Mat matrix){

    cv::Mat rotationMatrix = (cv::Mat_<double>(3, 3) <<
                              matrix.at<double>(0,0), matrix.at<double>(0,1), matrix.at<double>(0,2),
                              matrix.at<double>(1,0), matrix.at<double>(1,1), matrix.at<double>(1,2),
                              matrix.at<double>(2,0), matrix.at<double>(2,1), matrix.at<double>(2,2));


    cv::Mat rotationMatrixTrans = rotationMatrix.inv();
    //cv::Mat rotationMatrixTrans;
    //transpose(rotationMatrix, rotationMatrixTrans);

    cv::Mat translationMatrix = (cv::Mat_<double>(3, 1) << matrix.at<double>(0,3), matrix.at<double>(1,3), matrix.at<double>(2,3));

    cv::Mat translationMatrixTrans = -rotationMatrixTrans * translationMatrix;

    cv::Mat inversMatrix = (cv::Mat_<double>(4, 4) <<
                           rotationMatrixTrans.at<double>(0,0), rotationMatrixTrans.at<double>(0,1), rotationMatrixTrans.at<double>(0,2), translationMatrixTrans.at<double>(0,0),
                           rotationMatrixTrans.at<double>(1,0), rotationMatrixTrans.at<double>(1,1), rotationMatrixTrans.at<double>(1,2), translationMatrixTrans.at<double>(0,1),
                           rotationMatrixTrans.at<double>(2,0), rotationMatrixTrans.at<double>(2,1), rotationMatrixTrans.at<double>(2,2), translationMatrixTrans.at<double>(0,2),
                           0, 0, 0, 1);
    return inversMatrix;

}

cv::Mat WorkspaceCalibration::getTransformationFlange2CameraHandEye(int numbOfPose, int method){

    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> T_gripper2base;
    std::vector<cv::Mat> R_base2gripper;
    std::vector<cv::Mat> T_base2gripper;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> T_target2cam;
    cv::Mat R_cam2gripper = (cv::Mat_<float>(3, 3));
    cv::Mat T_cam2gripper = (cv::Mat_<float>(3, 1));


    cv::Mat R_base2world = (cv::Mat_<float>(3, 3));
    cv::Mat T_base2world = (cv::Mat_<float>(3, 1));
    cv::Mat R_gripper2cam = (cv::Mat_<float>(3, 3));
    cv::Mat T_gripper2cam = (cv::Mat_<float>(3, 1));

    //Load TCP poses from file
    std::string fileLocationRobotTcp = ("../Detection/RobotposeData.txt");

    loadFileRobotTCP(fileLocationRobotTcp);

    //Load Rvec and Tvec from camera from file
    std::string fileLocation = ("../Detection/MarkertransposeData.txt");

    loadFileTranRot(fileLocation);




//    for(size_t i = 0; i < mTcpPose.size()-1;i++){
////        float rv_len = sqrt(pow(mTcpPose[i].at<float>(0,3), 2) + pow(mTcpPose[i].at<float>(0,4), 2) + pow(mTcpPose[i].at<float>(0,5), 2));
////        float scale = 1 - 2 * M_PI / rv_len;
////        cv::Vec3f robotPoseRVec{mTcpPose[i].at<float>(0,3) * scale,mTcpPose[i].at<float>(0,4) * scale,mTcpPose[i].at<float>(0,5) * scale};
////        cv::Vec3f robotPoseRVec{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
////        cv::Mat robotPoseRM;
////        Rodrigues(robotPoseRVec, robotPoseRM);
////        R_gripper2baseRM.push_back(robotPoseRM);
//        cv::Vec3f theta{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
//        float factor = sqrt(pow(theta[0], 2) + pow(theta[1], 2) + pow(theta[2], 2));
//        cv::Vec3f robotUnitVector{theta[0]/factor, theta[1]/factor, theta[2]/factor};
//        cv::Mat robotRm = eulerAnglesToRotationMatrix(robotUnitVector);
//        R_gripper2base.push_back(robotRm);
//    }


    for(int i = 0; i < numbOfPose; i++){
        cv::Vec3f theta{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
        float factor = sqrt(pow(theta[0], 2) + pow(theta[1], 2) + pow(theta[2], 2));
        cv::Vec3f robotUnitVector{theta[0]/factor, theta[1]/factor, theta[2]/factor};
        //float factor = 1/3;
       // cv::Vec3f robotUnitVector{theta[0]*factor, theta[1]*factor, theta[2]*factor};

        cv::Mat robotRm = rvec2RotationMatrix(robotUnitVector);
        //cv::Mat robotPoseRVec = (cv::Mat_<float>(3, 1) << mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5));
//        cv::Mat robotRm;
//        Rodrigues(robotPoseRVec, robotRm);

        const cv::Mat robotPoseTVec = (cv::Mat_<float>(3, 1) << mTcpPose[i].at<float>(0,0),mTcpPose[i].at<float>(0,1),mTcpPose[i].at<float>(0,2));

        cv::Mat Trans_robotMatrix = (cv::Mat_<double>(4, 4) <<
                robotRm.at<double>(0,0), robotRm.at<double>(0,1), robotRm.at<double>(0,2), robotPoseTVec.at<double>(0,0),
                robotRm.at<double>(1,0), robotRm.at<double>(1,1), robotRm.at<double>(1,2), robotPoseTVec.at<double>(0,1),
                robotRm.at<double>(2,0), robotRm.at<double>(2,1), robotRm.at<double>(2,2), robotPoseTVec.at<double>(0,2),
                0, 0, 0, 1);

        cv::Mat inverseRobotMatrix = getInversMatrix(Trans_robotMatrix);

        cv::Mat inverseRotationMatrix = (cv::Mat_<double>(3, 3) <<
                 inverseRobotMatrix.at<double>(0,0), inverseRobotMatrix.at<double>(0,1), inverseRobotMatrix.at<double>(0,2),
                 inverseRobotMatrix.at<double>(1,0), inverseRobotMatrix.at<double>(1,1), inverseRobotMatrix.at<double>(1,2),
                 inverseRobotMatrix.at<double>(2,0), inverseRobotMatrix.at<double>(2,1), inverseRobotMatrix.at<double>(2,2));

        cv::Mat inverseTranslationMatrix = (cv::Mat_<float>(3, 1) << inverseRobotMatrix.at<float>(0,3),inverseRobotMatrix.at<float>(1,3),inverseRobotMatrix.at<float>(2,3));

        R_gripper2base.push_back(inverseRotationMatrix);
        T_gripper2base.push_back(inverseTranslationMatrix);
    }

    for(int i = 0; i < numbOfPose; i++){
        cv::Vec3f theta{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
//        float factor = sqrt(pow(theta[0], 2) + pow(theta[1], 2) + pow(theta[2], 2));
//        cv::Vec3f robotUnitVector{theta[0]/factor, theta[1]/factor, theta[2]/factor};
//        cv::Mat robotRm = eulerAnglesToRotationMatrix(robotUnitVector);
        cv::Mat robotRm;
        Rodrigues(theta,robotRm);
        R_base2gripper.push_back(robotRm);
    }

//    for(size_t i = 0; i < R_gripper2baseRM.size(); i++){
//        std::cout<< "Test af R_gripper2baseRM " << i << ": " << R_gripper2baseRM[i] <<std::endl;
//    }

//    for(size_t i = 0; i < mTcpPose.size()-1; i++){
//        const cv::Mat robotPoseTVec = (cv::Mat_<float>(3, 1) << mTcpPose[i].at<float>(0,0)*1000,mTcpPose[i].at<float>(0,1)*1000,mTcpPose[i].at<float>(0,2)*1000);
//        T_gripper2base.push_back(robotPoseTVec);
//    }
    for(int i = 0; i < numbOfPose; i++){
        const cv::Mat robotPoseTVec = (cv::Mat_<float>(3, 1) << mTcpPose[i].at<float>(0,0),mTcpPose[i].at<float>(0,1),mTcpPose[i].at<float>(0,2));
        T_base2gripper.push_back(robotPoseTVec);
    }

//    for(size_t i = 0; i < T_gripper2base.size(); i++){
//        std::cout<< "Test af robotPoseTVec " << i << ": " << T_gripper2base[i] <<std::endl;
//    }


    for(int i = 0; i < numbOfPose; i++){
        cv::Mat camImgPoseRVec = (cv::Mat_<double>(3, 1) << mR_target2cam[i].at<double>(0,0), mR_target2cam[i].at<double>(0,1), mR_target2cam[i].at<double>(0,2));
        cv::Mat camImgPoseRM;
        Rodrigues(camImgPoseRVec, camImgPoseRM);
        R_target2cam.push_back(camImgPoseRM);
//        R_target2camRM.push_back(eulerAnglesToRotationMatrix(camImgPoseRVec));
    }

//    for(size_t i = 0; i < R_target2camRM.size(); i++){
//        std::cout<< "Test af R_target2cam " << i << ": " << R_target2camRM[i] <<std::endl;
//    }

    for(int i = 0; i < numbOfPose; i++){
        const cv::Mat camImgPose = (cv::Mat_<double>(3, 1) << mT_target2cam[i].at<double>(0,0), mT_target2cam[i].at<double>(0,1), mT_target2cam[i].at<double>(0,2));
        T_target2cam.push_back(camImgPose);
    }

//    for(size_t i = 0; i < T_target2cam.size(); i++){
//        std::cout<< "Test af T_target2cam " << i << ": " << T_target2cam[i] <<std::endl;
//    }



//    calibrateHandEye(R_gripper2baseRM, T_gripper2base, R_target2camRM, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_ANDREFF);
    //CALIB_HAND_EYE_TSAI, CALIB_HAND_EYE_PARK, CALIB_HAND_EYE_HORAUD, CALIB_HAND_EYE_ANDREFF, CALIB_HAND_EYE_DANIILIDIS ////// Andreff

//            std::cout<< "Er noget her til 2!" <<std::endl;
//            std::cout<< "Robot rotation matrix: " << "\n" << R_base2gripper[0] <<std::endl;
//            std::cout<< "Robot translation matrix: " << "\n" << T_base2gripper[0] <<std::endl;
//            std::cout<< "Robot rotation matrix inverse: " << "\n" << R_gripper2base[0] <<std::endl;
//            std::cout<< "Robot translation matri inverse: x" << "\n" << T_gripper2base[0] <<std::endl;

//    if(method == 0){
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
//    } else if (method == 1) {
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_PARK);
//    }else if (method == 2) {
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_HORAUD);
//    }else if (method == 3) {
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_ANDREFF);
//    }else if (method == 4) {
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_DANIILIDIS);
//    }else {
//        calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
//    }

    if(method == 0){
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
    } else if (method == 1) {
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_PARK);
    }else if (method == 2) {
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_HORAUD);
    }else if (method == 3) {
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_ANDREFF);
    }else if (method == 4) {
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_DANIILIDIS);
    }else {
        calibrateHandEye(R_base2gripper, T_base2gripper, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
    }



//    calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_PARK);// cv::CALIB_HAND_EYE_ANDREFF);
//    std::cout << R_cam2gripper << std::endl;

    cv::calibrateRobotWorldHandEye(R_target2cam,T_target2cam,R_base2gripper,T_base2gripper, R_base2world, T_base2world, R_gripper2cam, T_gripper2cam,cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);

    cv::Mat TRANS_calibrateRobotWorldHandEye = (cv::Mat_<double>(4, 4) <<
            R_gripper2cam.at<double>(0,0), R_gripper2cam.at<double>(0,1), R_gripper2cam.at<double>(0,2), T_gripper2cam.at<double>(0,0),
            R_gripper2cam.at<double>(1,0), R_gripper2cam.at<double>(1,1), R_gripper2cam.at<double>(1,2), T_gripper2cam.at<double>(0,1),
            R_gripper2cam.at<double>(2,0), R_gripper2cam.at<double>(2,1), R_gripper2cam.at<double>(2,2), T_gripper2cam.at<double>(0,2),
            0, 0, 0, 1);

    cv::Mat TRANS_calibrateRobotWorldHandEyeBase2World = (cv::Mat_<double>(4, 4) <<
            R_base2world.at<double>(0,0), R_base2world.at<double>(0,1), R_base2world.at<double>(0,2), T_base2world.at<double>(0,0),
            R_base2world.at<double>(1,0), R_base2world.at<double>(1,1), R_base2world.at<double>(1,2), T_base2world.at<double>(0,1),
            R_base2world.at<double>(2,0), R_base2world.at<double>(2,1), R_base2world.at<double>(2,2), T_base2world.at<double>(0,2),
            0, 0, 0, 1);

//    std::cout<< "RobotWorld gripper to camera: " << "\n" << TRANS_calibrateRobotWorldHandEye <<std::endl;
//    std::cout<< "RobotWorld base to world: " << "\n" << TRANS_calibrateRobotWorldHandEyeBase2World <<std::endl;




    cv::Mat TEECAMHANDEYE_ORIGINAL = (cv::Mat_<double>(4, 4) <<
            R_cam2gripper.at<double>(0,0), R_cam2gripper.at<double>(0,1), R_cam2gripper.at<double>(0,2), T_cam2gripper.at<double>(0,0),
            R_cam2gripper.at<double>(1,0), R_cam2gripper.at<double>(1,1), R_cam2gripper.at<double>(1,2), T_cam2gripper.at<double>(0,1),
            R_cam2gripper.at<double>(2,0), R_cam2gripper.at<double>(2,1), R_cam2gripper.at<double>(2,2), T_cam2gripper.at<double>(0,2),
            0, 0, 0, 1);

    cv::Mat TEECAMHANDEYE_TEST = (cv::Mat_<double>(4, 4) <<
            0, 0, 1, -T_cam2gripper.at<double>(0,0)/1000,
            0, 1, 0, T_cam2gripper.at<double>(0,1)/1000,
            1, 0, 0, -T_cam2gripper.at<double>(0,2)/1000,
            0, 0, 0, 1);


    return TEECAMHANDEYE_ORIGINAL;

}

vpHomogeneousMatrix WorkspaceCalibration::vispHandEyeCalibration(){


    std::vector<vpHomogeneousMatrix> cMo;
    std::vector<vpHomogeneousMatrix> rMe;
    vpHomogeneousMatrix eMc;


    //Load TCP poses from file
    std::string fileLocationRobotTcp = ("../Detection/RobotposeData.txt");

    loadFileRobotTCP(fileLocationRobotTcp);

    //Load Rvec and Tvec from camera from file
    std::string fileLocation = ("../Detection/MarkertransposeData.txt");

    loadFileTranRot(fileLocation);

    for(size_t i = 0; i < mTcpPose.size()-1;i++){


        cv::Vec3f robotPoseRVec{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
//        float factor = sqrt(pow(robotPoseRVec[0], 2) + pow(robotPoseRVec[1], 2) + pow(robotPoseRVec[2], 2));
//        cv::Vec3f robotUnitVector{robotPoseRVec[0]/factor, robotPoseRVec[1]/factor, robotPoseRVec[2]/factor};
//        cv::Mat robotPoseRM = rvec2RotationMatrix(robotUnitVector);

        const cv::Mat robotPoseTVec = (cv::Mat_<float>(3, 1) << mTcpPose[i].at<float>(0,0)*1000,mTcpPose[i].at<float>(0,1)*1000,mTcpPose[i].at<float>(0,2)*1000);

//        cv::Mat Trans_robotMatrix = (cv::Mat_<double>(4, 4) <<
//                robotPoseRM.at<double>(0,0), robotPoseRM.at<double>(0,1), robotPoseRM.at<double>(0,2), robotPoseTVec.at<double>(0,0),
//                robotPoseRM.at<double>(1,0), robotPoseRM.at<double>(1,1), robotPoseRM.at<double>(1,2), robotPoseTVec.at<double>(0,1),
//                robotPoseRM.at<double>(2,0), robotPoseRM.at<double>(2,1), robotPoseRM.at<double>(2,2), robotPoseTVec.at<double>(0,2),
//                0, 0, 0, 1);



//        cv::Mat inverseRobotMatrix = getInversMatrix(Trans_robotMatrix);

//        cv::Mat inverseRotationMatrix = (cv::Mat_<double>(3, 3) <<
//                inverseRobotMatrix.at<double>(0,0), inverseRobotMatrix.at<double>(0,1), inverseRobotMatrix.at<double>(0,2),
//                inverseRobotMatrix.at<double>(1,0), inverseRobotMatrix.at<double>(1,1), inverseRobotMatrix.at<double>(1,2),
//                inverseRobotMatrix.at<double>(2,0), inverseRobotMatrix.at<double>(2,1), inverseRobotMatrix.at<double>(2,2));

//        cv::Mat inverseTranslationMatrix = (cv::Mat_<float>(3, 1) << inverseRobotMatrix.at<float>(0,3),inverseRobotMatrix.at<float>(1,3),inverseRobotMatrix.at<float>(2,3));


        cv::Mat robotPoseRM;
        Rodrigues(robotPoseRVec, robotPoseRM);
        std::vector<float> robotPoseTrans(12, 0);
        robotPoseTrans[0] = robotPoseRM.at<float>(0,0);//r11
        robotPoseTrans[1] = robotPoseRM.at<float>(0,1);//r12
        robotPoseTrans[2] = robotPoseRM.at<float>(0,2);//r13
        robotPoseTrans[3] = robotPoseTVec.at<float>(0,0);//t1
        robotPoseTrans[4] = robotPoseRM.at<float>(1,0);//r21
        robotPoseTrans[5] = robotPoseRM.at<float>(1,1);//r22
        robotPoseTrans[6] = robotPoseRM.at<float>(1,2);//r23
        robotPoseTrans[7] = robotPoseTVec.at<float>(0,1);//t2
        robotPoseTrans[8] = robotPoseRM.at<float>(2,0);//r31
        robotPoseTrans[9] = robotPoseRM.at<float>(2,1);//r32
        robotPoseTrans[10] = robotPoseRM.at<float>(2,2);//r33
        robotPoseTrans[11] = robotPoseTVec.at<float>(0,2);//t3

        vpHomogeneousMatrix robotHomoTrans(robotPoseTrans);


        rMe.push_back(robotHomoTrans.inverse());
    }


    for(size_t i = 0; i < mR_target2cam.size()-1;i++){

        cv::Vec3d camImgPoseRVec{mR_target2cam[i].at<double>(0,0), mR_target2cam[i].at<double>(0,1), mR_target2cam[i].at<double>(0,2)};
        const cv::Mat camImgPose = (cv::Mat_<double>(3, 1) << mT_target2cam[i].at<double>(0,0), mT_target2cam[i].at<double>(0,1), mT_target2cam[i].at<double>(0,2));
        cv::Mat camImgPoseRM;
        Rodrigues(camImgPoseRVec, camImgPoseRM);
        std::vector<double> cameraTrans(12, 0);
        cameraTrans[0] = camImgPoseRM.at<double>(0,0);//r11
        cameraTrans[1] = camImgPoseRM.at<double>(0,1);//r12
        cameraTrans[2] = camImgPoseRM.at<double>(0,2);//r13
        cameraTrans[3] = camImgPose.at<double>(0,0);//t1
        cameraTrans[4] = camImgPoseRM.at<double>(1,0);//r21
        cameraTrans[5] = camImgPoseRM.at<double>(1,1);//r22
        cameraTrans[6] = camImgPoseRM.at<double>(1,2);//r23
        cameraTrans[7] = camImgPose.at<double>(0,1);//t2
        cameraTrans[8] = camImgPoseRM.at<double>(2,0);//r31
        cameraTrans[9] = camImgPoseRM.at<double>(2,1);//r32
        cameraTrans[10] = camImgPoseRM.at<double>(2,2);//r33
        cameraTrans[11] = camImgPose.at<double>(0,2);//t3

        vpHomogeneousMatrix cameraHomoTrans(cameraTrans);

        cMo.push_back(cameraHomoTrans.inverse());
    }


    int doneCalibration = vpHandEyeCalibration::calibrate(cMo,rMe,eMc);

    return eMc;


}

vpHomogeneousMatrix WorkspaceCalibration::testAfVisp(int numbOfPose){


    std::vector<vpHomogeneousMatrix> cMo;
    std::vector<vpHomogeneousMatrix> rMe;
    vpHomogeneousMatrix eMc;

    //Load TCP poses from file
    std::string fileLocationRobotTcp = ("../Detection/RobotposeData.txt");

    loadFileRobotTCP(fileLocationRobotTcp);

    //Load Rvec and Tvec from camera from file
    std::string fileLocation = ("../Detection/MarkertransposeData.txt");

    loadFileTranRot(fileLocation);

    for(size_t i = 0; i < mTcpPose.size();i++){

        cv::Vec3f robotPoseRVec{mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5)};
        float factor = sqrt(pow(robotPoseRVec[0], 2) + pow(robotPoseRVec[1], 2) + pow(robotPoseRVec[2], 2));
        cv::Vec3f robotUnitVector{robotPoseRVec[0]/factor, robotPoseRVec[1]/factor, robotPoseRVec[2]/factor};

        vpThetaUVector robotRotationVector(mTcpPose[i].at<float>(0,3),mTcpPose[i].at<float>(0,4),mTcpPose[i].at<float>(0,5));
        //vpThetaUVector robotRotationVector(robotUnitVector[0],robotUnitVector[1],robotUnitVector[2]);

        vpTranslationVector robotTranslationVector(mTcpPose[i].at<float>(0,0),mTcpPose[i].at<float>(0,1),mTcpPose[i].at<float>(0,2));
        //vpTranslationVector robotTranslationVector(mTcpPose[i].at<float>(0,0)*1000,mTcpPose[i].at<float>(0,1)*1000,mTcpPose[i].at<float>(0,2)*1000);

        vpPoseVector robotPoseVector(robotTranslationVector,robotRotationVector);

        rMe.push_back(vpHomogeneousMatrix(robotPoseVector));
    }

    for(size_t i = 0; i < mR_target2cam.size();i++){

        vpThetaUVector cameraRotationVector(mR_target2cam[i].at<double>(0,0), mR_target2cam[i].at<double>(0,1), mR_target2cam[i].at<double>(0,2));

        vpTranslationVector cameraTranslationVector(mT_target2cam[i].at<double>(0,0), mT_target2cam[i].at<double>(0,1), mT_target2cam[i].at<double>(0,2));

        vpPoseVector cameraPoseVector(cameraTranslationVector,cameraRotationVector);

        cMo.push_back(vpHomogeneousMatrix(cameraPoseVector));
    }

    int doneCalibration = vpHandEyeCalibration::calibrate(cMo,rMe,eMc);

    return eMc;
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
