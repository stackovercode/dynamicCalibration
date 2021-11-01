#include "workspaceCalibration.h"

/********************************************************************************
*                               CONSTRUCTOR                                     *
********************************************************************************/
WorkspaceCalibration::WorkspaceCalibration(CameraSettings& cameraSetting, DetectionObject& detectionObject)
    : Camera(cameraSetting), mDetectionObject{detectionObject}{
    ////////// Redding ind data //////////////
    getCalibrationData("../Detection/CalibrationData.txt");
    loadFileTranRot("../Detection/MarkertransposeData.txt");
    loadFileRobotJoint("../Detection/RobotjointData.txt");
    loadFileRobotTCP("../Detection/RobotposeData.txt");
    loadFileImagePoints("../Detection/CalibrationData.txt");
    /////////// Creating rectifying Maps //////////////
    cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, mCamerasettings.getResolution(), CV_32FC1, mMapX, mMapY);
}

void WorkspaceCalibration::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, double lengthXmm, double lengthYmm){
    //cv::Vec6f robotJointAngles = {-1.36689,-1.28005,-1.90308,-3.10318,-1.70538,1.5708};
    cv::Vec6f robotJointAngles = mJointPose[24];
    cv::Mat testMatrix = getTransformationMatrixBase2Cam(robotJointAngles);

    cv::Point2f centerPoint = mCenterPoint;
    cv::Point2f diaPoint = mDiaPoint;

    cv::Mat OrigoMat = (cv::Mat_<double>(4, 1) << centerPoint.x, centerPoint.y, getDistance2Object(centerPoint, diaPoint)/getPixelPermm(centerPoint, diaPoint), 1);
    cv::Mat OrigoPoint = getTransformationMatrixImage2Camera(mR_target2cam[0], mT_target2cam[0]) * OrigoMat;
    OrigoPoint.at<double>(0,0) = (OrigoMat.at<double>(0,0) * getPixelPermm(centerPoint, diaPoint))/1000;
    OrigoPoint.at<double>(1,0) = (OrigoMat.at<double>(1,0) * getPixelPermm(centerPoint, diaPoint))/1000;
    OrigoPoint.at<double>(2,0) = (OrigoMat.at<double>(2,0) * getPixelPermm(centerPoint, diaPoint))/1000;

    cv::Mat robotPoint = testMatrix * OrigoPoint;

    std::cout << "Point: " << robotPoint << std::endl;
    std::cout << "Transformation: " << testMatrix << std::endl;
}

void WorkspaceCalibration::action(Pylon::CInstantCamera& camera, ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller)
{
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
    Pylon::CGrabResultPtr ptrGrabResult;
    Pylon::CPylonImage pylonImage;
    cv::Mat openCvImage, undistortedImage;

    for (int i = 0; i < mWorkspaceCorners.size(); ++i) {
        bool isGrabbingPosition = false;
        cv::Vec3d vecObjectPosition;
        std::vector<double> objectVectorX;
        std::vector<double> objectVectorY;
        std::vector<double> objectVectorZ;

        int noDetection = 0;
        int grabbingIterator = 0;
        int frame = 1;
        while ( camera.IsGrabbing())
        {
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            if (ptrGrabResult->GrabSucceeded())
            {
                formatConverter.Convert(pylonImage, ptrGrabResult);
                openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

                cv::remap(openCvImage, undistortedImage, mMapX, mMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

                bool isObjectFound = mDetectionObject.locate(vecObjectPosition, undistortedImage);
                if ( !isObjectFound )
                {
                    ++noDetection;
                    std::cerr << "Error: object not detected:" << noDetection << " times."<< std::endl;
                }

                //Textbox counter in the bottom of video feed
                std::stringstream ss;
                ss << "[ " << grabbingIterator << " images of 10" <<" ]";
                //cv::putText(undistortedImage, ss.str(), cv::Point(600,30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255),2);

                // Create an OpenCV display window.
                std::stringstream window;
                window << "Video feed:" << ( ( !isGrabbingPosition ) ? "Press G to grab Object Center" : " "  )
                                        << ( ( i==0 ) ? " At top left corner in ROI" : " "  )
                                        << ( ( i==1 ) ? " At top right corner in ROI" : " "  )
                                        << ( ( i==2 ) ? " At buttom left corner in ROI" : " "  )
                                        << ( ( i==3 ) ? " At buttom right corner in ROI" : " "  );
                cv::namedWindow( window.str() , cv::WINDOW_AUTOSIZE);
                cv::imshow( window.str(), undistortedImage);

                // Detect 'key' is pressed
                int keyPressed = cv::waitKey(1);
                if(keyPressed == 'g'|| keyPressed == 'G' || isGrabbingPosition)
                {
                    isGrabbingPosition = true;

                    if (grabbingIterator < 10 && isObjectFound){
                        objectVectorX.push_back(vecObjectPosition[0]);
                        objectVectorY.push_back(vecObjectPosition[1]);
                        objectVectorZ.push_back(vecObjectPosition[2]);

                        grabbingIterator++;

                    } else if(grabbingIterator == 10) {
                        std::sort(objectVectorX.begin(), objectVectorX.end());
                        std::sort(objectVectorY.begin(), objectVectorY.end());
                        std::sort(objectVectorZ.begin(), objectVectorZ.end());

                        cv::Point meanObjectPosition(objectVectorX[5], objectVectorY[5]);
                        mWorkspaceCorners[i] = meanObjectPosition;
                        std::cout << "Mean object position =\n " << meanObjectPosition << std::endl;

                        std::cout << "Destroying Window..." << std::endl;
                        cv::destroyAllWindows();

                        std::stringstream vindue;
                        vindue << "Result of grab";
                        cv::namedWindow( vindue.str() , cv::WINDOW_AUTOSIZE);
                        cv::imshow( vindue.str(), undistortedImage);
                        cv::waitKey(0);
                        break;
                    }
                }

                frame++;
            }
            else
            {
                std::cerr << "[ Fail ]:  " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
                exit(-1);
            }
        }
    }
    std::cout << "Shutting down camera..." << std::endl;
    camera.Close();
    std::cout << "Camera successfully closed." << std::endl;

    std::cout << "point1 " << mWorkspaceCorners[0] << std::endl
              << "point2 " << mWorkspaceCorners[1] << std::endl
              << "point3 " << mWorkspaceCorners[2] << std::endl
              << "point4 " << mWorkspaceCorners[3] << std::endl;
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