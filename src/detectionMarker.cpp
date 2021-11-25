#include "detectionMarker.h"
#include <cstddef>

using namespace cv;
using namespace std;



DetectionMarker::DetectionMarker( CameraSettings& cameraSettings, int verticalIntersections, int horizontalIntersections, int squareSize, int numberOfCalibrationImages)
    : Camera(cameraSettings),
      mVerticalIntersections(verticalIntersections),
      mHorizontalIntersections(horizontalIntersections),
      mSquareSize(squareSize),
      mNumberOfCalibrationImages(numberOfCalibrationImages){
    ////////// Redding ind calibrationData //////////////
    getCalibrationData("../Detection/CalibrationData.txt");

    /////////// Creating rectifying Maps //////////////
    cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, mCamerasettings.getResolution(), CV_32FC1, mMapX, mMapY);

}

void DetectionMarker::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, bool flagDetectMarker){
    std::cout<< "Inside detectionMarker!" << std::endl;
    flagDetect = flagDetectMarker;
    detectImages(reciver, controller);
}

void DetectionMarker::detectImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){
   Camera::initialize(reciver, controller);
}

// Override
void DetectionMarker::action(Pylon::CInstantCamera& camera,  ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
    Pylon::CGrabResultPtr ptrGrabResult;
    Pylon::CPylonImage pylonImage;

    cv::Mat openCvImage;

    int imageNr = 1;
    int frame = 1;
    int progress = 1;
    bool runSQ = false;
    while ( camera.IsGrabbing()){
        camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded()){
            formatConverter.Convert(pylonImage, ptrGrabResult);
            // Create an OpenCV image from a pylon image.
            openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

            cv::Size frameSize(1920, 1200);
            cv::Size patternSize(7 - 1, 6 - 1);
            vector<Point2f> corners, imageFramePoints;
            vector<Point3f> framePoints, boardPoints;


            cv::Mat mapX, mapY, imgUndistorted;
            imgUndistorted = openCvImage.clone();
            cv::Mat mRvec = Mat(Size(3, 1), CV_64F);
            cv::Mat mTvec = Mat(Size(3, 1), CV_64F);
            cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, frameSize, CV_32FC1, mMapX, mMapY);

            cv::remap(openCvImage,imgUndistorted,mMapX,mMapY,1);

            cv::Mat gray;
            cvtColor(imgUndistorted,gray,COLOR_BGR2GRAY);

            bool patternfound = cv::findChessboardCorners(imgUndistorted, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

            if(patternfound){
            cv::cornerSubPix(gray,corners,cv::Size(4,4), cv::Size(-1, -1), cv::TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

            }

            for (int i = 0; i < patternSize.height; i++){
                for (int j = 0; j < patternSize.width; j++){
                    boardPoints.push_back(Point3f(float(i), float(j), 0.0));
                }
            }
            framePoints.push_back(Point3f(0.0, 0.0, 0.0));
            framePoints.push_back(Point3f(2.0, 0.0, 0.0));
            framePoints.push_back(Point3f(0.0, 2.0, 0.0));
            framePoints.push_back(Point3f(0.0, 0.0, 2.0));
            framePoints.push_back(Point3f(4.0, 5.0, 0.0));
            framePoints.push_back(Point3f(4.0, 0.0, 0.0));
            framePoints.push_back(Point3f(0.0, 5.0, 0.0));

            try{
                solvePnP(Mat(boardPoints), Mat(corners), mCameraMatrix, mDistortionCoefficient, mRvec, mTvec, false);
                cout<< "Rotation vector " << mRvec <<endl;
                cout<< "Translation vector " << mTvec <<endl;
            } catch(exception& e){
                cout<< "Exception: " << endl;
            }

            projectPoints(framePoints, mRvec, mTvec, mCameraMatrix, mDistortionCoefficient, imageFramePoints);

            line(imgUndistorted, imageFramePoints[0], imageFramePoints[1], Scalar(0,255,0), 2, LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[2], Scalar(255,0,0), 2, LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[3], Scalar(0,0,255), 2, LINE_AA);

            double focalL = 3316.188;
            double widthObj = 64.03;
            double distanceObj;

            distanceObj = (widthObj*focalL)/lineLength(imageFramePoints[0].x, imageFramePoints[0].y, imageFramePoints[4].x, imageFramePoints[4].y);
            double pixelPmm = widthObj / lineLength(imageFramePoints[0].x, imageFramePoints[0].y, imageFramePoints[4].x, imageFramePoints[4].y);

            Vec4f checkerboardLine1, checkerboardLine2;

            checkerboardLine1 = {imageFramePoints[0].x, imageFramePoints[0].y, imageFramePoints[4].x, imageFramePoints[4].y};
            checkerboardLine2 = {imageFramePoints[5].x, imageFramePoints[5].y, imageFramePoints[6].x, imageFramePoints[6].y};

            Point2i checkerboardCenter = centerPoint(checkerboardLine1, checkerboardLine2);
            Point2i frameCenter = {imgUndistorted.size().width/2, imgUndistorted.size().height/2};



            cv::Mat Origo = (cv::Mat_<double>(4, 1) << vectorBetween2Points(imageFramePoints[0],frameCenter).x, vectorBetween2Points(imageFramePoints[0],frameCenter).y, (distanceObj/pixelPmm), 1);

            cv::Mat OrigoPoint = Origo;
            OrigoPoint.at<double>(0,0) = (OrigoPoint.at<double>(0,0) * pixelPmm)/1000;
            OrigoPoint.at<double>(1,0) = (OrigoPoint.at<double>(1,0) * pixelPmm)/1000;
            OrigoPoint.at<double>(2,0) = (OrigoPoint.at<double>(2,0) * pixelPmm)/1000;
            OrigoPoint.at<double>(3,0) = 1;


            cv::Point2f xVector = vectorBetween2Points(imageFramePoints[0], imageFramePoints[2]);

            double zRotation = (atan(xVector.y/xVector.x));

            cv::Mat RRodriguesMatrix = (cv::Mat_<double>(3,3));
            cv::Mat RRodriguesMatrixTrans = (cv::Mat_<double>(3,3));

            Rodrigues(mRvec, RRodriguesMatrix);

            transpose(RRodriguesMatrix, RRodriguesMatrixTrans);

            cv::Vec3f rotation = rotationMatrixToEulerAngles(RRodriguesMatrixTrans);
            cv::Vec3f robotRvec;

            if(rotation[0] > 0){
                robotRvec[0] = -(M_PI - rotation[0]);

            }else if(rotation[0] < 0){
                robotRvec[0] = M_PI + rotation[0];
            }

            if(rotation[1] > 0){
                 robotRvec[1] = -rotation[1];

            }else if(rotation[1] < 0){
                 robotRvec[1] = -(rotation[1]);
            }
            robotRvec[2] = zRotation;

           char keyPressed;


//          if (flagDetect) {
//              mRobotPoint3d[0] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).x;
//              mRobotPoint3d[1] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).y;
//              mRobotPoint3d[2] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).z;
//              mRobotPoint3d[3] = 0.0;
//              mRobotPoint3d[4] = 0.0;
//              mRobotPoint3d[5] = zRotation;
//              keyPressed = 'c';
//          } else if (flagDetect && runSQ){
//              keyPressed = cv::waitKey(1);
//          } else {
//              mRobotPoint3d[0] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).x;
//              mRobotPoint3d[1] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).y;
//              mRobotPoint3d[2] = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj).z;


//              if(rotation[0] > 0){
//                  mRobotPoint3d[3] = -(M_PI - rotation[0]);

//              }else if(rotation[0] < 0){
//                  mRobotPoint3d[3] = M_PI + rotation[0];
//              }

//              if(rotation[1] > 0){
//                  mRobotPoint3d[4] = -rotation[1];

//              }else if(rotation[1] < 0){
//                  mRobotPoint3d[4] = -(rotation[1]);
//              }
//              mRobotPoint3d[5] = zRotation;
//              //keyPressed = 'j';
//              keyPressed = cv::waitKey(1);
//          }



            drawMarker(imgUndistorted, {frameCenter.x, frameCenter.y}, Scalar(0,0,255), MARKER_STAR, 20, 4, 4);
            drawMarker(imgUndistorted, {checkerboardCenter.x, checkerboardCenter.y}, Scalar(0,0,255), MARKER_CROSS, 20, 4, 4);


            openCvImage = imgUndistorted.clone();
            int width = openCvImage.size().width * 60/100;
            int height = openCvImage.size().height * 60/100;
            cv::Size dimension (width, height);
            cv::resize(openCvImage,openCvImage,dimension);

            std::stringstream vindue;
            vindue << "Video feed: Press J to perfom job, C to get to checker, Q to Cancel and Quit" << imageNr;
            cv::namedWindow( vindue.str() , cv::WINDOW_AUTOSIZE);
            cv::imshow( vindue.str(), openCvImage);

            //char keyPressed = cv::waitKey(1);
            if(keyPressed == 'j'|| keyPressed == 'J' ){
                cv::destroyWindow(vindue.str());
                MoveArm urArm;
                double velocity = 0.02;
                double acceleration = 0.02;
                if(progress < 4){
                std::cout << "Inside loop. start Progress: " << progress << std::endl;
                //urArm.getToJob(reciver, controller, mRobotPoint3d, progress, velocity, acceleration);
                progress++;
                }
                imageNr++;
            } else if (keyPressed == 't'|| keyPressed == 'T' ) { // Quit if Q is Pressed

                MoveArm urArm;
                WorkspaceCalibration transMatrix;
                cv::Mat robotTransMatrix = transMatrix.getRobotTransformationMatrix(urArm.receiveJPose(reciver)) * transMatrix.getTransformationFlange2EndEffector() * transMatrix.getTransformationEndEffector2Camera();
                cv::Mat robotTvec = robotTransMatrix * OrigoPoint;
                cv::Vec6f robotPoint;
                robotPoint[0] = robotTvec.at<double>(0,0);
                robotPoint[1] = robotTvec.at<double>(0,1);
                robotPoint[2] = robotTvec.at<double>(0,2);
                robotPoint[3] = mRvec.at<double>(0,0);
                robotPoint[4] = mRvec.at<double>(0,1);
                robotPoint[5] = mRvec.at<double>(0,2);

                double velocity = 0.02;
                double acceleration = 0.02;
                moveFrame = urArm.getToCheckerboard(reciver, controller, 0, robotPoint, velocity, acceleration);

                //imageNr++;
            }else if (keyPressed == 'c'|| keyPressed == 'C' ) { // Quit if Q is Pressed
                runSQ = true;
                std::cout << "Shutting down camera..." << std::endl;
                cv::destroyWindow(vindue.str());
                MoveArm urArm;
                double velocity = 0.02;
                double acceleration = 0.02;
                moveFrame = urArm.getToCheckerboard(reciver, controller, mRobotPoint3d, velocity, acceleration);

                std::cout << "Shutting down camera..." << std::endl;
                camera.Close();
                cv::destroyWindow(vindue.str());
                exit(0);

                //imageNr++;
            }else if (keyPressed == 'q'|| keyPressed == 'Q' ) { // Quit if Q is Pressed
                std::cout << "Shutting down camera..." << std::endl;
                camera.Close();
                cv::destroyWindow(vindue.str());
                std::cout << "Camera successfully closed." << std::endl
                          << "The Grapping Proces was terminated." << std::endl
                          << "Closing program" << std::endl;
                exit(0);
            }


            if (imageNr > mNumberOfCalibrationImages ) {
                break;
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

void DetectionMarker::getCalibrationData(std::string fileLocation)
{
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

        mOpticalCenter = {static_cast<int>(mCameraMatrix.val[2]),static_cast<int>(mCameraMatrix.val[5])};

        std::cout << "[ Info ]: The following data have been read in... \n"
                  << "\nerrorRMS:\n " << mErrorRMS
                  << "\nCamaraMatrix:\n " << mCameraMatrix
                  << "\nDistortionCoeficient:\n " << mDistortionCoefficient
                  << "\nOpticalCenter:\n " << mOpticalCenter << std::endl;
    }
}

double DetectionMarker::lineLength(double sX, double sY, double eX, double eY){
    double v1Coordinate, v2Coordinate;
    double vLength;

    v1Coordinate = eX-sX;
    v2Coordinate = eY-sY;

    vLength = sqrt(pow(v1Coordinate, 2)+pow(v2Coordinate, 2));

    return vLength;
}

cv::Point2f DetectionMarker::centerPoint(cv::Vec4f line1, cv::Vec4f line2)
{
    float cPX, cPY;
    cv::Point2f cPoint;

    cPX = (((((line1[0]*line1[3])-(line1[1]*line1[2]))*(line2[0]-line2[2]))-((line1[0]-line1[2])*((line2[0]*line2[3])-(line2[1]*line2[2]))))/(((line1[0]-line1[2])*(line2[1]-line2[3]))-((line1[1]-line1[3])*(line2[0]-line2[2]))));

    cPY =  (((((line1[0]*line1[3])-(line1[1]*line1[2]))*(line2[1]-line2[3]))-((line1[1]-line1[3])*((line2[0]*line2[3])-(line2[1]*line2[2]))))/(((line1[0]-line1[2])*(line2[1]-line2[3]))-((line1[1]-line1[3])*(line2[0]-line2[2]))));

    cPoint = {cPX, cPY};


    return cPoint;
}

cv::Point2f DetectionMarker::vectorBetween2Points(cv::Point2f startPoint, cv::Point2f endPoint){

    cv::Point2f pointVector = {(endPoint.x - startPoint.x), (endPoint.y - startPoint.y)};

    return pointVector;
}

cv::Point3f DetectionMarker::vectorfromframeCPtoCBCp(cv::Point2i checkerBoardCP, cv::Point2i frameCP, double pixelPmm, double distanceObj){

    Point2f robotPoint2d = vectorBetween2Points(checkerBoardCP, frameCP) * pixelPmm;
    Point3f robotPoint3d;

//    robotPoint3d.x = (robotPoint2d.x + 16)/1000; // + 16 for translation between camera and tcp(gripper) in x axis
//    robotPoint3d.y = (robotPoint2d.y + 43)/1000; // + 43 for translation between camera and tcp(gripper) in y axis
//    robotPoint3d.z = (distanceObj - 129)/1000; // - 129 for translation between camera and tcp(gripper) in z axis

    robotPoint3d.x = (robotPoint2d.x)/1000;
    robotPoint3d.y = (robotPoint2d.y)/1000;
    robotPoint3d.z = 0.0;


    return robotPoint3d;
}

double DetectionMarker::getDistance2Object(cv::Point2f origo, cv::Point2f dia){
    double focalL = 3316.188; //f = (pixel*distance)/width = (536.6*395.52/64)
    double widthObj = 64.03;

    double distanceObj = (widthObj*focalL)/ DetectionMarker::lineLength(origo.x, origo.y, dia.x, dia.y);

    return distanceObj;
}

double DetectionMarker::getPixelPermm(cv::Point2f origo, cv::Point2f dia){
    double widthObj = 64.03;

    double pixelPmm = widthObj / DetectionMarker::lineLength(origo.x, origo.y, dia.x, dia.y);

    return pixelPmm;
}

bool DetectionMarker::isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

cv::Vec3f DetectionMarker::rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}

cv::Vec3f DetectionMarker::rpy2rv(cv::Vec3f rpy){

    float alpha = rpy[2];
    float beta = rpy[1];
    float gamma = rpy[0];

    float cAlpha = cos(alpha);
    float cBeta = cos(beta);
    float cGamma = cos(gamma);
    float sAlpha = sin(alpha);
    float sBeta = sin(beta);
    float sGamma = sin(gamma);

    float r11 = cAlpha*cBeta;
    float r12 = cAlpha*sBeta*sGamma-sAlpha*cGamma;
    float r13 = cAlpha*sBeta*cGamma+sAlpha*sGamma;
    float r21 = sAlpha*cBeta;
    float r22 = sAlpha*sBeta*sGamma+cAlpha*cGamma;
    float r23 = sAlpha*sBeta*cGamma-cAlpha*sGamma;
    float r31 = -sBeta;
    float r32 = cBeta*sGamma;
    float r33 = cBeta*cGamma;

    float theta = acos((r11+r22+r33-1)/2);
    float sth = sin(theta);
    float kx = (r32-r23)/(2*sth);
    float ky = (r13-r31)/(2*sth);
    float kz = (r21-r12)/(2*sth);

    cv::Vec3f rvec;
    rvec[0] = (theta*kx);
    rvec[1] = (theta*ky);
    rvec[2] = (theta*kz);

    return rvec;
}

bool DetectionMarker::writeFileTranRot (Mat tempRvec, Mat tempTvec){
    std::ofstream myfile;
    myfile.open ("../Detection/MarkertransposeData.txt");
    string matAsStringTempRvec (tempRvec.begin<unsigned char>(), tempRvec.end<unsigned char>());
    string matAsStringTempTvec (tempTvec.begin<unsigned char>(), tempTvec.end<unsigned char>());
    myfile << "Reprojection error = \n" << mErrorRMS << std::endl
           << "Camera Matrix = \n" <<  mCameraMatrix << std::endl
           << "DistortionCoefficient= \n" << mDistortionCoefficient << std::endl;
    myfile.close();

    return 0;

}
