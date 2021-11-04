#include "detectionMarker.h"

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

void DetectionMarker::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){


    cout<< "Insite detectionMarker!" <<endl;
    detectImages(reciver, controller);

}

void DetectionMarker::detectImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){
   Camera::initialize(reciver, controller);
}

// Override
void DetectionMarker::action(Pylon::CInstantCamera& camera,  ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller)
{
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
    Pylon::CGrabResultPtr ptrGrabResult;
    Pylon::CPylonImage pylonImage;

    cv::Mat openCvImage;

    int imageNr = 1;
    int frame = 1;
    bool runSQ = false;
    while ( camera.IsGrabbing())
    {
        camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded())
        {

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

            if (runSQ) {
                try{
                    solvePnP(Mat(boardPoints), Mat(corners), mCameraMatrix, mDistortionCoefficient, mRvec, mTvec, false);
                    //writeFileTranRot(mRvec, mTvec);
                    cout<< "Rotation vector " << mRvec <<endl;
                    cout<< "Translation vector " << mTvec <<endl;
                } catch(exception& e){
                    cout<< "Exception: " << endl;
                }
                runSQ = false;
            }
            projectPoints(framePoints, mRvec, mTvec, mCameraMatrix, mDistortionCoefficient, imageFramePoints);

            line(imgUndistorted, imageFramePoints[0], imageFramePoints[1], Scalar(0,255,0), 2, LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[2], Scalar(255,0,0), 2, LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[3], Scalar(0,0,255), 2, LINE_AA);

//            drawMarker(imgUndistorted, imageFramePoints[4], Scalar(0,0,255), MARKER_CROSS, 20, 4, 4);
//            drawMarker(imgUndistorted, imageFramePoints[5], Scalar(0,0,255), MARKER_CROSS, 20, 4, 4);
//            drawMarker(imgUndistorted, imageFramePoints[6], Scalar(0,0,255), MARKER_CROSS, 20, 4, 4);

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

            mRobotPoint3d = vectorfromframeCPtoCBCp(checkerboardCenter, frameCenter, pixelPmm, distanceObj);

            MoveArm urArm;
            double velocity = 0.1, acceleration = 0.1;
            urArm.getToCheckerboard(reciver, controller, mRobotPoint3d, velocity, acceleration);

//            drawMarker(imgUndistorted, {frameCenter.x, frameCenter.y}, Scalar(0,0,255), MARKER_STAR, 20, 4, 4);
//            drawMarker(imgUndistorted, {checkerboardCenter.x, checkerboardCenter.y}, Scalar(0,0,255), MARKER_CROSS, 20, 4, 4);

//            Point2f robotPointtest = vectorBetween2Points(imageFramePoints[0], frameCenter) * pixelPmm;
//            Point3f robotPointtest3d;

//            robotPointtest3d.x = (robotPointtest.x + 16)/1000; // + 16 for translation between camera and tcp(gripper) in x axis
//            robotPointtest3d.y = (robotPointtest.y + 43)/1000; // + 43 for translation between camera and tcp(gripper) in y axis
//            robotPointtest3d.z = (distanceObj - 129)/1000; // - 129 for translation between camera and tcp(gripper) in z axis

//            mRobotPointtest3d = robotPointtest3d;

//            cout<< "Origo from pP: " << imageFramePoints[0] <<endl;
//            cout<< "Origo from findCorners: " << corners[0] <<endl;
//            cout<< "Origo from pP in mm: " << imageFramePoints[0].x/61.8 << ", " <<imageFramePoints[0].y/61.8 <<endl;
//            cout<< "Origo from findCorners in mm: " << corners[0].x/61.8 << ", " <<corners[0].y/61.8 <<endl;
//            cout<< "Point 1 " << imageFramePoints[1] <<endl;
//            cout<< "Rotation vector " << mRvec <<endl;
//            cout<< "Translation vector " << mTvec <<endl;
//            cout<< "Distance to object: " << distanceObj <<endl;
            //cout<< robotPointtest3d <<endl;

            openCvImage = imgUndistorted.clone();
            int width = openCvImage.size().width * 60/100;
            int height = openCvImage.size().height * 60/100;
            cv::Size dimension (width, height);
            cv::resize(openCvImage,openCvImage,dimension);

            std::stringstream vindue;
            vindue << "Video feed: Press G to grab and colect imge, Q to Cancel and Quit or B to brake and reuse old image to calibration. nr. " << imageNr;
            cv::namedWindow( vindue.str() , cv::WINDOW_AUTOSIZE);
            cv::imshow( vindue.str(), openCvImage);

            // std::cout << "" << std::endl;
            // Detect key press 'q' is pressed
            char keyPressed = cv::waitKey(1);
            if(keyPressed == 'g'|| keyPressed == 'G' ){
                runSQ = true;
                //std::cout << "Grabbing and saving imge" << imageNr << ". to folder \"imageResources\"..." << std::endl;
                //std::stringstream fileName;
                //fileName<< "../imageResources/" << "Image" << imageNr << ".png";
                //cv::imwrite( fileName.str(), openCvImage );
                //std::cout << "Grabing and saving image to lacation was succesfull" << std::endl;
                cv::destroyWindow(vindue.str());
                imageNr++;

            } else if (keyPressed == 'q'|| keyPressed == 'Q' ) { // Quit if Q is Pressed
                std::cout << "Shutting down camera..." << std::endl;
                camera.Close();
                cv::destroyWindow(vindue.str());
                std::cout << "Camera successfully closed." << std::endl
                          << "The Grapping Proces was terminated." << std::endl
                          << "Closing program" << std::endl;
                exit(0);
            }else if (keyPressed == 'b'|| keyPressed == 'B' ) { // Brake and reuse old images.
                std::cout << "Shutting down camera..." << std::endl;
                camera.Close();
                cv::destroyWindow(vindue.str());
                std::cout << "Camera successfully closed." << std::endl
                          << "The Grapping Proces was terminated." << std::endl
                          << "Breaking out and continueing program with calibration." << std::endl;
                break;

            }else if (keyPressed == 'n'|| keyPressed == 'N' ) { // regrab N image.
                cv::destroyWindow(vindue.str());
                imageNr++;

            }


            if (imageNr > mNumberOfCalibrationImages ) {
                break;
            }
            /////

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

    robotPoint3d.x = (robotPoint2d.x + 16)/1000; // + 16 for translation between camera and tcp(gripper) in x axis
    robotPoint3d.y = (robotPoint2d.y + 43)/1000; // + 43 for translation between camera and tcp(gripper) in y axis
    robotPoint3d.z = (distanceObj - 129)/1000; // - 129 for translation between camera and tcp(gripper) in z axis

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
