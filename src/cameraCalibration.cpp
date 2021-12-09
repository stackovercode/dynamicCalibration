#include "cameraCalibration.h"
#include <cstddef>


CameraCalibration::CameraCalibration(CameraSettings& cameraSettings, int verticalIntersections, int horizontalIntersections, int squareSize, int numberOfCalibrationImages)
    : Camera(cameraSettings),
      mVerticalIntersections(verticalIntersections),
      mHorizontalIntersections(horizontalIntersections),
      mSquareSize(squareSize),
      mNumberOfCalibrationImages(numberOfCalibrationImages)
{

}

void CameraCalibration::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){

    collectCalibratingImages(reciver, controller);

    drawChessboardCorners(chessboardCornersArray, true);

    generateCheckerboardWorld(checkerboardWorldArray);

    calibrate(chessboardCornersArray,checkerboardWorldArray, true);
}

void CameraCalibration::collectCalibratingImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller){
   Camera::initialize(reciver, controller);
}

// Override
void CameraCalibration::action(Pylon::CInstantCamera& camera, ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller )
{
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
    Pylon::CGrabResultPtr ptrGrabResult;
    Pylon::CPylonImage pylonImage;

    cv::Mat openCvImage;

    int imageNr = 1;
    int frame = 1;
    bool calibrateRunTime = false;
    bool addGausNoise = false;
    std::vector<double> initPose = {0.0,0.0,0.0,0.0,0.0,0.0};




    while ( camera.IsGrabbing())
    {
        camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded())
        {
            //std::string ur5IP = "192.168.100.50";
            //ur_rtde::RTDEControlInterface rtde_control(ur5IP, 30004);

            formatConverter.Convert(pylonImage, ptrGrabResult);
            // Create an OpenCV image from a pylon image.
            openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

            /*SolveVaribales*/
            cv::Mat newMapX, newMapY, newImgUndistorted;
            cv::Size newFrameSize(1920, 1200);
            cv::Size patternSize(7 - 1, 6 - 1);
            std::vector<cv::Point2f> corners, imageFramePoints;
            std::vector<cv::Point3f> framePoints, boardPoints;
            cv::Mat newRotationMatrix = (cv::Mat_<double>(3,3));
            cv::Vec3d eulerAngels;
            cv::Mat newTMethoedRotationMatrix = (cv::Mat_<double>(3,3));

            cv::Matx33f newCameraMatrix(cv::Matx33f::eye());
                newCameraMatrix = {2492.366, 0, 959.5,
                                   0, 2492.366, 599.5,
                                   0, 0, 1};

            cv::Vec<float, 5> mNewDistortionCoefficient(0, 0, 0, 0, 0);
                mNewDistortionCoefficient = {-0.0721785, -2.06161, 0, 0, 0};

            cv::Mat mNewRvec = cv::Mat(cv::Size(3, 1), CV_64F);
            cv::Mat mNewTvec = cv::Mat(cv::Size(3, 1), CV_64F);
            /*END*/



            double NoiseStdDev = 50; //65

            if(addGausNoise){

                cv::Mat GausNoise = cv::Mat(openCvImage.size(), CV_8UC3);

                randn(GausNoise, 0, NoiseStdDev);
                openCvImage += GausNoise;

                normalize(openCvImage, openCvImage, 0 ,255, cv::NORM_MINMAX, CV_8UC3);
            }



            char keyPressed;

            if (calibrateRunTime) {
                keyPressed = 'a';
            } else {
                keyPressed = cv::waitKey(1);
            }


            // std::cout << "" << std::endl;
            // Detect key press 'q' is pressed

            /*SolvePnP Start*/

            cv::initUndistortRectifyMap(newCameraMatrix, mNewDistortionCoefficient, cv::Matx33f::eye(), newCameraMatrix, newFrameSize, CV_32FC1, newMapX, newMapY);
            cv::remap(openCvImage,newImgUndistorted,newMapX,newMapY,1);

            cv::Mat gray;
            cvtColor(newImgUndistorted,gray,cv::COLOR_BGR2GRAY);

            bool patternfound = cv::findChessboardCorners(newImgUndistorted, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);


            if(patternfound){
              cv::cornerSubPix(gray,corners,cv::Size(4,4), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            }

            for (int i = 0; i < patternSize.height; i++){
                for (int j = 0; j < patternSize.width; j++){
                    boardPoints.push_back(cv::Point3f(float(i), float(j), 0.0));
                }
            }

            framePoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
            framePoints.push_back(cv::Point3f(2.0, 0.0, 0.0));
            framePoints.push_back(cv::Point3f(0.0, 2.0, 0.0));
            framePoints.push_back(cv::Point3f(0.0, 0.0, 2.0));
            framePoints.push_back(cv::Point3f(4.0, 5.0, 0.0));
            framePoints.push_back(cv::Point3f(4.0, 0.0, 0.0));
            framePoints.push_back(cv::Point3f(0.0, 5.0, 0.0));


            try{
                solvePnP(cv::Mat(boardPoints), cv::Mat(corners), newCameraMatrix, mNewDistortionCoefficient, mNewRvec, mNewTvec, false);
            } catch(std::exception& e){
                std::cout<< "Exception: " << std::endl;
            }

            projectPoints(framePoints, mNewRvec, mNewTvec, newCameraMatrix, mNewDistortionCoefficient, imageFramePoints);

            line(imgUndistorted, imageFramePoints[0], imageFramePoints[1], cv::Scalar(0,255,0), 2, cv::LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[2], cv::Scalar(255,0,0), 2, cv::LINE_AA);
            line(imgUndistorted, imageFramePoints[0], imageFramePoints[3], cv::Scalar(0,0,255), 2, cv::LINE_AA);


            /*SolvePnP Slut*/


            mWidth = openCvImage.size().width * 60/100;
            mHeight = openCvImage.size().height * 60/100;

           cv::Size dimension (mWidth, mHeight);
           cv::resize(openCvImage,openCvImage,dimension);




            std::stringstream vindue;
            vindue << "Video feed: Press A to automatic grab and collect image, Press G to manully grab and colect imge, Q to Cancel and Quit or B to brake and reuse old image to calibration. nr. or press N to regrab existing image" << imageNr;
            cv::namedWindow( vindue.str() , cv::WINDOW_AUTOSIZE);
            // Display the current image in the OpenCV display window.
            //cv::imshow( vindue.str(), openCvImage);
            cv::imshow( vindue.str(), openCvImage);

            if(keyPressed == 'a'|| keyPressed == 'A' ){
                calibrateRunTime = true;
                std::cout << "Grabbing and saving imge" << imageNr << ". to folder \"imageResources\"..." << std::endl;
                std::stringstream fileName;
                fileName<< "../imageResources/" << "Image" << imageNr << ".png";
                cv::imwrite( fileName.str(), openCvImage );
                std::cout << "Grabing and saving image to location was succesfull" << std::endl;
                cv::destroyWindow(vindue.str());
                /* RTDE handler object */
                MoveArm ur5arm;
                Rodrigues(mNewRvec, newRotationMatrix);
                newTMethoedRotationMatrix = getAngles(newRotationMatrix, eulerAngels);
                cv::Point3d pa{eulerAngels};
                pa.x = vpMath::rad(pa.x);
                pa.y = vpMath::rad(pa.y);
                pa.z = vpMath::rad(pa.z);
                //std::cout << "Euler angles: \n" << eulerAngels << std::endl;
                //writeFileTranRot3(newTMethoedRotationMatrix);
                //mRobotPose.push_back(ur5arm.moveCalibrate(reciver, controller,1.0,0.9, imageNr));
                tempRvec.push_back(mNewRvec);
                tempTvec.push_back(mNewTvec);
                initPose = ur5arm.poseSwift(reciver,controller,0.10,0.10,imageNr, initPose, mNumberOfCalibrationImages, false);
                imageNr++;

            } else if(keyPressed == 'g'|| keyPressed == 'G' ){
                std::cout << "Grabbing and saving imge" << imageNr << ". to folder \"imageResources\"..." << std::endl;
                std::stringstream fileName;
                fileName<< "../imageResources/" << "Image00" << imageNr << ".png";
                cv::imwrite( fileName.str(), openCvImage );
                std::cout << "Grabing and saving image to lacation was succesfull" << std::endl;
                cv::destroyWindow(vindue.str());
                /* RTDE handler object */
                MoveArm ur5arm;
                //mRobotPose.push_back(ur5arm.moveCalibrate(reciver, controller,1.0,0.9, imageNr));
                initPose = ur5arm.poseSwift(reciver,controller,0.1,0.1,imageNr, initPose, mNumberOfCalibrationImages, false);
                //mRobotPose.push_back(initPose);
                imageNr++;

            } else if (keyPressed == 'n'|| keyPressed == 'N' ) { // Quit if Q is Pressed
                std::cout << "Adding Gaussian noise to image." << std::endl;
                addGausNoise = true;
                std::cout << "Added Gaussian noise to image was succesfull." << std::endl;
            } else if (keyPressed == 'e'|| keyPressed == 'E' ) { // Quit if Q is Pressed
                std::cout << "Removing Gaussian noise from image." << std::endl;
                addGausNoise = false;
                std::cout << "Removed Gaussian noise from image was succesfull." << std::endl;
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

            }else if (keyPressed == 'x'|| keyPressed == 'X' ) { // regrab N image.
                cv::destroyWindow(vindue.str());
                imageNr++;

            }

            if (imageNr > mNumberOfCalibrationImages ) {
                WorkspaceCalibration transMatrix;
                writeFileTranRot(tempRvec, tempTvec);
                std::cout << "Hand Eye: \n" << transMatrix.getTransformationFlange2CameraHandEye(25, 0) << std::endl;
                //transMatrix.vispHandEyeCalibration(true, tempRvec, tempTvec);
                //std::cout << "Hand eye trans form visp: " << transMatrix.vispHandEyeCalibration(true, tempRvec, tempTvec) <<std::endl;
                //writeFileRobotPoses(mRobotPose);

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

void CameraCalibration::drawChessboardCorners(std::vector<std::vector<cv::Point2f>>& chessboardCornersArray, bool flagShowImage){
    std::vector<cv::String> fileNames;
    cv::glob("../imageResources/Image*.png", fileNames, false); // Generate a list of all files that match the globbing pattern.

    bool calibrateRunTime = true;
    if (fileNames.size() != mNumberOfCalibrationImages) {
        std::cerr << "Number of images in folder \"imageResources\" is not equel to required number of calibration images. \n"
                  << "ImgesResources = " << fileNames.size() << " Required = " << mNumberOfCalibrationImages;
        exit(-1);
    }
    cv::Size patternSize(mHorizontalIntersections, mVerticalIntersections);
    // Detect feature points
    for (auto const &file : fileNames) {
        std::cout << std::string(file) << std::endl;

        // 1. Read in the image an call cv::findChessboardCorners()
        cv::Mat image = cv::imread(file);
        bool isPatternfound = cv::findChessboardCorners(image, patternSize, chessboardCorners, cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE); // 0 can also be CALIB_CB_NORMALIZE, CALIB_CB_EXHAUSTIVE, and CALIB_CB_ACURACY .
        std::cout << "Found Chessboard Corners: " << std::boolalpha << isPatternfound << std::endl;

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        cv::Mat grayImage = cv::imread(file,0);
        cv::Size winSize = cv::Size( 5, 5 );
        cv::Size zeroZone = cv::Size( -1, -1 );
        cv::TermCriteria term(1,100,0);
        cv::cornerSubPix(grayImage, chessboardCorners, winSize, zeroZone, term);


        chessboardCornersArray.push_back(chessboardCorners);

        // Display
        if (flagShowImage) {
            cv::drawChessboardCorners(image, patternSize, chessboardCorners, isPatternfound );
            std::stringstream vindue;
            vindue << "Chessboard detection: Press B to Break and Not show any more images. Showing: " << file;
            cv::namedWindow(vindue.str() , cv::WINDOW_AUTOSIZE);

            //int width = image.size().width * 60/100;
            //int height = image.size().height * 60/100;

            cv::Size dimension (mWidth, mHeight);
            cv::resize(image,image,dimension);

            cv::imshow( vindue.str(), image);

            //char keyPressed = cv::waitKey(1);
            char keyPressed = 'b';

            //int keyPressed = cv::waitKey(0);
            if(keyPressed == 'b'|| keyPressed == 'B' ){
                flagShowImage = false;
            }
            cv::destroyWindow(vindue.str());
        }
    }
}

void CameraCalibration::generateCheckerboardWorld(std::vector<std::vector<cv::Point3f>>& checkerboardWorldArray )
{
    for ( int y=0 ; y < (mVerticalIntersections) ; y++ ) {
        for ( int x=0 ; x < (mHorizontalIntersections) ; x++ ) {
            checkerboardWorld.push_back(cv::Point3f(x*mSquareSize,y*mSquareSize,0));
        }
    }
    for ( int n=0 ; n < mNumberOfCalibrationImages ; ++n) {
        checkerboardWorldArray.push_back(checkerboardWorld);
    }
}

void CameraCalibration::calibrate(std::vector<std::vector<cv::Point2f>> const chessboardCornersArray, std::vector<std::vector<cv::Point3f>> const checkerboardWorldArray, bool flagShowImage )
{
    std::cout << "Calibrating..." << std::endl;

    if (checkerboardWorldArray.size() != mNumberOfCalibrationImages && chessboardCornersArray.size() != mNumberOfCalibrationImages) {
        std::cerr << "checkerboardWorldArray is not equel to required numberOfImages \n"
                  << "checkerboardWorldArray= " << checkerboardWorldArray.size() << " and \n"
                  << "chessboardCornersArray= " << chessboardCornersArray.size() << "\n"
                  << "Both required to be: " << mNumberOfCalibrationImages << std::endl;
        exit(-1);
    }

    //cv::Matx33f cameraMatrix(cv::Matx33f::eye());  // intrinsic camera matrix
    //cv::Vec<float, 5> distortionCoefficient(0, 0, 0, 0, 0); // distortion coefficients


    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO
            + cv::CALIB_FIX_K3
            + cv::CALIB_ZERO_TANGENT_DIST
            + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::TermCriteria term(1,100,0);

    mErrorRMS = cv::calibrateCamera(checkerboardWorldArray,chessboardCornersArray,mCamerasettings.getResolution(),mCameraMatrix,mDistortionCoefficient,rvecs,tvecs,flags,term);
    //writeFileTranRot(rvecs, tvecs);

        framePoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
        framePoints.push_back(cv::Point3f(2.0, 0.0, 0.0));
        framePoints.push_back(cv::Point3f(0.0, 2.0, 0.0));
        framePoints.push_back(cv::Point3f(0.0, 0.0, 2.0));
        framePoints.push_back(cv::Point3f(4.0, 5.0, 0.0));
        framePoints.push_back(cv::Point3f(4.0, 0.0, 0.0));
        framePoints.push_back(cv::Point3f(0.0, 5.0, 0.0));

    cv::projectPoints(framePoints, rvecs[0], tvecs[0], mCameraMatrix, mDistortionCoefficient, imageFramePoints);


    try{
        cv::solvePnP(checkerboardWorldArray[24], chessboardCornersArray[24], mCameraMatrix, mDistortionCoefficient, mRvec, mTvec, false);
        std::cout << "SolvePNP rotation: " << mRvec << std::endl;
        std::cout << "SolvePNP translation: " << mTvec << std::endl;
    } catch(std::exception& e){
        std::cout<< "Exception: " << std::endl;
    }

    // Display lens corrected images
    if (flagShowImage) {
        cv::Mat mapX, mapY;
        cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, mCamerasettings.getResolution(), CV_32FC1, mapX, mapY);

        std::vector<cv::String> fileNames;
        cv::glob("../imageResources/Image*.png", fileNames, false); // Generate a list of all files that match the globbing pattern.

        for (auto const &file : fileNames) {

            cv::Mat image = cv::imread(file, cv::IMREAD_COLOR);
            cv::Mat imgUndistorted;
            cv::remap(image, imgUndistorted, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );

            if (flagShowImage) {

                std::stringstream vindue;
                vindue << "Undistorted image: Press B to Break and Not show any more images. Image. " << file;
                cv::namedWindow( vindue.str(), cv::WINDOW_AUTOSIZE);

                //cv::Size dimension (mWidth, mHeight);
                //cv::resize(imgUndistorted,imgUndistorted,dimension);

//                mWidth = imgUndistorted.size().width * 60/100;
//                mHeight = imgUndistorted.size().height * 60/100;

                cv::Size dimension (mWidth, mHeight);
                cv::resize(imgUndistorted,imgUndistorted,dimension);


                cv::imshow( vindue.str(), imgUndistorted);
                int keyPressed = cv::waitKey(0);
                //char keyPressed = 'b';
                if(keyPressed == 'b'|| keyPressed == 'B' ){
                    flagShowImage = false;
                }
                cv::destroyWindow(vindue.str());
            }
        }
    }

    std::cout << "Writing calibartion file..." << std::endl;
    if ( writeFile()){
        std::cout << "Could not write file!" << std::endl;
        exit(-1);
    } else {
        std::cout << "Success! Files written" << std::endl;
    }
}


bool CameraCalibration::writeFile ()
{

    std::ofstream myfile;
    myfile.open ("../Detection/CalibrationData.txt");
    myfile << "Reprojection error = \n" << mErrorRMS << std::endl
           << "Camera Matrix = \n" <<  mCameraMatrix << std::endl
           << "DistortionCoefficient= \n" << mDistortionCoefficient << std::endl
           << "Center point = \n" << std::to_string(imageFramePoints[0].x) << ", " << std::to_string(imageFramePoints[0].y) << std::endl
           << "Diagonal point = \n" << std::to_string(imageFramePoints[4].x) << ", " << std::to_string(imageFramePoints[4].y) << std::endl
           << "Rotation vector center = \n" <<  mRvec << std::endl
           << "Translation vector center = \n" <<  mTvec << std::endl;
    myfile.close();
    return 0;
}



std::string CameraCalibration::cameraCalibrationToString()
{
    std::stringstream ss;
    ss << "Reprojection error = \n" << mErrorRMS << std::endl
       << "Camera Matrix = \n" <<  mCameraMatrix << std::endl
       << "DistortionCoefficient= \n" << mDistortionCoefficient << std::endl
       << "Center point = \n" << std::to_string(imageFramePoints[0].x) << ", " << std::to_string(imageFramePoints[0].y) << std::endl
       << "Diagonal point = \n" << std::to_string(imageFramePoints[4].x) << ", " << std::to_string(imageFramePoints[4].y) << std::endl
       << "Rotation vector center = \n" <<  mRvec << std::endl
       << "Translation vector center = \n" <<  mTvec << std::endl;
    std::string result = ss.str();

    return result;
}

std::string CameraCalibration::readVector(std::vector<double> result)
{
    std::ostringstream vts;
    if (!result.empty()){
    std::copy(result.begin(), result.end()-1,
        std::ostream_iterator<double>(vts, ","));
        vts << result.back();
    }
    return vts.str();
}


void CameraCalibration::writeFileTranRot (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec){
    std::ofstream myfile;
    myfile.open ("../Detection/MarkertransposeData.txt");
    for (int i = 0; i < tempRvec.size(); i++) {
        myfile << "Rvec: " << i+1 << " = \n" << tempRvec[i] << std::endl
                  << "Tvec: " << i+1 << " = \n" << tempTvec[i] << std::endl;
    }
    myfile.close();

}




void CameraCalibration::loadFileTranRot(std::string fileLocation){

}


void CameraCalibration::dataPacker(std::vector<std::vector<double>> dataArray)
{
    std::array<std::string,25> tempArray;

    for (int i = 0; i < dataArray.size() ; i+=25) {
        tempArray[0] += readVector(dataArray[i])+';';   //ActTCPPose#1
        tempArray[1] += readVector(dataArray[i+1])+';'; //ActTCPPose#2
        tempArray[2] += readVector(dataArray[i+2])+';'; //ActTCPPose#3
        tempArray[3] += readVector(dataArray[i+3])+';'; //ActTCPPose#4
        tempArray[4] += readVector(dataArray[i+4])+';'; //ActTCPPose#5
        tempArray[5] += readVector(dataArray[i+5])+';'; //ActTCPPose#6
        tempArray[6] += readVector(dataArray[i+6])+';'; //ActTCPPose#7
        tempArray[7] += readVector(dataArray[i+7])+';'; //ActTCPPose#8
        tempArray[8] += readVector(dataArray[i+8])+';'; //ActTCPPose#9
        tempArray[9] += readVector(dataArray[i+9])+';'; //ActTCPPose#10
        tempArray[10] += readVector(dataArray[i+10])+';'; //ActTCPPose#11
        tempArray[11] += readVector(dataArray[i+11])+';'; //ActTCPPose#12
        tempArray[12] += readVector(dataArray[i+12])+';'; //ActTCPPose#13
        tempArray[13] += readVector(dataArray[i+13])+';'; //ActTCPPose#14
        tempArray[14] += readVector(dataArray[i+14])+';'; //ActTCPPose#15
        tempArray[15] += readVector(dataArray[i+15])+';'; //ActTCPPose#16
        tempArray[16] += readVector(dataArray[i+16])+';'; //ActTCPPose#17
        tempArray[17] += readVector(dataArray[i+17])+';'; //ActTCPPose#18
        tempArray[18] += readVector(dataArray[i+18])+';'; //ActTCPPose#19
        tempArray[19] += readVector(dataArray[i+19])+';'; //ActTCPPose#20
        tempArray[20] += readVector(dataArray[i+20])+';'; //ActTCPPose#21
        tempArray[21] += readVector(dataArray[i+21])+';'; //ActTCPPose#22
        tempArray[22] += readVector(dataArray[i+22])+';'; //ActTCPPose#23
        tempArray[23] += readVector(dataArray[i+23])+';'; //ActTCPPose#24
        tempArray[24] += readVector(dataArray[i+24])+';'; //ActTCPPose#25

    }

    for (int i = 0; i < 25 ; ++i) {
        tempArray[i] = tempArray[i].substr(0, tempArray[i].length()-1);
    }


}

cv::Mat CameraCalibration::getAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles){

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix(cv::Mat(3,4, CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
//    std::cout << "rotCamerMatrix: " << "\n" << rotCamerMatrix << std::endl;
//    std::cout << "cameraMatrix: " << "\n" << cameraMatrix << std::endl;
//    std::cout << "rotMatrix: " << "\n" << rotMatrix << std::endl;
//    std::cout << "transVect: " << "\n" << transVect << std::endl;
//    std::cout << "rotMatrixX: " << "\n" << rotMatrixX << std::endl;
//    std::cout << "rotMatrixY: " << "\n" << rotMatrixY << std::endl;
//    std::cout << "rotMatrixZ: " << "\n" << rotMatrixZ << std::endl;
//    std::cout << "eulerAngles: " << "\n"  << eulerAngles << std::endl;
    return rotMatrix;
}

void CameraCalibration::writeFileTranRot4 (cv::Mat tempRvec){
    std::ofstream myfile;
    myfile.open ("../Detection/RotationTransposeData.txt", std::ios::app);
        myfile << "--------------------" << std::endl
               << tempRvec << std::endl;
    myfile.close();
}

void CameraCalibration::writeFileTranRot5 (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec){
    std::ofstream myfile;
    myfile.open ("../Detection/MarkertransposeData.txt", std::ios::app);
    for (size_t i = 0; i < tempRvec.size(); i++) {
        myfile << "Rvec: " << i+1 << " = \n" << tempRvec[i] << std::endl
                  << "Tvec: " << i+1 << " = \n" << tempTvec[i] << std::endl;
    }
    myfile.close();

}
