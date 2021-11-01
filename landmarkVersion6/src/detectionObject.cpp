#include "detectionObject.h"


DetectionObject::DetectionObject(CameraSettings& cameraSettings, cv::Scalar lowerHSVBoundary,
                                 cv::Scalar upperHSVBoundary,
                                 int objectRadiusWorld,
                                 int hough_dp, int hough_upperThres,
                                 int hough_centerThres, int hough_minRadius,
                                 int hough_maxRadius)
    : mCameraSettings{cameraSettings},
      mLowerBoundary{lowerHSVBoundary},
      mUpperBoundary{upperHSVBoundary},
      mObjectRadiusWorld{objectRadiusWorld},
      mHough_dp{hough_dp},
      mHoughUpperThres{hough_upperThres},
      mHoughCenterThres{hough_centerThres},
      mHoughMinRadius{hough_minRadius},
      mHoughMaxRadius{hough_maxRadius}
{
    ////////// Redding ind calibrationData //////////////
    getCalibrationData("../Detection/CalibrationData.txt");
    /////////// Creating rectifying Maps //////////////
    //Copy in
    //cv::initUndistortRectifyMap(mCameraMatrix, mDistortionCoefficient, cv::Matx33f::eye(), mCameraMatrix, mCameraSettings.getResolution(), CV_32FC1, mMapX, mMapY);

}

bool DetectionObject::locate(cv::Vec3d& vecObjectPosition, cv::Mat undistortedImage, cv::Point2f pointOne, cv::Point2f pointTwo, cv::Point2f pointThree, cv::Point2f pointFour){

    std::vector<cv::Point2f> corners;
    line(undistortedImage, pointOne, pointThree, cv::Scalar(0,255,0), 4, cv::LINE_AA);
    line(undistortedImage, pointTwo, pointThree, cv::Scalar(0,255,0), 4, cv::LINE_AA);
    line(undistortedImage, pointFour, pointOne, cv::Scalar(0,255,0), 4, cv::LINE_AA);
    line(undistortedImage, pointFour, pointTwo , cv::Scalar(0,255,0), 4, cv::LINE_AA);

    int width = undistortedImage.size().width * 60/100;
    int height = undistortedImage.size().height * 60/100;

    cv::Size dimension (width, height);
    cv::resize(undistortedImage,undistortedImage,dimension);
    cv::imshow("hhh", undistortedImage);

    return true;
}

void DetectionObject::getCalibrationData(std::string fileLocation)
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
        ifs >> tempFloat; //mErrorRMS;

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
            //mDistortionCoefficient[index] = tempFloat;
            ifs >> trashChar;
        }

        mOpticalCenter = {static_cast<int>(mCameraMatrix.val[2]),static_cast<int>(mCameraMatrix.val[5])};

/*        std::cout << "[ Info ]: The following data have been read in... \n"
                  << "\nerrorRMS:\n " << mErrorRMS
                  << "\nCamaraMatrix:\n " << mCameraMatrix
                  << "\nDistortionCoeficient:\n " << mDistortionCoefficient
                  << "\nOpticalCenter:\n " << mOpticalCenter << std::endl;*/
    }
}

int DetectionObject::getObjectRadiusWorld() const
{
    return mObjectRadiusWorld;
}

int DetectionObject::getHough_dp() const
{
    return mHough_dp;
}

int DetectionObject::getHoughUpperThres() const
{
    return mHoughUpperThres;
}

int DetectionObject::getHoughCenterThres() const
{
    return mHoughCenterThres;
}

int DetectionObject::getHoughMinRadius() const
{
    return mHoughMinRadius;
}

int DetectionObject::getHoughMaxRadius() const
{
    return mHoughMaxRadius;
}

cv::Scalar DetectionObject::getLowerBoundary() const
{
    return mLowerBoundary;
}

cv::Scalar DetectionObject::getUpperBoundary() const
{
    return mUpperBoundary;
}
