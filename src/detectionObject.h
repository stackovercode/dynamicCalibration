#ifndef DETECTIONOBJECT_H
#define DETECTIONOBJECT_H
#include "camera.h"
#include "cameraSettings.h"
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>


// Class not currently in use
class DetectionObject
{
public:
    DetectionObject( CameraSettings& cameraSettings, cv::Scalar lowerHSVBoundary={35, 150, 60},
                    cv::Scalar upperHSVBoundary={60, 255, 255},
                    int objectRadiusWorld=20,
                    int hough_dp=2, int hough_upperThres=100,
                    int hough_centerThres=10, int hough_minRadius=15,
                    int hough_maxRadius=20);

    bool locate(cv::Vec3d& vecObjectPosition, cv::Mat undistortedImage, cv::Point2f pointOne = {0.0,0.0}, cv::Point2f pointTwo = {0.0,0.0}, cv::Point2f pointThree = {0.0,0.0}, cv::Point2f pointFour = {0.0,0.0});


    int getObjectRadiusWorld() const;
    int getHough_dp() const;
    int getHoughUpperThres() const;
    int getHoughCenterThres() const;
    int getHoughMinRadius() const;
    int getHoughMaxRadius() const;
    cv::Scalar getLowerBoundary() const;
    cv::Scalar getUpperBoundary() const;

private:
    CameraSettings mCameraSettings;

    void getCalibrationData(std::string fileLocation);
//    double              mErrorRMS;
    cv::Matx33f         mCameraMatrix;
//    cv::Vec<float, 5>   mDistortionCoefficient;
    cv::Point           mOpticalCenter;
    cv::Mat             mMapX, mMapY;

    int mObjectRadiusWorld,
        mHough_dp,
        mHoughUpperThres,
        mHoughCenterThres,
        mHoughMinRadius,
        mHoughMaxRadius;
    cv::Scalar
        mLowerBoundary,
        mUpperBoundary;


};

#endif // DETECTIONOBJECT_H
