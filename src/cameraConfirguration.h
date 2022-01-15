#ifndef CAMERACONFIRGURATION_H
#define CAMERACONFIRGURATION_H
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <opencv2/opencv.hpp>

class CameraConfirguration
{
public:
    CameraConfirguration(int focalLength = 12, int exposure = 30000 , std::pair<double, double> sensorSize = {6.6, 4.1}, cv::Size resolution = {1920, 1200} )
        : mFocalLength{focalLength}, mExposure{exposure}, mSensorSize{sensorSize}, mResolution{resolution}{

    }

    int getExposure() const{
    return mExposure;
    }

    int getFocalLength() const{
    return mFocalLength;
    }

    std::pair<double, double> getSensorSize() const{
    return mSensorSize;
    }

    cv::Size getResolution() const{
    return mResolution;
    }

private:
    int mExposure;
    int mFocalLength;
    std::pair<double,double> mSensorSize;
    cv::Size mResolution;

    void initialize();
};

#endif // CAMERACONFIRGURATION_H
