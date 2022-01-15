#ifndef CAMERA_H
#define CAMERA_H
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "cameraConfirguration.h"

class Camera
{
public:
    Camera(CameraConfirguration);
    virtual ~Camera() = default;

    void initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6f jointBase);

protected:
    CameraConfirguration mCameraConfirguration;

    virtual void action(Pylon::CInstantCamera& camera, ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6f jointBase) = 0;
};

#endif // CAMERA_H
