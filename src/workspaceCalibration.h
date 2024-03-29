#ifndef WORKSPACECALIBRATION_H
#define WORKSPACECALIBRATION_H
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "camera.h"
#include "cameraCalibration.h"
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <opencv2/calib3d.hpp>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationMatrix.h>

class WorkspaceCalibration
{
public:

    WorkspaceCalibration();

    ~WorkspaceCalibration();

    cv::Mat initialize(ur_rtde::RTDEReceiveInterface &reciver,ur_rtde::RTDEControlInterface &controller, double lengthXmm, double lengthYmm, cv::Vec6f robotJointAngels);

    std::string transformationMatrixToString();

    cv::Mat getTransformationFlange2EndEffector();
    cv::Mat getTransformationEndEffector2Camera();
    cv::Mat getTransformationCamera2EndEffector(int numbOfPose, int method);
    cv::Mat getRobotTransformationMatrix(cv::Vec6f robotJointAngles);
    cv::Mat getTransformationMatrixBase2Cam(cv::Vec6f robotJointAngles);
    cv::Mat getTransformationMatrixImage2Camera(cv::Mat rvec, cv::Mat tvec);
    cv::Mat getTransformationFlange2CameraHandEye(int numbOfPose, int method);
    cv::Mat getInversMatrix(cv::Mat);
    void vispHandEyeCalibration(bool flagChoice);
    vpHomogeneousMatrix testAfVisp(int numbOfPose);
    double lineLength(double sX, double sY, double eX, double eY);
    double getDistance2Object(cv::Point2f origo, cv::Point2f dia);
    double getPixelPermm(cv::Point2f origo, cv::Point2f dia);
    cv::Point2f vectorBetween2Points(cv::Point2f startPoint, cv::Point2f endPoint);
    void writeFileTranRot(std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec);
    void loadFileTranRot(std::string fileLocation);
    void loadFileRobotJoint(std::string fileLocation);
    void loadFileRobotTCP(std::string fileLocation);

    void loadFileImagePoints(std::string filepath);

protected:
    double mErrorRMS;
    cv::Matx33f mCameraMatrix;
    std::vector<double> mRotMatrix;
    std::vector<double> mTransMatrix;
    cv::Vec<float, 5> mDistortionCoefficient;
    cv::Point mOpticalCenter;
    cv::Mat mMapX, mMapY;
    std::array<cv::Point, 4> mWorkspaceCorners;
    cv::Vec<float, 2> mCenterPoint;
    cv::Vec<float, 2> mDiaPoint;
    cv::Matx31d mRvec;
    cv::Matx31d mTvec;

    std::vector<cv::Mat> mTcpPose;
    std::vector<cv::Mat> mJointPose;
    std::vector<cv::Mat> mR_target2cam;
    std::vector<cv::Mat> mT_target2cam;


    std::vector<cv::Vec3d> mCameraPoints;
    std::vector<cv::Vec3d> mRobotPoints;
    cv::Vec3d mCameraCenter;
    cv::Vec3d mRobotCenter;
    std::vector<cv::Vec3d> mCameraQ;
    std::vector<cv::Vec3d> mRobotQ;
    cv::Mat_<double> mHMatrix{cv::Mat_<double>::zeros(3, 3)};
    cv::Mat mS, mU, mV;
    cv::Mat_<double> mRotationMatrix{cv::Mat_<double>::zeros(3,3)};
    cv::Mat_<double> mTranslationMatrix{cv::Mat_<double>::zeros(1,3)};
    cv::Mat_<double> mTransformationMatrix{cv::Mat_<double>::zeros(4,4)};
    cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta);
    cv::Mat rvec2RotationMatrix(cv::Vec3f theta);

    void getCalibrationData(std::string fileLocation);
    void action(Pylon::CInstantCamera& camera,  ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);
    void writeFile ( double pixelRatioX, double pixelRatioY);

};

#endif // WORKSPACECALIBRATION_H
