#ifndef DETECTIONMARKER_H
#define DETECTIONMARKER_H
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "camera.h"
#include "moveArm.h"
#include "cameraSettings.h"
#include "workspaceCalibration.h"
#include <fstream>


class DetectionMarker : public Camera
{
public:

    DetectionMarker( CameraSettings& cameraSettings, int verticalIntersections = 5, int horizontalIntersections = 6,
                                                       int squareSize = 10, int numberOfCalibrationImages = 20);
    virtual ~DetectionMarker() = default;

    void initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, bool flagDetectMarker);
    void markerDet();

    std::string markerDetectionToString();

    cv::Vec6d mRobotPoint3d;
    std::vector<double> moveFrame;

    std::vector<cv::Mat> rvecs, tvecs;


private:
    int mNumberOfCalibrationImages;
    int mVerticalIntersections;
    int mHorizontalIntersections;
    int mSquareSize;
    bool flagDetect;


    double mErrorRMS;
    cv::Matx33f mCameraMatrix;
    cv::Vec<float, 5> mDistortionCoefficient;
    cv::Point mOpticalCenter;
    cv::Mat mMapX, mMapY;
    std::array<cv::Point, 4> mWorkspaceCorners;

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

    void getCalibrationData(std::string fileLocation);
    void action(Pylon::CInstantCamera& camera,  ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);

    double lineLength(double sX, double sY, double eX, double eY);
    double getDistance2Object(cv::Point2f origo, cv::Point2f dia);
    double getPixelPermm(cv::Point2f origo, cv::Point2f dia);
    cv::Point2f centerPoint(cv::Vec4f line1, cv::Vec4f line2);
    cv::Point2f vectorBetween2Points(cv::Point2f startPoint, cv::Point2f endPoint);
    cv::Point3f vectorfromframeCPtoCBCp(cv::Point2i checkerBoardCP, cv::Point2i frameCP, double pixelPmm, double distanceObj);
    bool isRotationMatrix(cv::Mat &R);
    cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
    cv::Vec3f rpy2rv(cv::Vec3f rpy);
    cv::Vec3f ToRotVector(cv::Vec3f rpy);
    void getSolvepnpRvecTvec();
    void writeFileTranRot2 (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec);

    bool writeFileTranRot (cv::Mat tempRvec, cv::Mat tempTvec);
    void detectImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);

};

#endif // DETECTIONMARKER_H
