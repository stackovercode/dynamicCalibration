#ifndef DETECTIONCHECKERBOARD_H
#define DETECTIONCHECKERBOARD_H
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "camera.h"
#include "moveRobot.h"
#include "cameraConfirguration.h"
#include "workspaceCalibration.h"
#include <fstream>
#include <opencv2/videoio.hpp>
#include <visp3/core/vpMath.h>


class DetectionCheckerboard : public Camera
{
public:


    DetectionCheckerboard( CameraConfirguration& cameraConfirguration, int verticalIntersections = 5, int horizontalIntersections = 6,
                                                       int squareSize = 10, int numberOfCalibrationImages = 25);
    virtual ~DetectionCheckerboard() = default;

    void initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6f jointBase, bool flagDetectMarker);
    void markerDet();
    cv::Mat getEulerAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles);

    std::string markerDetectionToString();

    cv::Vec6d mRobotPoint3d;
    std::vector<double> moveFrame;

    std::vector<cv::Mat> rvecs, tvecs;
    bool mainProcesState;
    std::vector<double> jointBaseFrame;

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
    cv::Mat inlier = cv::Mat(cv::Size(3, 1), CV_64F);

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

    cv::Mat myRotationMatrix = (cv::Mat_<double>(3,3));
    cv::Mat myNewRotationMatrix = (cv::Mat_<double>(3,3));
    cv::Vec3d eulerAngels;

    void getCalibrationData(std::string fileLocation);
    void action(Pylon::CInstantCamera& camera,  ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6f jointBase);

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
    void getSolvepnpRvecTvec(bool flagChangeInProcedureRotation);
    void writeFileTranRot2 (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec);
    void writeFileTranRot3 (cv::Mat tempRvec);

    bool writeFileTranRot (cv::Mat tempRvec, cv::Mat tempTvec);
    void detectImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller, cv::Vec6f jointBase);

};

#endif // DETECTIONCHECKERBOARD_H
