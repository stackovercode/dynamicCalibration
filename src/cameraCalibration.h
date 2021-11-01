#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "moveArm.h"
#include "camera.h"
#include <fstream>
#include "algorithm"
#include <vector>
#include <ostream>



class CameraCalibration : public Camera
{
public:
    CameraCalibration( CameraSettings& cameraSettings, int verticalIntersections = 5, int horizontalIntersections = 6,
                                                       int squareSize = 10, int numberOfCalibrationImages = 25);
    virtual ~CameraCalibration() = default;


    void initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);
    std::string readVector(std::vector<double> result);
    std::string cameraCalibrationToString();

    // Currently not in use:
    void dataPacker(std::vector<std::vector<double>>); //mangeler et client parameter adder når det kan testes.


private:
    std::vector<std::vector<double>> mRobotPose;

    int mNumberOfCalibrationImages;
    int mVerticalIntersections;
    int mHorizontalIntersections;
    int mSquareSize;
    std::string mTempPose;
    int mPositionStatus;
    std::vector<cv::Point3f> checkerboardWorld;
    std::vector<cv::Point2f> chessboardCorners;
    std::vector<std::vector<cv::Point2f>> chessboardCornersArray;
    std::vector<std::vector<cv::Point3f>> checkerboardWorldArray;
    std::vector<cv::Mat> rvecs, tvecs;

    std::vector<cv::Point2f> corners, imageFramePoints;
    std::vector<cv::Point3f> framePoints, boardPoints;
    cv::Mat mapX, mapY, imgUndistorted;

    cv::Mat mRvec = cv::Mat(cv::Size(3, 1), CV_64F);
    cv::Mat mTvec = cv::Mat(cv::Size(3, 1), CV_64F);
    std::vector<cv::Mat> mRvecArr, mTvecArr;
    cv::Mat gray;

    
    int mWidth;
    int mHeight;

    double mErrorRMS = 0;
    cv::Matx33f mCameraMatrix = cv::Matx33f(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> mDistortionCoefficient = {0, 0, 0, 0, 0}; // distortion coefficients

    // In use
    void action(Pylon::CInstantCamera& camera, ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);
    void collectCalibratingImages(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller);
    void drawChessboardCorners(std::vector<std::vector<cv::Point2f>>& chessboardCornersArray, bool flagShowImage = true);
    void generateCheckerboardWorld(std::vector<std::vector<cv::Point3f>>& checkerboardWorldArray );
    void calibrate(std::vector<std::vector<cv::Point2f>> const chessboardCornersArray, std::vector<std::vector<cv::Point3f>> const checkerboardWorldArray, bool flagShowImage = true );

    bool writeFile ();
    void writeFileRobotPoses(std::vector<std::vector<double>> robotPoses);
    void writeFileTranRot (std::vector<cv::Mat> tempRvec, std::vector<cv::Mat> tempTvec);
    void loadFileTranRot (std::string fileLocation);

};

#endif // CAMERACALIBRATION_H
