#include "camera.h"

Camera::Camera(CameraSettings cammeraSettings)
    : mCamerasettings{cammeraSettings}
{
}

void Camera::initialize(ur_rtde::RTDEReceiveInterface &reciver, ur_rtde::RTDEControlInterface &controller)
{
    Pylon::PylonAutoInitTerm autoInitTerm;
    try {
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        GenApi::INodeMap& nodemap= camera.GetNodeMap();
        camera.Open();
        camera.MaxNumBuffer = 5;

        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Auto exposure disabled." << std::endl
                      << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
        }

        std::cout << "Trying to set exposure to: " << mCamerasettings.getExposure() << std::endl;
        std::cout << "Exposure changed from: " << exposureTime->GetValue();
        if(exposureTime.IsValid())
        {
            if(mCamerasettings.getExposure() >= exposureTime->GetMin() && mCamerasettings.getExposure()<= exposureTime->GetMax())
            {
                exposureTime->SetValue(mCamerasettings.getExposure());
                std::cout << " To: " << mCamerasettings.getExposure() << std::endl;
            }
            else if(mCamerasettings.getExposure() <= exposureTime->GetMin())
            {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << "To: " << exposureTime->GetValue() << std::endl;
                std::cout << ">> Exposure was set to the minimum available value." << std::endl;
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMax());
                std::cout << "To: " << exposureTime->GetValue() << std::endl;
                std::cout << ">> Exposure was set to the maximum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }

        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        std:: cout << "Starting the grabing proces..." << std::endl;

        action(camera, reciver, controller);


    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "[ Fail ]: An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        exit(-1);
    }
}
