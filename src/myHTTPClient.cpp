#include "myHTTPClient.h"

MyHTTPClient::MyHTTPClient(std::string ip, int port)
{
    mHost = ip + ":";
    mHost += std::to_string(port);
    std::cout << "<HTTP CLIENT CREATED>" << std::endl;
    curlpp::Cleanup cleanup;
}

bool MyHTTPClient::begin()
{
    mRequest.reset();
    std::stringstream ss;
    std::string path = "/begin?";
    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path);
    mRequest.setOpt<curlpp::options::WriteStream>(&ss);
    mRequest.perform();
    std::string response;
    response = ss.str();
    if (response == "yes")
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Not in use
//void MyHTTPClient::sendTransformationMatrix(std::string tMatrix)
//{
//    mRequest.reset();
//    std::string path = "/tMatrix?";
//    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path + tMatrix);
//    mRequest.perform();
//}

void MyHTTPClient::sendCameraCalibrationMatrix(std::string ccMatrix)
{
    mRequest.reset();
    std::string path = "/ccMatrix?";
    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path + ccMatrix);
    mRequest.perform();
}

// Not in use
//void MyHTTPClient::sendPoints(std::array<std::string, 6> points)
//{
//    mRequest.reset();
//    std::string path = "/points?";
//    std::string variables = points[0] + "&" + points[1] + "&" +
//                            points[2] + "&" + points[3] + "&" +
//                            points[4] + "&" + points[5];
//    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path + variables);
//    std::cout << "URI: http://" + mHost + path + variables << std::endl;
//    mRequest.perform();
//}

void MyHTTPClient::sendStatus(std::string status)
{
    mRequest.reset();
    std::string path = "/status?";
    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path + status);
    mRequest.perform();
}

// Not in use
//void MyHTTPClient::sendEndOfThrow()
//{
//    mRequest.reset();
//    std::string path = "/end?";
//    mRequest.setOpt<curlpp::options::Url>("http://" + mHost + path);
//    mRequest.perform();
//}
