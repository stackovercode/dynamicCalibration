#ifndef MYHTTPCLIENT_H
#define MYHTTPCLIENT_H
#include <iostream>
#include <sstream>
#include <curlpp/cURLpp.hpp>
#include <curl/curl.h>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>


class MyHTTPClient
{
public:
    MyHTTPClient(std::string ip, int port);

    bool begin();
    void sendTransformationMatrix(std::string tMatrix);
    void sendCameraCalibrationMatrix(std::string ccMatrix);
    void sendPoints(std::array<std::string, 6> points);
    void sendStatus(std::string status);
    void sendEndOfThrow();

private:
    std::string mHost;
    curlpp::Easy mRequest;
};

#endif // MYHTTPCLIENT_H
