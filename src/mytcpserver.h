#ifndef MYTCPSERVER_H
#define MYTCPSERVER_H

#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <QDebug>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <string>
#include <sys/stat.h>
#include <unistd.h>


class MyTcpServer : public QObject
{
    Q_OBJECT
public:
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::duration<float> fsec;
    explicit MyTcpServer(QObject *parent = nullptr);

signals:

public slots:
    void newConnection();

private:
    /** Gets the current date used for the logfile */
    inline std::string getCurrentDateTime(std::string currentDateTime);

    /** Checks if the logfile for the current date exist */
    inline bool exist(const std::string& name);

    /** Create a daily logfile, and handle the message wich are send to the log */
    inline void dayLogFile(std::string logMsg);

    /** Send the physical output message to 'Command' */
    inline void physicalOutput(std::string message);

    /* Variables used in the program */
    QTcpServer *mServer;
    QTcpSocket *socket;
    useconds_t mSeconds{1000000};
    int mActivation{0};
    int mFlag{-1};
    quint16 mPort;
    bool added;
    size_t found;
    std::string msg;
    std::string messagesCMD[3] {"Pose", "Close server", "Terminate server"};
    std::string programs[4] {"System: Open", "System: Close", "Pose", "The program has been executed"};
    int programTimes[12]{0};

    std::chrono::_V2::system_clock::time_point mTimerStart, mTimerStop;
};

#endif // MYTCPSERVER_H
