#include "mytcpserver.h"

MyTcpServer::MyTcpServer(QObject *parent) :
    QObject(parent) {
    // create a timer and server
    mServer = new QTcpServer(this);
    qDebug() << "Opening server...";

    // Start timer
    mTimerStart = Time::now();
    usleep(2 * mSeconds);

    // whenever a user connects, it will emit signal
    connect(mServer, SIGNAL(newConnection()), this, SLOT(newConnection()));

    mServer->waitForNewConnection(50000);   // Behøves ikke
    mPort = 50005;
    //mPort = 29999;

    if (!mServer->listen(QHostAddress::Any, mPort)) {
        qDebug() << "Server could not start";
    } else {
        qDebug() << "Server started on port" << mPort << "and using IPv" << QHostAddress::Any;
    }
}

int usleep(useconds_t useconds);

void MyTcpServer::newConnection() {
    // Grab the socket
    socket = mServer->nextPendingConnection();

    socket->waitForBytesWritten(10000);
    socket->waitForReadyRead(60000 * 120);     // (60000 = 1 min) * 120 minuter)
    QByteArray qByteArray = socket->readAll();
    qDebug() << "Message: " << qByteArray;
    added = false;

    std::string message{static_cast<std::string>(qByteArray)};
    if ((found = message.find(messagesCMD[0])) != std::string::npos) {
        if (mFlag != 0) {
            socket->write("POSE\r\n");
            physicalOutput(messagesCMD[0]);
        } else {socket->write("GRIPPER IS ALREADY CLOSED");}

    } else if ((found = message.find(messagesCMD[2])) != std::string::npos) {
        if (mFlag != 1) {
            socket->write("CLOSING SERVER\r\n");
            physicalOutput(messagesCMD[1]);
        } else {socket->write("SERVER IS ALREADY CLOSED");}

    } else if ((found = message.find(messagesCMD[2])) != std::string::npos) {
        socket->write("TERMINATE SERVER");

        /* End timer, and create 'timeSpend' */
        mTimerStop = Time::now();
        fsec fs{mTimerStop - mTimerStart};
        ms d{std::chrono::duration_cast<ms>(fs)};
        double timeWithTwoDigitDoubleSec{(std::floor(static_cast<double>(std::stold(std::to_string(d.count())))) / 100) / 10};
        timeWithTwoDigitDoubleSec = std::floor(timeWithTwoDigitDoubleSec);
        double timeWithTwoDigitDoubleMin{std::floor(timeWithTwoDigitDoubleSec / 60)};
        double timeWithTwoDigitDoubleHour{std::floor(timeWithTwoDigitDoubleMin / 60)};
        timeWithTwoDigitDoubleSec -= timeWithTwoDigitDoubleMin * 60;
        timeWithTwoDigitDoubleMin -= timeWithTwoDigitDoubleHour * 60;
        std::string timeWithTwoDigitStringSec{std::to_string(static_cast<int>(timeWithTwoDigitDoubleSec))};
        std::string timeWithTwoDigitStringMin{std::to_string(timeWithTwoDigitDoubleMin)};
        std::string timeWithTwoDigitStringHour{std::to_string(timeWithTwoDigitDoubleHour)};
        timeWithTwoDigitStringSec.erase(std::remove(timeWithTwoDigitStringSec.begin(), timeWithTwoDigitStringSec.end(), '0'), timeWithTwoDigitStringSec.end());
        timeWithTwoDigitStringMin.erase(std::remove(timeWithTwoDigitStringMin.begin(), timeWithTwoDigitStringMin.end(), '0'), timeWithTwoDigitStringMin.end());
        timeWithTwoDigitStringHour.erase(std::remove(timeWithTwoDigitStringHour.begin(), timeWithTwoDigitStringHour.end(), '0'), timeWithTwoDigitStringHour.end());
        timeWithTwoDigitStringMin.erase(std::remove(timeWithTwoDigitStringMin.begin(), timeWithTwoDigitStringMin.end(), '.'), timeWithTwoDigitStringMin.end());
        timeWithTwoDigitStringHour.erase(std::remove(timeWithTwoDigitStringHour.begin(), timeWithTwoDigitStringHour.end(), '.'), timeWithTwoDigitStringHour.end());

        (timeWithTwoDigitDoubleMin < 1)? timeWithTwoDigitStringMin = "0": timeWithTwoDigitStringMin;
        (timeWithTwoDigitDoubleHour < 1)? timeWithTwoDigitStringHour = "0": timeWithTwoDigitStringHour;

        std::string timeSpend{timeWithTwoDigitStringHour + " hour and " + timeWithTwoDigitStringMin + " minutes and " + timeWithTwoDigitStringSec + " seconds."};

        // Write to log file
        // Basic layout with number of grips, active time
        msg += "The total number of grips is " + std::to_string(mActivation) + ", done in this program."
                + "\n\t\t\tThe active time for this server is " + timeSpend
                + "\n\t\t\tThe activated programs are:";

        // Write if 'System' programs has been executed, and if so, create a new line after
        for (int index{0}; index < 2; index++) {
            if (programTimes[index] > 0) {msg += "\n\t\t\t\t" + programs[index] + " was executed";}
        }
        if ((programTimes[0] > 0) || (programTimes[1] > 0)) {msg += '\n';}

        // Write which 'Drink' programs has been executed, and times for each program
        for (int index{2}; index < 11; index++) {
            if (programTimes[index] > 0) {
                msg += "\n\t\t\t\t" + programs[index] + " was executed " + std::to_string(programTimes[index]);
                msg += (programTimes[index] == 1)? " time":" times";
            }
        }

        // Number of programs started and ended
        msg += '\n';
        int times{0};
        for (int index{2}; index < 11; index++) {times += programTimes[index];}
        msg += "\n\t\t\t\tThe amount of programs which has been started: " + std::to_string(times);
        msg += "\n\t\t\t\tThe amount of programs which has been executed: " + std::to_string(programTimes[11] - 1);
        msg += "\n\t\t\t\tThe amount of failures: " + std::to_string(times - (programTimes[11] - 1));

        dayLogFile(msg);


        // The server waits for a bit 00after the client has shutdown, before it itself shuts down.
        // It does so to counter a problem with the addreess being in use after the server otherwise closes down normally.
        // Also the robots program needs to be shut down promptly afer its connection is shut down.
        usleep(2 * mSeconds);
        socket->close();

        exit(0); // Closing the program

    } else {
        for (int index{0}; index < 12; index++) {
            if ((found = message.find(programs[index])) != std::string::npos) {
                if (added == false) {
                    programTimes[index] += 1;
                    socket->write("PROGRAM ADDED TO LIST");
                    added = true;
                } else {socket->write("INVALID INPUT");}
            }
        }

    }

    socket->close();
}



//***************************************************************************************
//*         Log file section                                                            *
//***************************************************************************************
std::string MyTcpServer::getCurrentDateTime(std::string currentDateTime) {
    time_t now = time(nullptr);
    struct tm  tstruct;
    char buf[256];
    tstruct = *localtime(&now);
    if(currentDateTime == "now")
        strftime(buf, sizeof(buf), "%d-%m-%Y %X", &tstruct);
    else if(currentDateTime == "date")
        strftime(buf, sizeof(buf), "%d-%m-%Y", &tstruct);

    return std::string(buf);
}

bool MyTcpServer::exist(const std::string &name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

void MyTcpServer::dayLogFile(std::string logMsg) {
    std::string filePathLog = "C:/Users/kaspe/Desktop/TcpServerGripper/build-TcpServer-Desktop_Qt_5_12_0_MinGW_64_bit-Debug/release/Log_" + getCurrentDateTime("date") + ".txt";
    //std::string filePathLog = "/home/pi/TcpServerGripper/build-TcpServer-Desktop_Qt_5_12_0_MinGW_64_bit-Debug/release/Log_" + getCurrentDateTime("date") + ".txt";
    std::string text;
    if (!exist(filePathLog)) {text = "This program contains the log for the gripper.\n";}
    std::string now = getCurrentDateTime("now");
    std::ofstream ofstream(filePathLog.c_str(), std::ios_base::out | std::ios_base::app);
    ofstream << text << "\n\n" << now << '\t' << logMsg;
    ofstream.close();
}



//***************************************************************************************
//*         Send message to 'Command'                                                   *
//***************************************************************************************
void MyTcpServer::physicalOutput(std::string message) {
    if (message == "Close gripper") {
       //digitalWrite(1, HIGH);

        mFlag = 0;
        mActivation++;

    } else if (message == "Open gripper") {
        //digitalWrite(1, LOW);

        mFlag = 1;

    } else if (message == "Close server") {
        //digitalWrite(1, LOW);

        mFlag = 2;
    } else if (message == "Pose") {
        std::cout << "Pose printed" << std::endl;

        mFlag = 2;
    }
}
