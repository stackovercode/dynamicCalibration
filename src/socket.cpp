#include "socket.h"

Socket::Socket(QObject *parent): QObject(parent) {
}

int usleep(useconds_t useconds);

void Socket::Running() {
    Client = new QTcpSocket(this);

    qDebug() << " -  Connecting to server...\n";

    usleep(1 * mSeconds);

    // QString IPv4{"169.254.89.190"};  // This is the IPv4 for the UR5 robot : rasp
    QString IPv4{"192.168.100.50"};       // This is the IPv4 for the UR5 robot : PC
    //QString IPv4{"localhost"};       // This is the localhost which is used for testing on YAT
    //QString IPv4{"127.0.0.1"};
    //quint16 Port {29999};               // This is the dashboard servers port

    quint16 Port {29999};

    Client->connectToHost(IPv4, Port);

    if(!Client->waitForConnected(60 * 1000)) {
        qDebug() << " -  Error when trying to connect:\n" << "\tError message:" <<  Client->errorString();

        exit(1);
    }
    connected();
}


void Socket::connected() {
    qDebug() << " -  Connection is established!";

    // Message from server "Connected: Universal Robots Dashboard Server"
    Client->waitForReadyRead(waitForReadyReadSec);
    qDebug() << "\t" << "Message from server: " << Client->readAll();

    usleep(1 * mSeconds);

    // Opens the robot
    Open();

    choicesProgram();
}


void Socket::Open() {
    qDebug() << "\n -  Robot start up\n";

    // Closing popup    Message from server "Closing popup"
    Client->write("Close popup\r\n");
    usleep(1 * mSeconds);

    // Brake release    Message from server "Brake releasing"
    Client->write("Brake release\r\n");
    usleep(1 * mSeconds);

    // Load Open.urp    Message from server "..."
    Client->write("Load /usbdisk/Open.urp\r\n");

    Client->write("hej");

    play();

    if (programHasBeenLoaded == false) {
        qDebug() << " -  Program could not be loaded";

        usleep(1 * mSeconds);
        Open();
    }

    // Start program    Message from server "..."
    Client->write("Play\r\n");
    usleep(1 * mSeconds);

    // Done with Open.urp
    running();

    qDebug() << " -  Program has been executed\n\n\n";
}


void Socket::running() {
    bool running{true};

    while (running == true) {
        Client->write("Running\r\n");
        Client->waitForReadyRead(waitForReadyReadSec);

        QByteArray qByteArray = Client->readAll();
        std::string message{static_cast<std::string>(qByteArray)};

        size_t found;
        if ((found = message.find("Program running: false")) != std::string::npos) {
            running = false;
        }
        usleep(1 * mSeconds);
    }
}


void Socket::play() {
    usleep(1 * mSeconds);

    Client->waitForReadyRead(waitForReadyReadSec);
    programHasBeenLoaded = true;

    Client->waitForReadyRead(waitForReadyReadSec);
    QByteArray qByteArray = Client->readAll();

    std::string message{static_cast<std::string>(qByteArray)};

    size_t found;
    if ((found = message.find("File not found")) != std::string::npos) {
        programHasBeenLoaded = false;
    } else {
        qDebug() << " -  Program has been loaded";

        usleep(1 * mSeconds);
        Client->write("Play\r\n");
    }

    usleep(1 * mSeconds);
}


// Choices in the program
void Socket::choicesProgram() {
    char choice;
    qDebug() << "\n";
    qDebug() << "__________________________________________________________________________________";
    qDebug() << " -  List of drinks: "
             << "\n" << "\t" << "Number 1: pose estimation"
             << "\n" << "\t" << "Number 2: _______________"
             << "\n"
             << "\n" << "   Else if shutdown 's'";
    qDebug() << " -  Chose command. For command enter the number, else enter the letter for shutdown: \n";
    std::cout << " >>>  ";
    std::cin >> choice;
    qDebug() << "__________________________________________________________________________________\n";


    switch (choice) {
      case '1':
        Client->write("Load /usbdisk/poseEstimation.urp\r\n");
        Client->write("popup Hej Anton!\n");
        play();

        if (programHasBeenLoaded == false) {
           qDebug() << " -  Program could not be loaded";
           usleep(1 * mSeconds);
           choicesProgram();
        }

        qDebug() << "\n -  Program is running";

        // Done with ___.urp file
        running();
        usleep(55 * mSeconds);
        break;
    case '2':

        break;
    case 'S': case 's':
        Client->write("Load /usbdisk/Close.urp\r\n");
        play();

        if (programHasBeenLoaded == false) {
            qDebug() << " -  Program could not be loaded";

            usleep(1 * mSeconds);
            choicesProgram();
        }

        // Done with ___.urp file
        running();

        Client->write("Shutdown\r\n");

        disconnected();
        break;

    default:
        qDebug() << "\n -  ('" << choice << "'), Is not a valid input, try again";
        choicesProgram();

        break;
}

    qDebug() << " -  Program has been executed\n\n\n";


    // Repeat program choice
    choicesProgram();
}


void Socket::disconnected() {
    qDebug() << "\n\n -  Disconnected!";

    exit(0);
}
