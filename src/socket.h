#ifndef SOCKET_H
#define SOCKET_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <QAbstractSocket>
#include <QDebug>
#include <QObject>
#include <QTcpSocket>

class Socket : public QObject
{
    Q_OBJECT
public:    
    explicit Socket(QObject *parent = nullptr);
    void Running();

signals:

public slots:
    void connected();
    void disconnected();

    void choicesProgram();
    void play();
    void Open();
    void running();

private:
    QTcpSocket *Client;

    int waitForReadyReadSec{5 * 1000}; // 6 min waiting
    bool programHasBeenLoaded{false};

    useconds_t mSeconds{1000000};
};

#endif // SOCKET_H
