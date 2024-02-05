#ifndef TCPXMLSERVER_H
#define TCPXMLSERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>
#include <QDebug>

class TcpXmlServer : public QObject
{
    Q_OBJECT
public:
    explicit TcpXmlServer(quint16 port, QObject *parent = nullptr)
        : QObject(parent), tcpServer(new QTcpServer(this))
    {
        connect(tcpServer, &QTcpServer::newConnection, this, &TcpXmlServer::onNewConnection);
        if (!tcpServer->listen(QHostAddress::Any, port))
        {
            qDebug() << "Server could not start";
        }
        else
        {
            qDebug() << "Server started on port" << port;
        }
    }

private slots:
    void onNewConnection()
    {
        QTcpSocket *socket = tcpServer->nextPendingConnection();
        connect(socket, &QTcpSocket::readyRead, this, [=]() { onReadyRead(socket); });
        connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
    }

    void onReadyRead(QTcpSocket *socket)
    {
        QByteArray data = socket->readAll();
        qDebug() << "Received:" << data;

        // Aquí procesarías el XML recibido y generarías una respuesta
        QString response = "Respuesta del servidor";
        socket->write(response.toUtf8());
    }

private:
    QTcpServer *tcpServer;
};


#endif // TCPXMLSERVER_H
