#ifndef WEBSOCKETXMLSERVER_H
#define WEBSOCKETXMLSERVER_H

#include <QWebSocketServer>
#include <QWebSocket>
#include <QtXml>
#include <QVector3D>
#include <QDebug>

class WebSocketXMLServer : public QObject
{
    Q_OBJECT
public:
    explicit WebSocketXMLServer(quint16 port, QObject *parent = nullptr)
        : QObject(parent)
        , m_pWebSocketServer(new QWebSocketServer(QStringLiteral("XML WebSocket Server"),
                                                  QWebSocketServer::NonSecureMode, this))
    {
        if (m_pWebSocketServer->listen(QHostAddress::Any, port))
        {
            connect(m_pWebSocketServer, &QWebSocketServer::newConnection,
                    this, &WebSocketXMLServer::onNewConnection);
            qDebug() << "WebSocket server started on port" << port;
        }
    }

private slots:
    void onNewConnection()
    {
        QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

        connect(pSocket, &QWebSocket::textMessageReceived,
                this, &WebSocketXMLServer::processTextMessage);

        connect(pSocket, &QWebSocket::disconnected,
                this, &WebSocketXMLServer::socketDisconnected);
    }

    void processTextMessage(QString message)
    {
        QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
        QDomDocument doc;
        if (doc.setContent(message))
        {
            // Aquí procesas el XML. El siguiente es solo un ejemplo.
            QDomElement root = doc.documentElement();

            // Obtener los elementos de posición y orientación
            QDomElement positionElement = root.firstChildElement("position");
            QDomElement orientationElement = root.firstChildElement("orientation");

            // Obtener los valores de posición y orientación como cadenas de caracteres
            QString positionStr = positionElement.text();
            QString orientationStr = orientationElement.text();

            // Dividir las cadenas en componentes
            QStringList positionList = positionStr.split(",");
            QStringList orientationList = orientationStr.split(",");

            // Crear objetos QVector3D para posición y orientación
            if (positionList.size() == 3 && orientationList.size() == 3) {
                QVector3D position(positionList[0].toFloat(), positionList[1].toFloat(), positionList[2].toFloat());
                QVector3D orientation(orientationList[0].toFloat(), orientationList[1].toFloat(), orientationList[2].toFloat());

                // Imprimir la posición y orientación
                qInfo() << "Posición: " << position;
                qInfo() << "Orientación: " << orientation;
            } else {
                qWarning() << "Error al dividir las cadenas de posición u orientación";
            }

            QString response = "Recibido XML con raíz: " + root.tagName();
            if (pClient)
            {
                pClient->sendTextMessage(response);
            }
        }
        else
        {
            if (pClient)
            {
                pClient->sendTextMessage("Error al procesar XML");
            }
        }
    }

    void socketDisconnected()
    {
        QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
        if (pClient)
        {
            pClient->deleteLater();
        }
    }

private:
    QWebSocketServer *m_pWebSocketServer;
};


#endif // WEBSOCKETXMLSERVER_H
