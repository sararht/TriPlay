#ifndef WEBSOCKETXMLSERVER_H
#define WEBSOCKETXMLSERVER_H

#include <QWebSocketServer>
#include <QWebSocket>
#include <QtXml>

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
