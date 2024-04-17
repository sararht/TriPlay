#ifndef TCPXMLSERVER_H
#define TCPXMLSERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>
#include <QDebug>
#include <QDomDocument>
#include <QVector3D>
#include "sensor.h"
#include "qvector3dd.h"
#include "KD_tree_cpp/kdnode.h"
#include "rendervtk.h"
#include <QTime>

class TcpXmlServer : public QObject
{
    Q_OBJECT
public:
    explicit TcpXmlServer(quint16 port, sensor* sensor_model, KDNode* tree, renderVTK* renderer_vtk, QObject *parent = nullptr)
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
            _sensor_model = sensor_model;
            _tree = tree;
            _renderer_vtk = renderer_vtk;
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
        QTime startTime = QTime::currentTime();

       QByteArray data = socket->readAll();
       // qDebug() << "Received:" << data;

        // Parsear el mensaje XML
        QDomDocument doc;
        if (!doc.setContent(data)) {
            qWarning() << "Error al parsear el mensaje XML";
            return;
        }

        // Obtener el elemento raíz
        QDomElement root = doc.documentElement();

        // Obtener los elementos de posición y orientación
        QDomElement positionElement = root.firstChildElement("position");
        QDomElement orientationElement = root.firstChildElement("orientation");
        QDomElement parametersElement = root.firstChildElement("parameters");

        // Obtener los valores de posición y orientación como cadenas de caracteres
        QString positionStr = positionElement.text();
        QString orientationStr = orientationElement.text();
        QString workingRangeStr = parametersElement.firstChildElement("working_range").text();
        QString workingDistanceStr = parametersElement.firstChildElement("working_distance").text();
        QString resolutionStr = parametersElement.firstChildElement("resolution").text();
        QString fovStr = parametersElement.firstChildElement("fov").text();


        // Dividir las cadenas en componentes
        QStringList positionList = positionStr.split(",");
        QStringList orientationList = orientationStr.split(",");

        float workingRange = workingRangeStr.toFloat();
        float workingDistance = workingDistanceStr.toFloat();
        float resolution = resolutionStr.toFloat();
        float fov = fovStr.toFloat();

//        qInfo() << "Working Range: " << workingRange;
//         qInfo() << "Working Distance: " << workingDistance;
//         qInfo() << "Resolution: " << resolution;
//         qInfo() << "FOV: " << fov;

        // Crear objetos QVector3D para posición y orientación
        if (positionList.size() == 3 && orientationList.size() == 3) {
            QVector3D position(positionList[0].toFloat(), positionList[1].toFloat(), positionList[2].toFloat());
            QVector3D orientation(orientationList[0].toFloat(), orientationList[1].toFloat(), orientationList[2].toFloat());

            // Ahora tienes los objetos QVector3D, puedes usarlos como necesites
            // Por ejemplo, puedes pasarlos a una función para su procesamiento posterior
          //  procesarPosicionYorientacion(position, orientation);
//            qInfo() << "Pos: " << position;
//            qInfo() << "orientation: " << orientation;

            //Crear sensor model
            QVector3Dd origin(position.x(),position.y(),position.z());
            _sensor_model->updatePositionSensor(origin, orientation.x(), orientation.y(), orientation.z());
            _sensor_model->updateCaracteristicsSensor(workingRange, workingDistance,fov,resolution);
            _renderer_vtk->updateRendered(*_sensor_model);
            _sensor_model->getMeasurement(*_tree, true);



        } else {
            qWarning() << "Error al dividir las cadenas de posición u orientación";
        }


        // Aquí procesarías el XML recibido y generarías una respuesta
        // Crear una cadena para almacenar los datos serializados
        QString mensaje="";

        mensaje.append("sensor_data:");

        // Convertir cada elemento del vector a una cadena y agregarlo al mensaje
        for (int i=0; i<_sensor_model->sensor_data.size();i++)
        {
            mensaje.append(QString::number(_sensor_model->sensor_data[i].z())).append(",");
        }

        mensaje.chop(1);
        mensaje.append(";");  // Separador entre tipos de datos

        mensaje.append("normal_data:");
        for (int i=0; i<_sensor_model->sensor_data.size();i++)
        {
            mensaje.append(QString::number(_sensor_model->normal_data[i].x())).append(",");
            mensaje.append(QString::number(_sensor_model->normal_data[i].y())).append(",");
            mensaje.append(QString::number(_sensor_model->normal_data[i].z())).append(",");
        }

        mensaje.chop(1);
        mensaje.append(";");  // Separador entre tipos de datos
        mensaje.append("pc_data:");
        for (int i=0; i<_sensor_model->sensor_data.size();i++)
        {
            mensaje.append(QString::number(_sensor_model->sensor_data_points_real[i].x())).append(",");
            mensaje.append(QString::number(_sensor_model->sensor_data_points_real[i].y())).append(",");
            mensaje.append(QString::number(_sensor_model->sensor_data_points_real[i].z())).append(",");
        }

        mensaje.chop(1);
        mensaje.append("END");

        qInfo() << mensaje.size();

        QTime endTime = QTime::currentTime();
        int timeDifference = startTime.msecsTo(endTime);
        qDebug() << "Tiempo de ejecución:" << timeDifference << "milisegundos";

        socket->write(mensaje.toUtf8());
    }



private:
    QTcpServer *tcpServer;
    sensor* _sensor_model;
    KDNode* _tree;
    renderVTK* _renderer_vtk;

};


#endif // TCPXMLSERVER_H
