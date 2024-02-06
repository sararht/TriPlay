#ifndef MYINTERFACE_H
#define MYINTERFACE_H

#include <QObject>
#include <QThread>

class TriPluginInterface : public QObject
{

public:
    virtual ~TriPluginInterface() = default;
   // void run() override;
//    virtual void start();
    virtual void precalculate() = 0;
    virtual void calculate() = 0;
    virtual void postcalculate() = 0;
    virtual void initPlugin(int argc, char** argv, QVector<QVector3D> pos_sensor, QVector<QVector3D> rpy_sensor) = 0;
    virtual void setCustomFlag(bool flag) = 0;
    virtual void setPath(QString path) = 0;
    virtual void getTrajectory(QVector<QVector3D> &pos_sensor, QVector<QVector3D> &rpy_sensor) = 0;


private:
    QVector<QVector3D> _pos_sensor;
    QVector<QVector3D> _rpy_sensor;

};


Q_DECLARE_INTERFACE(TriPluginInterface, "simuInterface.TriPluginInterface")

#endif // MYINTERFACE_H
