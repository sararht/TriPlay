#ifndef MYINTERFACE_H
#define MYINTERFACE_H

#include <QObject>

class myInterface
{
public:
    virtual ~myInterface() = default;
    virtual void precalculate() = 0;
    virtual void calculate() = 0;
    virtual void postcalculate() = 0;

private:
    QVector<QVector3D> _pos_sensor;
    QVector<QVector3D> _rpy_sensor;

};


Q_DECLARE_INTERFACE(myInterface, "simuInterface.myInterface")

#endif // MYINTERFACE_H
