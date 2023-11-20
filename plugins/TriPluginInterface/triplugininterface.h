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
    virtual void initPlugin(int argc, char** argv) = 0;

};


Q_DECLARE_INTERFACE(TriPluginInterface, "simuInterface.TriPluginInterface")

#endif // MYINTERFACE_H
