#ifndef TRAJECTORYGENERATORPLUGIN_H
#define TRAJECTORYGENERATORPLUGIN_H

#include "trajectorygeneratorplugin_global.h"
#include "TriPluginInterface/triplugininterface.h"

#include <QObject>
#include <QVector>
#include <QVector3D>


class TrajectoryGeneratorPlugin : public TriPluginInterface
{  
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "simuInterface.TriPluginInterface" FILE "trajectorygeneratorplugin.json")
    Q_INTERFACES(TriPluginInterface)
public:
    virtual void precalculate() override;
    virtual void calculate() override;
    virtual void postcalculate() override;
    virtual void initPlugin(int argc, char **argv,  QVector<QVector3D> pos_sensor,  QVector<QVector3D> rpy_sensor) override;
};

#endif // TRAJECTORYGENERATORPLUGIN_H
