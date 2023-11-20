#ifndef ROSPLUGIN_H
#define ROSPLUGIN_H

#include "rosplugin_global.h"
#include "TriPluginInterface/triplugininterface.h"

#include <ros/ros.h>
#include <QObject>

class ROSPlugin : public TriPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "simuInterface.TriPluginInterface" FILE "rosplugin.json")
    Q_INTERFACES(TriPluginInterface)

public:
    virtual void precalculate() override;
    virtual void calculate() override;
    virtual void postcalculate() override;
    virtual void initPlugin(int argc, char **argv) override;


private:
    ros::NodeHandle *_nh;
    ros::Publisher _pub;
    int _argc;
    char ** _argv;
};

#endif // ROSPLUGIN_H
