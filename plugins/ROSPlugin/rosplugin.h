#ifndef ROSPLUGIN_H
#define ROSPLUGIN_H

#include "rosplugin_global.h"
#include "TriPluginInterface/triplugininterface.h"

#include <ros/ros.h>

#include <QObject>
#include <QVector>
#include <QVector3D>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

class ROSPlugin : public TriPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "simuInterface.TriPluginInterface" FILE "rosplugin.json")
    Q_INTERFACES(TriPluginInterface)

public:
    virtual void precalculate() override;
    virtual void calculate() override;
    virtual void postcalculate() override;
    virtual void initPlugin(int argc, char **argv,  QVector<QVector3D> pos_sensor,  QVector<QVector3D> rpy_sensor) override;


private:
    ros::NodeHandle *_nh;
    ros::Publisher _pub;
    int _argc;
    char ** _argv;

    QVector<QVector3D> _pos_sensor;
    QVector<QVector3D> _rpy_sensor;

   // moveit::planning_interface::MoveGroupInterface _move_group();

};

#endif // ROSPLUGIN_H
