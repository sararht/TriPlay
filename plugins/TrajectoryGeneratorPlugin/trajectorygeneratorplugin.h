#ifndef TRAJECTORYGENERATORPLUGIN_H
#define TRAJECTORYGENERATORPLUGIN_H

#include "trajectorygeneratorplugin_global.h"
#include "TriPluginInterface/triplugininterface.h"

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <opencv2/opencv.hpp>


struct pointTraj
{
    QVector3D positionXYZ;
    QVector3D orientationRPY;
};

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

    virtual void setCustomFlag(bool isFirstIteration) override;
    virtual void setPath(QString path) override;
    virtual void setROIS(QVector3D minPosBB, QVector3D maxPosBB) override;

    virtual void getTrajectory(QVector<QVector3D> &pos_sensor, QVector<QVector3D> &rpy_sensor, double &fov, double &vel, double &frames, int &resolution, double &uncertainty) override;
    virtual double costFunction() override;

private:
   QVector<QVector3D> _pos_sensor;
   QVector<QVector3D> _rpy_sensor;

   QVector<pointTraj> _traj_simple;
   QVector<pointTraj> _traj_interpolated;
   QVector<QVector3D> _pointcloud;
   QVector<QVector3D> _normal_map;
   QVector<QVector3D> _normal_scan_map;
   QVector<QVector3D> _scan_image;

   double _vel=0;
   double _fov=0;
   double _resolution=0;
   int _frames=0;
   double _uncertainty = 0;

   QVector<QVector<float>> _density_map;
   cv::Mat _density_map_raw;
   cv::Mat _normal_scan_image;

   bool _isFirstIteration;
   bool _isROI = false;

   QVector<QVector3D> _ROIs;
   QVector<pointTraj> _traj_simple_new;
   QVector<pointTraj> _traj_interpolated_new;

   QString _path;

};

#endif // TRAJECTORYGENERATORPLUGIN_H
