#ifndef SENSOR_H
#define SENSOR_H

//#include <QPointF>
//#include <QMatrix3x3>
#include <QQuaternion>
#include <QVector>
//#include <QDebug>
#include <iostream>

//#include <CL/cl.hpp>
//#include <boost/compute/program.hpp>

//#include "kdtree_c.h"
#include "KD_tree_cpp/kdnode.h"
#include "qvector3dd.h"

#include <vtktools.h>

class sensor
{
public:
    sensor(){};
    sensor(QVector3Dd origin_, QQuaternion q_,double working_range_, double working_distance_, double resolution_, double FOV_, double uncertainty_);
   // sensor(QVector3Dd origin, double roll_, double pitch_, double yaw_, double range_, double resolution_, double FOV_, double uncertainty_);
    sensor(QVector3Dd origin, double roll_, double pitch_, double yaw_, double working_range_, double working_distance_, double resolution_, double FOV_, double uncertainty_);

    void getMeasurement(KDNode &tree, bool isMeasurement);
    void getMeasurementParalell(KDNode &tree, bool isMeasurement);
    void getMeasurementVTK(vtkSmartPointer<vtkOBBTree> tree, bool isMeasurement);

public:
    QVector<double> d_intersect;
    QVector<QVector3Dd> normal_intersect;

    QVector<double> distances_data;
    QVector<QVector3Dd> normal_data;
    QVector<QVector3Dd> sensor_data;
    QVector<QVector3Dd> sensor_data_error;
    QVector<QVector3Dd> sensor_data_points_real;
    QVector<QVector3Dd> sensor_data_points_real_error;

    QVector3Dd origin;
    double roll;
    double pitch;
    double yaw;
    double range;
    double working_range;
    double working_distance;
    double resolution;
    double FOV;
    double uncertainty;
    double x_fov;
    QQuaternion q_previous;
    double roll_prev;
    double pitch_prev;
    double yaw_prev;

    QQuaternion q;
    QMatrix3x3 rotMat;
    std::vector<double> rotMat_array;
    void updatePositionSensor(QVector3Dd origin_, double roll_, double pitch_, double yaw_);
    void updateCaracteristicsSensor(double w_range_, double w_distance_, double FOV_, double resolution_);

    bool hit(KDNode *node, Ray &ray);
    //cl::Platform plf;
    //cl::Context cntxt;
    //cl::Device device;

    int count = 0;


};

#endif // SENSOR_H

