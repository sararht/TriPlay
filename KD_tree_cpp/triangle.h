#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <QVector3D>
#include "qvector3dd.h"

/*
struct vector3D
{
    float x_;
    float y_;
    float z_;

    float x(){return x_;};
    float y(){return y_;};
    float z(){return z_;};
};


inline QVector3D difVector3D(vector3D v1, vector3D v2)
{
    QVector3D diff(v1.x()-v2.x(), v1.y()-v2.y(), v1.z()-v2.z());
    return diff;
}
*/

class Triangle
{
public:
    Triangle();
    Triangle( QVector3Dd v0_i, QVector3Dd v1_i, QVector3Dd v2_i, QVector3Dd normal_i);
    QVector3Dd getMidPoint();

    QVector3Dd v0;
    QVector3Dd v1;
    QVector3Dd v2;
    QVector3Dd normal;



};

#endif // TRIANGLE_H
