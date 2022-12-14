#ifndef FITPLANE3D_H
#define FITPLANE3D_H

#include <QVector>
#include <QVector3D>

class FitPlane3D
{
public:
    FitPlane3D(QVector<QVector3D> points_);
    void build();
    QVector3D getNormal();


protected:
    QVector<QVector3D> points;
    QVector3D plane_normal;



};

#endif // FITPLANE3D_H
