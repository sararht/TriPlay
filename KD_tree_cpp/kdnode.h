#ifndef KDNODE_H
#define KDNODE_H

#include <iostream>
#include <math.h>
#include <QVector>
#include "qvector3dd.h"

#include "triangle.h"
#include "boundingbox.h"
#include <QSharedPointer>

struct Ray
{
    QVector3Dd origin;
    QVector3Dd dir;
};

inline bool BBoxRayIntersection(Ray &ray, BoundingBox &bbox)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    float invDx = 1/ray.dir.x();
    float invDy = 1/ray.dir.y();
    float invDz = 1/ray.dir.z();

    tmin = (bbox.minPos.x() - ray.origin.x()) * invDx;
    tmax = (bbox.maxPos.x() - ray.origin.x()) * invDx;
    tymin = (bbox.minPos.y() - ray.origin.y()) * invDy;
    tymax = (bbox.maxPos.y() - ray.origin.y()) * invDy;

    if (tmin > tmax) std::swap(tmin, tmax);
    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax))
           return false;
    if (tymin > tmin)
       tmin = tymin;
    if (tymax < tmax)
       tmax = tymax;

    tzmin = (bbox.minPos.z() - ray.origin.z())*invDz;
    tzmax = (bbox.maxPos.z() - ray.origin.z())*invDz;

    if (tzmin > tzmax)
        std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    return true;

}

inline bool TriangleRayIntersection(Ray &ray, Triangle &tri, double &distance)
{
    QVector3Dd edge1 = tri.v1 - tri.v0;
    QVector3Dd edge2 = tri.v2 - tri.v0;
    QVector3Dd tvec = ray.origin - tri.v0;

    QVector3Dd pvec = QVector3Dd::crossProduct(ray.dir, edge2);
    double det = QVector3Dd::dotProduct(edge1, pvec);

    double f = 1/det;
    double u = f * QVector3Dd::dotProduct(tvec,pvec);

    QVector3Dd q = QVector3Dd::crossProduct(tvec, edge1);
    double v = f * QVector3Dd::dotProduct(ray.dir,q);

    if (fabs(det) <= 0.00001 )
    {
        distance = -1;
        return false;
    }
    else if (u < 0.0 || u > 1.0)
    {
        distance = -1;
        return false;
    }
    else if (v < 0.0 || u+v > 1.0)
    {
        distance = -1;
        return false;
    }

    else
    {
        QVector3Dd qvec = QVector3Dd::crossProduct(tvec, edge1);
        distance = QVector3Dd::dotProduct(edge2,qvec)*f;

        return true;
    }
}

inline QVector3Dd rotate_point(double* sensor_T, QVector3Dd point)
{
    double point_aux[3] = {point.x(), point.y(), point.z()};

    double result_aux[4] = {sensor_T[0]*point_aux[0] + sensor_T[1]*point_aux[1] + sensor_T[2]*point_aux[2] ,
                         sensor_T[3]*point_aux[0] + sensor_T[4]*point_aux[1] + sensor_T[5]*point_aux[2] ,
                         sensor_T[6]*point_aux[0] + sensor_T[7]*point_aux[1] + sensor_T[8]*point_aux[2],
                         };

    QVector3Dd result = {result_aux[0] , result_aux[1] , result_aux[2] };

    return result;
}



class KDNode
{
public:
    KDNode();
    void delete_KDNode();

    QSharedPointer<KDNode> left;
    QSharedPointer<KDNode> right;
    std::vector<Triangle*> triangles;
    BoundingBox bbox;
    QSharedPointer<KDNode> build(std::vector<Triangle*> &tris, int depth);
  //  void SAH();

};

#endif // KDNODE_H
