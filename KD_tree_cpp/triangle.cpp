#include "triangle.h"

Triangle::Triangle()
{}

Triangle::Triangle( QVector3Dd v0_i, QVector3Dd v1_i, QVector3Dd v2_i, QVector3Dd normal_i)
{
    // QVector 3D
    v0=v0_i;
    v1=v1_i;
    v2=v2_i;
    normal = normal_i;
}


QVector3Dd Triangle::getMidPoint()
{
   return (v0+v1+v2)/3;

}


