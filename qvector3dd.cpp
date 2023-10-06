#include "qvector3dd.h"
#include <string>
#include <math.h>
#include <iostream>

QVector3Dd::QVector3Dd()
{

}

QVector3Dd::QVector3Dd(double x, double y, double z)
{
    v[0]=x;
    v[1]=y;
    v[2]=z;
}

// Operators
QVector3Dd QVector3Dd::operator+(const QVector3Dd &v2)
{
    return QVector3Dd(v[0] + v2.v[0], v[1] + v2.v[1], v[2] + v2.v[2]);
}

QVector3Dd QVector3Dd::operator-(const QVector3Dd &v2) const
{
    return QVector3Dd(v[0] - v2.v[0], v[1] - v2.v[1], v[2] - v2.v[2]);
}

QVector3Dd QVector3Dd::operator/(const double &divisor)
{
    return QVector3Dd(v[0]/divisor, v[1]/divisor, v[2]/divisor);
}

QVector3Dd QVector3Dd::operator*(const double &factor)
{
    return QVector3Dd(v[0]*factor, v[1]*factor, v[2]*factor);
}



// Functions

QVector3D QVector3Dd::toQVector3D()
{
    return QVector3D(float(v[0]), float(v[1]), float(v[2]));
}



double QVector3Dd::length() const
{
    double len = double(v[0]) * double(v[0]) +
                 double(v[1]) * double(v[1]) +
                 double(v[2]) * double(v[2]);
    return std::sqrt(len);

}

QVector3Dd QVector3Dd::normalized() const
{
    QVector3Dd n(v[0]/length(),v[1]/length(),v[2]/length());
    return n;
}
double QVector3Dd::dotProduct(const QVector3Dd& v1, const QVector3Dd& v2)
{
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

QVector3Dd QVector3Dd::crossProduct(const QVector3Dd& v1, const QVector3Dd& v2)
{
    return QVector3Dd(v1.y() * v2.z() - v1.z() * v2.y(),
                      v1.z() * v2.x() - v1.x() * v2.z(),
                      v1.x() * v2.y() - v1.y() * v2.x());
}


QVector3Dd QVector3Dd::abs()
{
    return QVector3Dd(fabs(v[0]),fabs(v[1]),fabs(v[2]));
}





QVector3Dd &QVector3Dd::operator+=(const QVector3Dd &vector)
{
    this->setX(this->x() + vector.x());
    this->setY(this->y() + vector.y());
    this->setZ(this->z() + vector.z());

    return *this;
}

QVector3Dd &QVector3Dd::operator-=(const QVector3Dd &vector)
{
    this->setX(this->x() - vector.x());
    this->setY(this->y() - vector.y());
    this->setZ(this->z() - vector.z());
    return *this;
}

QVector3Dd &QVector3Dd::operator/=(double divisor)
{    
    this->setX(this->x() / divisor);
    this->setY(this->y() / divisor);
    this->setZ(this->z() / divisor);

    return *this;
}

QVector3Dd &QVector3Dd::operator*=(const QVector3Dd &vector)
{
    this->setX(this->x() * vector.x());
    this->setY(this->y() * vector.y());
    this->setZ(this->z() * vector.z());

    return *this;
}

