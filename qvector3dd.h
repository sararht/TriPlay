#ifndef QVECTOR3DD_H
#define QVECTOR3DD_H

#include<QVector3D>

class QVector3Dd
{
public:
    QVector3Dd();
    QVector3Dd(double x, double y, double z);

    double x() const { return v[0]; };
    double y() const { return v[1]; };
    double z() const { return v[2]; };

    void setX(float aX) { v[0] = aX; }
    void setY(float aY) { v[1] = aY; }
    void setZ(float aZ) { v[2] = aZ; }

    QVector3Dd operator+(const QVector3Dd &v2);
    QVector3Dd operator-(const QVector3Dd &v2);
    QVector3Dd operator/(const double &divisor);
    QVector3Dd operator*(const double &factor);

    QVector3Dd abs();

    double length() const;
    QVector3Dd normalized() const;


    static double dotProduct(const QVector3Dd& v1, const QVector3Dd& v2); //In Qt 6 convert to inline and constexpr
    static QVector3Dd crossProduct(const QVector3Dd& v1, const QVector3Dd& v2); //in Qt 6 convert to inline and constexpr

    QVector3D toQVector3D();



  /*  QVector3Dd &operator+=(const QVector3Dd &vector);
    QVector3Dd &operator-=(const QVector3Dd &vector);
    QVector3Dd &operator/=(double divisor);
    QVector3Dd &operator*=(double factor);
*/

private:
    double v[3];

};

#endif // QVECTOR3DD_H
