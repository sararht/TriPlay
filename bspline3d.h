#ifndef BSPLINE3D_H
#define BSPLINE3D_H


// BSpline3D.hpp

#include <QVector>
#include <QVector3D>

class BSpline3D {
public:
  BSpline3D(const QVector<QVector3D>& controlPoints, int degree);

  QVector3D evaluate(float t) const;
  QVector<QVector3D> getPoints(float step) const;


private:
  QVector<QVector3D> m_controlPoints;
  int m_degree;
  QVector<float> m_knotVector;
};

#endif // BSPLINE3D_H
