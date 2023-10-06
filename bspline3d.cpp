#include <QVector>
#include <QVector3D>
#include <cmath>
#include <iostream>

#include "bspline3d.h"

BSpline3D::BSpline3D(const QVector<QVector3D>& controlPoints, int degree) {
    m_controlPoints = controlPoints;
    m_degree = degree;
    m_knotVector.resize(controlPoints.size() + degree + degree - 1);
    for (int i = 0; i < m_knotVector.size(); ++i) {
      m_knotVector[i] = (i - degree + 1) / (controlPoints.size() - 1);
    }

  }

float factorial(float n) {
    if (n == 0) {
      return 1;
    } else {
      return n * factorial(n - 1);
    }
}

QVector3D BSpline3D::evaluate(float t) const {

   // std::cout <<"-----------------------------" <<std::endl;

        // Calcula los parámetros de Bernstein para el punto de interpolación
        QVector<float> bernsteinParameters;
   //     std::cout << " T: , " << t << std::endl;

        for (int i = 0; i <= m_degree+1; ++i) {
            float param;
             if (m_degree - i >= 0) {
               param = factorial(m_degree) / factorial(i) / factorial(m_degree - i) * pow(t, i) * pow(1 - t, m_degree - i);
             } else {
               param = 0;
             }

             bernsteinParameters.push_back(param);
         //   std::cout << i << ", t, " << t << " PARAM: " << param << std::endl;
        }

        // Calcula el punto de interpolación
        QVector3D point;
        for (int i = 0; i < m_degree+1; ++i) {
          point += m_controlPoints[i] * bernsteinParameters[i];
        //  std::cout << point.x() << " " << point.y() << " " << point.z() << std::endl;
        }
      //  std::cout <<"-----------------------------" <<std::endl;

        return point;
 }

QVector<QVector3D> BSpline3D::getPoints(float step) const
{
    QVector<QVector3D> points;
    for (float t = 0; t <= 1; t += step) {
      points.push_back(evaluate(t));
    }

    return points;
}
