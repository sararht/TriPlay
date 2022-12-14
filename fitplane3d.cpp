#include "fitplane3d.h"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

FitPlane3D::FitPlane3D(QVector<QVector3D> points_)
{
    points = points_;
}

void FitPlane3D::build()
{

    // Copy coordinates to  matrix in Eigen format
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > coord(3, points.size());

    for (int i=0; i<points.size(); i++)
    {
        coord(0,i) = points[i].x();
        coord(1,i) = points[i].y();
        coord(2,i) = points[i].z();
    }

    // Calculate centroid
    float centroid_x = coord.row(0).mean();
    float centroid_y = coord.row(1).mean();
    float centroid_z = coord.row(2).mean();

    // Substract centroid
    coord.row(0).array() -= centroid_x;
    coord.row(1).array() -= centroid_y;
    coord.row(2).array() -= centroid_z;


    //SVD descomposition
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto plane_normal_ = svd.matrixU().rightCols<1>();

    plane_normal = QVector3D(plane_normal_.x(), plane_normal_.y(), plane_normal_.z());
}

QVector3D FitPlane3D::getNormal()
{
    return plane_normal;
}
