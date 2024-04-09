#include "sensor.h"


//int aaa =  2048*294929;
//std::vector<double>xcoor_(aaa);

void resizeVector(QVector<QVector3Dd> v_in, QVector<double> &v_out)
{
    for (int i=0; i<v_in.size();i++)
    {
        v_out.push_back(v_in[i].x());
        v_out.push_back(v_in[i].y());
        v_out.push_back(v_in[i].z());

    }
}
/*
sensor::sensor(QVector3Dd origin_, double roll_, double pitch_, double yaw_, double range_, double resolution_, double FOV_, double uncertainty_)
{   
    origin = origin_;
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
    range = range_;
    resolution = resolution_;
    FOV = FOV_;
    uncertainty = uncertainty_;

    x_fov = range*tan(FOV/2)*2;

    q = QQuaternion::fromEulerAngles(pitch, yaw, roll);
    rotMat = q.toRotationMatrix(); //CREO QUE ESTÁ MAL -->CON LA DE SCIPY ME IBA BIEN

    rotMat_array.clear();
    for (int i=0; i<9; i++)
        rotMat_array.push_back(rotMat.data()[i]);

}
*/

sensor::sensor(QVector3Dd origin_, QQuaternion q_,double working_range_, double working_distance_, double resolution_, double FOV_, double uncertainty_)
{
    origin = origin_;
    q = q_;
    QVector3D euler = q.toEulerAngles();
    roll = euler.z();
    pitch = euler.x();
    yaw = euler.y();

    working_range = working_range_;
    working_distance = working_distance_;
    range = working_distance + working_range/2;
    resolution = resolution_;
    FOV = FOV_;
    uncertainty = uncertainty_;

    x_fov = working_distance*tan(FOV/2)*2;

    rotMat = q.toRotationMatrix(); //CREO QUE ESTÁ MAL -->CON LA DE SCIPY ME IBA BIEN

    rotMat_array.clear();
    for (int i=0; i<9; i++)
        rotMat_array.push_back(rotMat.data()[i]);

    q_previous = QQuaternion(0,0,0,1);
    roll_prev = 0;
    pitch_prev = 0;
    yaw_prev = 0;
}

sensor::sensor(QVector3Dd origin_, double roll_, double pitch_, double yaw_,double working_range_, double working_distance_, double resolution_, double FOV_, double uncertainty_)
{
    origin = origin_;
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
    working_range = working_range_;
    working_distance = working_distance_;
    range = working_distance + working_range/2;
    resolution = resolution_;
    FOV = FOV_;
    uncertainty = uncertainty_;

    x_fov = working_distance*tan(FOV/2)*2;

    q = QQuaternion::fromEulerAngles(pitch, yaw, roll);
    rotMat = q.toRotationMatrix(); //CREO QUE ESTÁ MAL -->CON LA DE SCIPY ME IBA BIEN

    rotMat_array.clear();
    for (int i=0; i<9; i++)
        rotMat_array.push_back(rotMat.data()[i]);

    q_previous = QQuaternion(0,0,0,1);
    roll_prev = 0;
    pitch_prev = 0;
    yaw_prev = 0;

}

//inline void
//checkErr(cl_int err, const char * name)
//{
//    if (err != CL_SUCCESS) {
//        std::cerr << "ERROR: " << name
//                 << " (" << err << ")" << std::endl;
//        exit(EXIT_FAILURE);
//    }
//}

//deb

#include <vtkSTLReader.h>
#include <vtkModifiedBSPTree.h>
bool sensor::hit(KDNode *node, Ray &ray)
{
    if (BBoxRayIntersection(ray, node->bbox))
    {
        bool hit_tri = false;

        if(node->left->triangles.size() > 0 || node->right->triangles.size() > 0) //>0
        {
            bool hit_left = hit(node->left.get(), ray);
            bool hit_right = hit(node->right.get(), ray);

            return  hit_left || hit_right;
        }
        else
        {
            //QVector<double> distances;
            for (uint i=0; i<node->triangles.size(); i++)
            {
                double t_aux;
                if(TriangleRayIntersection(ray, *node->triangles[i], t_aux))
                {
                    hit_tri = true;
                    d_intersect.push_back(t_aux);
                    normal_intersect.push_back(node->triangles[i]->normal);
                }
            }


            if (hit_tri)
            {
                 return true;
            }
            return false;
        }
    }
    return false;
}

void sensor::getMeasurementVTK(vtkSmartPointer<vtkOBBTree> tree, bool isMeasurement = true)
{
    sensor_data.clear();
    sensor_data_error.clear();
    sensor_data_points_real.clear();
    distances_data.clear();

    sensor_data.shrink_to_fit();
    sensor_data_error.shrink_to_fit();
    sensor_data_points_real.shrink_to_fit();
    distances_data.shrink_to_fit();

    double x = range*tan(FOV/2);
    QVector3Dd B{-x,0,range};
    QVector3Dd C{x,0,range};

    double rot_mat[9]= {rotMat.data()[0], rotMat.data()[3], rotMat.data()[6],
                       rotMat.data()[1], rotMat.data()[4], rotMat.data()[7],
                       rotMat.data()[2], rotMat.data()[5], rotMat.data()[8]};

    double rot_mat_trans[9]= {rot_mat[0],rot_mat[3],rot_mat[6],
                             rot_mat[1],rot_mat[4],rot_mat[7],
                             rot_mat[2],rot_mat[5],rot_mat[8]};

    QVector3Dd B_world = rotate_point(rot_mat, B);
    QVector3Dd C_world = rotate_point(rot_mat, C);

    B= B_world;
    C= C_world;

    double cte_x = (C.x()-B.x())/resolution;
    double cte_y = (C.y()-B.y())/resolution;
    double cte_z = (C.z()-B.z())/resolution;

    QVector3Dd origin_aux_v = rotate_point(rot_mat_trans,QVector3Dd{origin.x(),origin.y(),origin.z()});
    QVector3Dd origin_aux(-origin_aux_v.x(), -origin_aux_v.y(), -origin_aux_v.z());


    double originVtk[3] = {origin.x(), origin.y(), origin.z()};
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkIdList> cellsIds;

    int N=resolution;
    for (int id_ray=0; id_ray<N; id_ray++)
    {
        double end[3] = {B.x() + cte_x*id_ray + origin.x(), B.y() + cte_y*id_ray + origin.y(), B.z() + cte_z*id_ray + origin.z()};

        if(vtkTools::getIntersectRayObbTree(tree,originVtk,end,points,cellsIds))
        {
            //Calcular distancias y coger el punto con menor distancia
            int id_min=0;
            double min = std::sqrt(vtkMath::Distance2BetweenPoints(originVtk, points->GetData()->GetTuple3(0)));

            QVector3Dd normal(0,1,0);

            for (int i=0; i<points->GetNumberOfPoints(); i++)
            {

//                std::cout << points->GetData()->GetTuple3(i)[0] <<", "
//                          << points->GetData()->GetTuple3(i)[1] <<", "
//                          << points->GetData()->GetTuple3(i)[2]<< std::endl;
            }

            // Punto 3D
            double maxE = uncertainty*0.001/2;
            double minE = -maxE;
            double error = ((double) rand()*(maxE-minE)/(double)RAND_MAX-minE);

            QVector3Dd point(points->GetData()->GetTuple3(id_min)[0],points->GetData()->GetTuple3(id_min)[1],points->GetData()->GetTuple3(id_min)[2]);
            QVector3Dd point_s_ = rotate_point(rot_mat_trans, QVector3Dd{point.x(), point.y(), point.z()});
            QVector3Dd point_s = QVector3Dd(point_s_.x(), point_s_.y(), point_s_.z()) + origin_aux;

            distances_data.push_back(min);
            normal_data.push_back(normal);

            sensor_data_points_real.push_back(point);
            sensor_data.push_back(point_s);
            sensor_data_error.push_back(QVector3Dd(point_s.x(),point_s.y(),point_s.z()+error));

        }

        else
        {
            distances_data.push_back(0);
            normal_data.push_back(QVector3Dd(0,0,0));

            sensor_data.push_back(QVector3Dd(0,0,0));
            sensor_data_error.push_back(QVector3Dd(0,0,0));
            sensor_data_points_real.push_back(QVector3Dd(0,0,0));
            sensor_data_points_real_error.push_back(QVector3Dd(0,0,0));

        }
    }
}

//NO FUNCIONA----
void sensor::getMeasurementParalell(vtkSmartPointer<vtkOBBTree> tree, bool isMeasurement=true)
{
    sensor_data.clear();
    sensor_data_error.clear();
    sensor_data_points_real.clear();
    distances_data.clear();

    sensor_data.shrink_to_fit();
    sensor_data_error.shrink_to_fit();
    sensor_data_points_real.shrink_to_fit();
    distances_data.shrink_to_fit();

    sensor_data.reserve(resolution);
    sensor_data_error.reserve(resolution);
    sensor_data_points_real.reserve(resolution);
    distances_data.reserve(resolution);
    normal_data.reserve(resolution);
    sensor_data_points_real_error.reserve(resolution);

    double x = range*tan(FOV/2);
    QVector3Dd B{-x,0,range};
    QVector3Dd C{x,0,range};

    double rot_mat[9]= {rotMat.data()[0], rotMat.data()[3], rotMat.data()[6],
                       rotMat.data()[1], rotMat.data()[4], rotMat.data()[7],
                       rotMat.data()[2], rotMat.data()[5], rotMat.data()[8]};

    double rot_mat_trans[9]= {rot_mat[0],rot_mat[3],rot_mat[6],
                             rot_mat[1],rot_mat[4],rot_mat[7],
                             rot_mat[2],rot_mat[5],rot_mat[8]};

    QVector3Dd B_world = rotate_point(rot_mat, B);
    QVector3Dd C_world = rotate_point(rot_mat, C);

    B= B_world;
    C= C_world;

    double cte_x = (C.x()-B.x())/resolution;
    double cte_y = (C.y()-B.y())/resolution;
    double cte_z = (C.z()-B.z())/resolution;

    QVector3Dd origin_aux_v = rotate_point(rot_mat_trans,QVector3Dd{origin.x(),origin.y(),origin.z()});
    QVector3Dd origin_aux(-origin_aux_v.x(), -origin_aux_v.y(), -origin_aux_v.z());

    double originVtk[3] = {origin.x(), origin.y(), origin.z()};


    int N=resolution;
    #pragma omp parallel for
    for (int id_ray=0; id_ray<N; id_ray++)
    {
       double end[3] = {B.x() + cte_x*id_ray + origin.x(), B.y() + cte_y*id_ray + origin.y(), B.z() + cte_z*id_ray + origin.z()};

        vtkSmartPointer<vtkPoints> points;
        vtkSmartPointer<vtkIdList> cellsIds;
        if(vtkTools::getIntersectRayObbTree(tree,originVtk,end,points,cellsIds))
        {

            int id_min=0;
            double min = std::sqrt(vtkMath::Distance2BetweenPoints(originVtk, points->GetData()->GetTuple3(0)));

            QVector3Dd normal(0,1,0);

            // Punto 3D
            double maxE = uncertainty*0.001/2;
            double minE = -maxE;
            double error = ((double) rand()*(maxE-minE)/(double)RAND_MAX-minE);;
            QVector3Dd point(points->GetData()->GetTuple3(id_min)[0],points->GetData()->GetTuple3(id_min)[1],points->GetData()->GetTuple3(id_min)[2]);
            QVector3Dd point_s_ = rotate_point(rot_mat_trans, QVector3Dd{point.x(), point.y(), point.z()});
            QVector3Dd point_s = QVector3Dd(point_s_.x(), point_s_.y(), point_s_.z()) + origin_aux;

            distances_data[id_ray] = (min);
            normal_data[id_ray] = (normal);
            sensor_data[id_ray] = point_s;
            sensor_data_points_real[id_ray] = point;
            sensor_data_error[id_ray] = QVector3Dd(point_s.x(),point_s.y(),point_s.z()+error);
            sensor_data_points_real_error[id_ray] = QVector3Dd(point.x(), point.y()+error, point.z());

        }

        else
        {
            distances_data[id_ray] = 0;
            normal_data[id_ray] = QVector3Dd(0,0,0);
            sensor_data[id_ray] = QVector3Dd(0,0,0);
            sensor_data_error[id_ray] = QVector3Dd(0,0,0);
            sensor_data_points_real[id_ray] = QVector3Dd(0,0,0);
            sensor_data_points_real_error[id_ray] = QVector3Dd(0,0,0);
        }
    }

}

void sensor::getMeasurement(KDNode &tree, bool isMeasurement=true)
{

    sensor_data.clear();
    normal_data.clear();
    sensor_data_error.clear();
    sensor_data_points_real.clear();
    distances_data.clear();

    d_intersect.clear();
    normal_intersect.clear();
    distances_data.clear();
    sensor_data_points_real_error.clear();

    sensor_data.shrink_to_fit();
    normal_data.shrink_to_fit();
    sensor_data_error.shrink_to_fit();
    sensor_data_points_real.shrink_to_fit();
    distances_data.shrink_to_fit();

    d_intersect.shrink_to_fit();
    normal_intersect.shrink_to_fit();
    distances_data.shrink_to_fit();
    sensor_data_points_real_error.shrink_to_fit();

    double x = range*tan(FOV/2);
    QVector3Dd B{-x,0,range};
    QVector3Dd C{x,0,range};

    double rot_mat[9]= {rotMat.data()[0], rotMat.data()[3], rotMat.data()[6],
                       rotMat.data()[1], rotMat.data()[4], rotMat.data()[7],
                       rotMat.data()[2], rotMat.data()[5], rotMat.data()[8]};

    double rot_mat_trans[9]= {rot_mat[0],rot_mat[3],rot_mat[6],
                             rot_mat[1],rot_mat[4],rot_mat[7],
                             rot_mat[2],rot_mat[5],rot_mat[8]};

    QVector3Dd B_world = rotate_point(rot_mat, B);
    QVector3Dd C_world = rotate_point(rot_mat, C);

    B= B_world;
    C= C_world;

    double cte_x = (C.x()-B.x())/resolution;
    double cte_y = (C.y()-B.y())/resolution;
    double cte_z = (C.z()-B.z())/resolution;

    QVector3Dd origin_aux_v = rotate_point(rot_mat_trans,QVector3Dd{origin.x(),origin.y(),origin.z()});
    QVector3Dd origin_aux(-origin_aux_v.x(), -origin_aux_v.y(), -origin_aux_v.z());


    int N=resolution;
    for (int id_ray=0; id_ray<N; id_ray++)
    {
        //Create Ray
        Ray ray;
        ray.origin = origin;
        QVector3Dd aux = QVector3Dd(B.x() + cte_x*id_ray + origin.x(), B.y() + cte_y*id_ray + origin.y(), B.z() + cte_z*id_ray + origin.z());
        ray.dir = origin*-1+aux;


        if (hit(&tree,ray))
        {
            double min = d_intersect[0];
            QVector3Dd normal = normal_intersect[0];
//            if(d_intersect.size()==1)
//                std::cout <<"PROBLEMAAAAA"<<std::endl;

            for (int j=0; j<d_intersect.size();j++)
            {
                if (d_intersect[j] < min)
                {
                    min = d_intersect[j];
                    normal = normal_intersect[j];
                }
            }


            distances_data.push_back(min);
            normal_data.push_back(normal);

            d_intersect.clear();
            normal_intersect.clear();

            d_intersect.shrink_to_fit();
            normal_intersect.shrink_to_fit();


            // Punto 3D
            double maxE = uncertainty*0.001/2;
            double minE = -maxE;
            double error = ((double) rand()*(maxE-minE)/(double)RAND_MAX-minE);;
            QVector3Dd point = origin + ray.dir*min;
            QVector3Dd point_s_ = rotate_point(rot_mat_trans, QVector3Dd{point.x(), point.y(), point.z()});
            QVector3Dd point_s = QVector3Dd(point_s_.x(), point_s_.y(), point_s_.z()) + origin_aux;

            if(isMeasurement && point_s.z()>range)
            {
                distances_data.push_back(0);
                normal_data.push_back(QVector3Dd(0,0,0));

                sensor_data.push_back(QVector3Dd(0,0,0));
                sensor_data_error.push_back(QVector3Dd(0,0,0));
                sensor_data_points_real.push_back(QVector3Dd(0,0,0));
                sensor_data_points_real_error.push_back(QVector3Dd(0,0,0));

                d_intersect.clear();
                d_intersect.shrink_to_fit();
                continue;
            }


            sensor_data.push_back(point_s);
            sensor_data_points_real.push_back(point);
            sensor_data_error.push_back(QVector3Dd(point_s.x(),point_s.y(),point_s.z()+error));
            sensor_data_points_real_error.push_back(QVector3Dd(point.x(), point.y()+error, point.z()));

        }

        else
        {
            distances_data.push_back(0);
            normal_data.push_back(QVector3Dd(0,0,0));

            sensor_data.push_back(QVector3Dd(0,0,0));
            sensor_data_error.push_back(QVector3Dd(0,0,0));
            sensor_data_points_real.push_back(QVector3Dd(0,0,0));
            sensor_data_points_real_error.push_back(QVector3Dd(0,0,0));

            d_intersect.clear();
            d_intersect.shrink_to_fit();

        }

    }

}



void sensor::updatePositionSensor(QVector3Dd origin_, double roll_, double pitch_, double yaw_)
{
    q_previous = q;
    roll_prev = roll;
    pitch_prev = pitch;
    yaw_prev = yaw;

    origin = origin_;
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
//----
    /*
    double roll_d, pitch_d, yaw_d;
    roll_d = -roll_prev + roll;
    pitch_d = -pitch_prev + pitch;
    yaw_d = -yaw_prev + yaw;

    QQuaternion q_rot;
    if(roll_d != 0)
        q_rot = QQuaternion::fromAxisAndAngle(0,0,1,roll_d);
    else if(pitch_d != 0)
        q_rot = QQuaternion::fromAxisAndAngle(1,0,0,pitch_d);
    else if(yaw_d != 0)
        q_rot = QQuaternion::fromAxisAndAngle(0,1,0,yaw_d);
    else
        q_rot = QQuaternion::fromEulerAngles(0,0,0);

    QQuaternion q_ = q_previous*q_rot;

    q = q_;
    */
//------
    q = QQuaternion::fromEulerAngles(pitch,yaw,roll);
    rotMat = q.toRotationMatrix();

    rotMat_array.clear();
    for (int i=0; i<9; i++)
        rotMat_array.push_back(rotMat.data()[i]);


}

void sensor::updateCaracteristicsSensor(double w_range_, double w_distance_, double FOV_, double resolution_)
{
    working_range = w_range_;
    working_distance = w_distance_;
    range = w_distance_ + w_range_/2;
    FOV = FOV_;
    resolution = resolution_;

    x_fov = w_distance_*tan(FOV/2)*2;

}
