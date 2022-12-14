
#include "kdtree_c.h"

float3 rotate_point(float* sensor_T, float3 point)
{
    float point_aux[3] = {point.x, point.y, point.z};

    float result_aux[4] = {sensor_T[0]*point_aux[0] + sensor_T[1]*point_aux[1] + sensor_T[2]*point_aux[2] ,
                         sensor_T[3]*point_aux[0] + sensor_T[4]*point_aux[1] + sensor_T[5]*point_aux[2] ,
                         sensor_T[6]*point_aux[0] + sensor_T[7]*point_aux[1] + sensor_T[8]*point_aux[2],
                         };

    float3 result = {result_aux[0] , result_aux[1] , result_aux[2] };

    return result;
}


float dotProduct(float3 vect_A, float3 vect_B)
{
    float product = vect_A.x * vect_B.x + vect_A.y * vect_B.y + vect_A.z * vect_B.z ;
    return product;
}

float3 crossProduct(float3 vect_A, float3 vect_B)
{
    float3 cross_P;

    cross_P.x = vect_A.y * vect_B.z - vect_A.z * vect_B.y;
    cross_P.y = vect_A.z * vect_B.x - vect_A.x * vect_B.z;
    cross_P.z = vect_A.x * vect_B.y - vect_A.y * vect_B.x;

    return cross_P;
}


kernel void frst_prog(__global float* v0, __global float* v1, __global float* v2,
                      __global float* origin, __constant int* resolution, __global float* range, __global float* FOV,
                      __global float* R, __global float* xcoor)
{

//--------------------------------------DATOS----------------------------------------------

int gid = get_global_id(0);
int id_rayos = get_global_id(1);

float3 A = {origin[0], origin[1], origin[2]};
const int n_rayos = resolution[0];
float alcance = range[0];
float alpha = FOV[0];

float x = alcance*tan(alpha/2);
//float cte = 2*x/n_rayos;


//-----------------------------------SENSOR FRAME--------------------------------------------

float sensor_T[16] = {R[0],R[1],R[2],0,
                      R[3],R[4],R[5],0,
                      R[6],R[7],R[8],0,
                        0,  0,   0,  1};

float sensor_R_[9] = {R[0],R[1],R[2],
                      R[3],R[4],R[5],
                      R[6],R[7],R[8]};


float3 B = {-x,0,alcance};
float3 C = {x,0,alcance};

float3 B_world = rotate_point(sensor_R_, B);
float3 C_world = rotate_point(sensor_R_, C);

B= B_world;
C= C_world;

float cte_x = (C.x-B.x)/n_rayos;
float cte_y = (C.y-B.y)/n_rayos;
float cte_z = (C.z-B.z)/n_rayos;

// ----------------------------------CREATE RAYS-----------------------------------------------

float3 point_aux = {B.x + cte_x*id_rayos + A.x, B.y + cte_y*id_rayos + A.y, B.z + cte_z*id_rayos + A.z};
float3 dir_ = -A+point_aux;

// ----------------------------------CALCULATE INTERSECTION-------------------------------------

float3 v0_ = {v0[3*gid], v0[3*gid+1], v0[3*gid+2]};
float3 v1_ = {v1[3*gid], v1[3*gid+1], v1[3*gid+2]};
float3 v2_ = {v2[3*gid], v2[3*gid+1], v2[3*gid+2]};

float3 edge1_ = v1_ - v0_;
float3 edge2_ = v2_ - v0_;
float3 tvec_ = A - v0_;


float3 pvec;
pvec = crossProduct(dir_, edge2_);
float det = dotProduct(edge1_, pvec);

float f = 1/det;
float u = f * dotProduct(tvec_,pvec);

float3 q = crossProduct(tvec_, edge1_);
float v = f * dotProduct(dir_,q);

if (fabs(det) <= 0.00001 )
    xcoor[id_rayos+gid*n_rayos] = -1;

else if (u < 0.0 || u > 1.0)
    xcoor[id_rayos+gid*n_rayos] = -1;

else if (v < 0.0 || u+v > 1.0)
    xcoor[id_rayos+gid*n_rayos] = -1;

else
{
    float u,v,t;
    u = dotProduct(tvec_,pvec)*f;
    float3 qvec = crossProduct(tvec_, edge1_);
    v = dotProduct(dir_, qvec)*f;
    t = dotProduct(edge2_,qvec)*f;

    if (t>=alcance)
        xcoor[id_rayos+gid*n_rayos] = -1;

    else
        xcoor[id_rayos+gid*n_rayos] = t;
}


}
