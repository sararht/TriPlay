
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



    kernel void get_points(__global float* origin, __constant int* resolution, __global float* range, __global float* FOV,
                           __global float* R, __global float* datos, __global float* points3D, __global float* points3D_s) 
    {

   //--------------------------------------DATOS----------------------------------------------

    int id_rayos = get_global_id(0);
    
    float3 A = {origin[0], origin[1], origin[2]};
    const int n_rayos = resolution[0];
    float alcance = range[0];
    float alpha = FOV[0];

    float x = alcance*tan(alpha/2);
    float cte = 2*x/n_rayos;

    float sensor_R_[9] = {R[0],R[1],R[2],
                          R[3],R[4],R[5],
                          R[6],R[7],R[8]};

    //-----------------------------------SENSOR FRAME--------------------------------------------

    float3 B = {-x,0,alcance};
    float3 C = {x,0,alcance};

    float3 B_world = rotate_point(sensor_R_, B); 
    float3 C_world = rotate_point(sensor_R_, C); 

    B= B_world;
    C= C_world;

    //printf("%v3hlf__",   B);
    float3 pos[2048];
    float3 dir[2048];
    //PASAR POS Y DIR COMO ARGUMENTOS :()

    float cte_x = (C.x-B.x)/n_rayos;
    float cte_y = (C.y-B.y)/n_rayos;
    float cte_z = (C.z-B.z)/n_rayos;

    // ----------------------------------CREATE RAYS-----------------------------------------------
        
    float3 point_aux = {B.x + cte_x*id_rayos + A.x, B.y + cte_y*id_rayos + A.y, B.z + cte_z*id_rayos + A.z};

    //pos[id_rayos] = point_aux;
    //dir[id_rayos] = -A + pos[id_rayos];
    float3 dir_ = -A + point_aux;


    // ----------------------------------CALCULATE POINT 3D-------------------------------------
   // printf("%f \n", datos[id_rayos]);

    if (datos[id_rayos] == 0)
    {
        points3D[id_rayos*3]=-1;
        points3D[id_rayos*3+1]=-1;
        points3D[id_rayos*3+2]=-1;

        points3D_s[id_rayos*3]=-1;
        points3D_s[id_rayos*3+1]=-1;
        points3D_s[id_rayos*3+2]=-1;
    }   
   
    else
    {
        float3 xcoor_ = A + datos[id_rayos]*dir_;

      

        float sensor_R[9] = {R[0],R[1],R[2],
                            R[3],R[4],R[5],
                            R[6],R[7],R[8]};
        float sensor_T_aux[9] = {R[0],R[3],R[6],
                                  R[1],R[4],R[7],
                                  R[2],R[5],R[8]};

        float3 A_aux = - (rotate_point(sensor_T_aux, A));
        float3 xcoor_s = rotate_point(sensor_T_aux, xcoor_) + A_aux;


        if (xcoor_s.z >= alcance)
        {
            points3D[id_rayos*3]=-1;
            points3D[id_rayos*3+1]=-1;
            points3D[id_rayos*3+2]=-1;

            points3D_s[id_rayos*3]=-1;
            points3D_s[id_rayos*3+1]=-1;
            points3D_s[id_rayos*3+2]=-1;
        }

        else
        {
            points3D[id_rayos*3]=xcoor_.x;
            points3D[id_rayos*3+1]=xcoor_.y;
            points3D[id_rayos*3+2]=xcoor_.z;
            
            points3D_s[id_rayos*3]=xcoor_s.x;
            points3D_s[id_rayos*3+1]=xcoor_s.y;
            points3D_s[id_rayos*3+2]=xcoor_s.z;
        }



    }



 
    }
