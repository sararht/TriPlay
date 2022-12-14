 kernel void filtrar_datos(   __global int* n_caras_, __constant int* resolution, __global float* range,
                         __global float* sensor_data_s, __global float* todo)
    {

    // Datos
    int id_rayos = get_global_id(0);
    const int n_rayos = resolution[0];
    float alcance = range[0];
    int n_caras = n_caras_[0];


    // Filtrar
    float min_value = alcance*3;
    for (int i=0; i<n_caras;i++)
    {
        if (todo[id_rayos + i*n_rayos] < min_value && (todo[id_rayos + i*n_rayos]!= -1))
        {
            min_value = todo[id_rayos + i*n_rayos];
        }

    }

    if (min_value >= alcance*3)
        sensor_data_s[id_rayos] = 0;

    else
        sensor_data_s[id_rayos] = min_value;


    }
