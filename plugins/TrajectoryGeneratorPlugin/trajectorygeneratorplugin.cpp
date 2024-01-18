#include "trajectorygeneratorplugin.h"
#include <iostream>
#include <QFile>
#include <QtXml/QDomDocument>
#include <cmath>
#include <QTextStream>
#include <omp.h>

void TrajectoryGeneratorPlugin::precalculate()
{

    std::cout << "PRECALCULATE" << std::endl;
}


static QVector<float> fromString(QString &str)
{
    str.replace("[", "");
    str.replace("]", "");
    str.replace(",", " ");
    str.replace(",", " ");

    QVector<float> vector;
    QStringList list = str.split(' ');
    for (int i=0; i<list.size(); i++)
    {
        QString numberGroup = list.at(i);
        vector.push_back(numberGroup.toFloat());
    }

    return vector;
}

static QVector<float> fromString2(QString &str)
{
    str.replace("[", "");
    str.replace("]", "");
    str.replace(",", "");
    str.replace(",", "");

    QVector<float> vector;
    QStringList list = str.split(' ');
    for (int i=0; i<list.size(); i++)
    {
        QString numberGroup = list.at(i);
        vector.push_back(numberGroup.toFloat());
    }

    return vector;
}

// Función para encontrar vecinos en un radio dado
QVector<int> findNeighborsInRadius(const QVector<QVector3D>& pointCloud, const QVector3D& center, float radius) {
    QVector<int> neighbors;

    #pragma omp parallel for
    for (int i = 0; i < pointCloud.size(); ++i) {
        const QVector3D& point = pointCloud[i];
        float distance = std::sqrt((point.x() - center.x()) * (point.x() - center.x()) +
                                   (point.y() - center.y()) * (point.y() - center.y()) +
                                   (point.z() - center.z()) * (point.z() - center.z()));


        if (distance <= radius) {
            #pragma omp critical
            {
                 neighbors.append(i);
            }
        }

    }

    return neighbors;
}

float findMinExcludingZero(const QVector<float>& values) {
    float minNonZero = std::numeric_limits<float>::max();  // Inicializar con un valor grande
    bool foundNonZero = false;

    for (float value : values) {
        if (value != 0) {
            foundNonZero = true;
            minNonZero = std::min(minNonZero, value);
        }
    }

    if (foundNonZero) {
        return minNonZero;
    } else {
        // Todos los valores son 0
        return 0.0;  // O algún otro valor predeterminado según tus necesidades
    }
}

static QVector<float> normVector(QVector<float> v)
{

    float max_density = *std::max_element(v.begin(), v.end());
    float min_density = findMinExcludingZero(v);

    QVector<float> v_norm;
    for (int i=0;i<v.size();i++)
        v_norm.push_back((v[i]-min_density)/(max_density-min_density));

    return v_norm;
}


void TrajectoryGeneratorPlugin::calculate()
{

    std::cout << "CALCULATE" << std::endl;


   // getTrajectoryNodes(nodes,vel,frames,FOV,resolution,w_range,w_distance,uncertainty,tree,path,false);

    QVector<QVector3D> pos_sensor;
    QVector<QVector3D> rpy_sensor;
    QVector<QVector3D> points;
    QVector<QVector3D> normals;
    QVector<QVector3D> measurements;
    int points_per_profile = 100;
    double working_distance = 270;
    int id_string = 0;

// Cargamos los datos--------------------------------------------------------------------------------
   // Trayectoria del sensor
   std::string path_global = "/home/sara/Descargas/PRUEBAS_DENSIDAD/traj_100/"; //portaRotu
   std::string path_file = path_global + "traj_sensor0" + std::to_string(id_string) +".xml";

   QFile file(path_file.c_str());
   QDomDocument xmlBOM;
   if (!file.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
   else{xmlBOM.setContent(&file);}
   QDomElement root = xmlBOM.documentElement();
   QDomElement Component=root.firstChild().toElement();
   if (Component.tagName()=="POSITION")
   {
       QDomElement Component2=Component.firstChild().toElement();
       while (Component2.tagName()=="XYZ")
       {
           QString values =Component2.firstChild().toText().data();
           QVector<float> v_f = fromString2(values);

           QVector3D posxyz_ = QVector3D(v_f[0], v_f[1], v_f[2]);
           pos_sensor.push_back(posxyz_);
           Component2 = Component2.nextSibling().toElement();

       }

   }
   Component = Component.nextSibling().toElement();
   if (Component.tagName()=="RPYdata")
   {
       QDomElement Component2=Component.firstChild().toElement();
       while (Component2.tagName()=="RPY")
       {
           QString values =Component2.firstChild().toText().data();
           QVector<float> v_f = fromString2(values);

           QVector3D rpy_ = QVector3D(v_f[0], v_f[1], v_f[2]);
          // QVector3Dd rpy_ = QVector3Dd(0,90,-3.5);

           rpy_sensor.push_back(rpy_);
           Component2 = Component2.nextSibling().toElement();
       }

   }

   // Puntos
   std::string path_1 = path_global+"step_0" + std::to_string(id_string) +"_real.txt";
   QFile file1(path_1.c_str());
   if (!file1.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
   while(!file1.atEnd())
   {
       QString line = file1.readLine();
       QVector<float> v_f = fromString(line);
       QVector3D points_ = QVector3D(v_f[0], v_f[1], v_f[2]);
       points.push_back(points_);
   }

   // Normales
  // std::string path_2 = "/home/sara/Descargas/prueba/normal_data0" + std::to_string(id_string) +".txt";
   std::string path_2 = path_global+"normal_data0" + std::to_string(id_string) +".txt";

   QFile file2(path_2.c_str());
   if (!file2.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
   while(!file2.atEnd())
   {
       QString line = file2.readLine();
       QVector<float> v_f = fromString(line);
       QVector3D normals_ = QVector3D(v_f[0], v_f[1], v_f[2]);
       normals.push_back(normals_);
   }

   // Medidas
   std::string path_3 = path_global+"step_error0" + std::to_string(id_string) +".txt";
   QFile file3(path_3.c_str());
   if (!file3.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
   while(!file3.atEnd())
   {
       QString line = file3.readLine();
       QVector<float> v_f = fromString(line);
       QVector3D measurements_ = QVector3D(v_f[0], v_f[1], v_f[2]);
       measurements.push_back(measurements_);

   }

   std::cout << "Datos cargados" << std::endl;


// Calculamos densidades sobre la medida---------------------------------------------------------------

    int frames = points.size()/points_per_profile;
    float radio_vecindad = 3.0;
    QVector<QVector<float>> densidades_total;
    QVector<float> mean_density_profiles;

    for(int p=0; p<pos_sensor.size(); p++)
    {
        QVector<QVector3D> points_p = points.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> normals_p = normals.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> measurements_p = measurements.mid(points_per_profile*p, points_per_profile);
        QVector<float> densidades(points_p.size(), 0.0);

        float mean_density = 0;
        int n=0;
        for(int i=0; i<points_p.size(); i++)
        {

           QVector3D punto_actual = points_p[i];
           if(punto_actual == QVector3D(0,0,0))
           {
               densidades[i] = 0;
               continue;
           }
           QVector<int> puntos_vecindad = findNeighborsInRadius(points, punto_actual, radio_vecindad);

           // Calcular distancias
           float distancia_total = 0.0;

           for (int j : puntos_vecindad) {

               const QVector3D& vecino = points[j];
               float distancia = std::sqrt((vecino.x() - punto_actual.x()) * (vecino.x() - punto_actual.x()) +
                                           (vecino.y() - punto_actual.y()) * (vecino.y() - punto_actual.y()) +
                                           (vecino.z() - punto_actual.z()) * (vecino.z() - punto_actual.z()));
               distancia_total += distancia;
           }


           // Calcular densidad (puedes ajustar esta función según tus necesidades)
           //float densidad_punto = 1.0 / (distancia_total / puntos_vecindad.size());
           float densidad_punto = puntos_vecindad.size()/(3.141592*radio_vecindad*radio_vecindad);

           // Almacenar la densidad en el vector
           densidades[i] = densidad_punto;
           mean_density += densidad_punto;
           n++;

        }

        if(n==0)n=1;
        mean_density_profiles.push_back(mean_density/n);
        densidades_total.push_back(densidades);

    }

    //Normalizamos las densidades
    float max_density = *std::max_element(mean_density_profiles.begin(), mean_density_profiles.end());
    float min_density = findMinExcludingZero(mean_density_profiles);

    mean_density_profiles = normVector(mean_density_profiles);

    //Calculamos desviación estándar
    double sum = std::accumulate(mean_density_profiles.begin(), mean_density_profiles.end(), 0.0);
    double m =  sum / mean_density_profiles.size();

    double accum = 0.0;
    std::for_each (mean_density_profiles.begin(), mean_density_profiles.end(), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (mean_density_profiles.size()-1));

    std::cout << "STD: " << stdev << std::endl;

    // A nivel de frame?
    for(int i=0; i<mean_density_profiles.size(); i++)
    {
       // std::cout << mean_density_profiles[i]<< std::endl;
        if(mean_density_profiles[i] == 0)
            continue;

        if((max_density-mean_density_profiles[i]) < (max_density-min_density)/2)
        {
            //Cambiar la traj aquí
           // std::cout << i << std::endl;
        }


        //std::cout << mean_density_profiles[i] << std::endl;
    }



    // Guardamos datos--------------------------------------------------------------------------
    std::cout << "Escribiendo datos..." << std::endl;
    // Crear un objeto QFile para el archivo de salida
    std::string path_densidades = path_global+"densidades2.txt";
    QFile outputFile(QString::fromStdString(path_densidades));

    // Abrir el archivo en modo de escritura (también se puede utilizar QIODevice::WriteOnly)
    if (outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        // Crear un objeto QTextStream para escribir en el archivo
        QTextStream stream(&outputFile);

        // Iterar sobre cada QVector en densidades_total
        for (const QVector<float>& densidades : densidades_total) {
            // Iterar sobre cada valor en el QVector
            for (float valor : densidades) {
                // Escribir el valor en el archivo seguido de un salto de línea
                stream << valor << "\n";
            }
        }

        // Cerrar el archivo después de escribir
        outputFile.close();
        std::cout << "Fin" << std::endl;

    }
}


void TrajectoryGeneratorPlugin::postcalculate()
{

    std::cout << "POSTCALCULATE" << std::endl;
}

void TrajectoryGeneratorPlugin::initPlugin(int argc, char **argv, QVector<QVector3D> pos_sensor, QVector<QVector3D> rpy_sensor)
{
    std::cout << "INIT" << std::endl;

}
