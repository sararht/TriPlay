#include "trajectorygeneratorplugin.h"
#include <iostream>
#include <QFile>
#include <QtXml/QDomDocument>
#include <cmath>
#include <QTextStream>
#include <omp.h>
#include <QDir>
#include <QXmlStreamWriter>

#include <opencv2/opencv.hpp>
#include <QtDebug>
#include <QQuaternion>
#include <eigen3/Eigen/Eigen>

void saveTraj(QString path, QString file_name, QVector<QVector3D> pos_data, QVector<QVector3D> rpy_data,
              double FPS, double vel, double FOV, int PPP, double uncertainty,
              bool complete =  true)
{
    QDir dir;
    if(!dir.exists(path))
    {
        qInfo()<<"HOLA";
        dir.mkdir(path);
    }

    QFile file(dir.filePath(path+file_name));

    if (file.open(QIODevice::WriteOnly))
    {
        QXmlStreamWriter xmlWriter(&file);
        xmlWriter.setAutoFormatting(true);

        if (complete)
        {
            xmlWriter.writeStartDocument("");
            xmlWriter.writeStartElement("TRAJECTORY");

            //PASARLE TAMBIÉN LOS DATOS PROPORCIONADOS POR EL SENSOR, SI LA Z SE VA DE RANGO DEL SENSOR MODIFICAR LA DISTANCIA¿?
            xmlWriter.writeStartElement("POSITION");
            for (int i=0; i<pos_data.size(); i++)
                xmlWriter.writeTextElement("XYZ", QString::number(pos_data[i].x())+", "+QString::number(pos_data[i].y())+", "+QString::number(pos_data[i].z()));
            xmlWriter.writeEndElement();

            xmlWriter.writeStartElement("RPYdata");
            for (int i=0; i<rpy_data.size(); i++)
                xmlWriter.writeTextElement("RPY", QString::number(rpy_data[i].x())+", "+QString::number(rpy_data[i].y())+", "+QString::number(rpy_data[i].z()));
            xmlWriter.writeEndElement();

            xmlWriter.writeTextElement("FPS", QString::number(FPS));
            xmlWriter.writeTextElement("Velocity", QString::number(vel));
            xmlWriter.writeTextElement("FOV", QString::number(FOV));
            xmlWriter.writeTextElement("Resolution", QString::number(PPP));
            xmlWriter.writeTextElement("Uncertainty", QString::number(uncertainty));

            xmlWriter.writeEndElement();

        }
        else
        {
            //xmlWriter.writeProcessingInstruction("");
            int id_n = pos_data.size() -1;
            xmlWriter.writeStartElement("Step");
            xmlWriter.writeTextElement("From", "["+QString::number(pos_data[0].x())+", "+QString::number(pos_data[0].y())+", "+QString::number(pos_data[0].z()) +"]");
            xmlWriter.writeTextElement("To", "["+QString::number(pos_data[id_n].x())+", "+QString::number(pos_data[id_n].y())+", "+QString::number(pos_data[id_n].z()) +"]");
            xmlWriter.writeTextElement("Line", "[0,0,0,0]");
            xmlWriter.writeEndElement();

        }

    }
    file.close();


}
bool writeMatRaw(const QString &filename, char mode_a_or_w, const cv::Mat &m_in)
{
    char mode[]="-b";
    char text[]="IMG_INFO";
    mode[0]=mode_a_or_w;
    FILE* fid = fopen(filename.toLatin1().constData(),mode);
    if (fid == nullptr)
    {
        return false;
    }

    fwrite(text,1,8,fid);

    int rows_coded=m_in.rows;
    int cols_coded=m_in.cols;
    fwrite(&rows_coded,sizeof(int),1,fid);
    fwrite(&cols_coded,sizeof(int),1,fid);

    char type[]="XXXX";
    switch (m_in.type())
    {
    case CV_32FC1:
        strcpy(type,"FL4B");
        break;
    case CV_32SC1:
        strcpy(type,"SI4B");
        break;
    case CV_8UC1:
        strcpy(type,"UI1B");
        break;
    case CV_16UC1:
        strcpy(type,"UI2B");
        break;
    case CV_64FC1:
        strcpy(type,"FL8B");
        break;
    case CV_64FC3:
        strcpy(type,"F38B");
        break;
    case CV_32FC3:
        strcpy(type,"F34B");
        break;
    case CV_64FC4:
        strcpy(type,"F48B");
        break;
      case CV_32FC4:
        strcpy(type,"F44B");
        break;
    default:
        fclose(fid);
        return false;
    }

    fwrite(type,1,4,fid);

    for (int i_row=0;i_row<m_in.rows;i_row++)
        fwrite(m_in.ptr(i_row),1,m_in.step[0],fid);

    fclose(fid);
    return true;

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

        if (point == QVector3D(0,0,0))
            continue;

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
QVector3D calcularMediana(QVector<QVector3D> &normales) {
    int n = normales.size();

    // Copiar el vector para no modificar el original
    QVector<QVector3D> copiaNormales = normales;

    // Ordenar el vector copia
    std::sort(copiaNormales.begin(), copiaNormales.end(), [](const QVector3D &a, const QVector3D &b) {
        return a.length() < b.length();  // Ordenar por longitud
    });

    // Calcular la mediana
    QVector3D mediana;
    if (n % 2 == 0) {
        // Si hay un número par de elementos, la mediana es el promedio de los dos elementos del medio
        mediana = 0.5 * (copiaNormales[n / 2 - 1] + copiaNormales[n / 2]);
    } else {
        // Si hay un número impar de elementos, la mediana es el elemento del medio
        mediana = copiaNormales[n / 2];
    }

    return mediana;
}



//Angle en grados
void calculateDisplacement(double angle_deg, double distance, double& displacementX, double& displacementY) {
    // Calcula los desplazamientos en X e Y
    displacementX = distance * sin(angle_deg / 180.0 * M_PI);
    displacementY = distance * (1 - cos(angle_deg / 180.0 * M_PI));
}



//std::string path_global = "/home/sara/Descargas/PRUEBAS_DENSIDAD/traj_100/"; //portaRotu
int points_per_profile = 100;
double working_distance = 150;
//bool first_it = false;

void TrajectoryGeneratorPlugin::precalculate()
{

    //Limpiar todo------------------
    _traj_interpolated.clear();
    _traj_simple.clear();
    _traj_interpolated_new.clear();
    _traj_simple_new.clear();
    _pos_sensor.clear();
    _rpy_sensor.clear();
    _pointcloud.clear();
    _normal_map.clear();
    _normal_scan_map.clear();
    _scan_image.clear();
    _density_map.clear();

     //------------------------------



    std::string path_global = _path.toStdString();
    qInfo() << "PRECALCULATE: Loading data...";


    int id_string = 0;
    QVector<QVector3D> pos_sensor, rpy_sensor;
    QVector<QVector3D> pos_sensor_simple, rpy_sensor_simple;



    ///-----------

    // Cargamos los datos--------------------------------------------------------------------------------
       // Trayectoria del sensor
       std::string path_file22 = path_global + "step_00_simple.xml";

       QFile file22(path_file22.c_str());
       QDomDocument xmlBOM22;
       if (!file22.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
       else{xmlBOM22.setContent(&file22);}
       QDomElement root22 = xmlBOM22.documentElement();
       QDomElement Component22=root22.firstChild().toElement();

       while(!Component22.isNull())
       {
           if (Component22.tagName()=="POSITION")
           {
               QDomElement Component2=Component22.firstChild().toElement();
               while (Component2.tagName()=="XYZ")
               {
                   QString values =Component2.firstChild().toText().data();
                   QVector<float> v_f = fromString2(values);

                   QVector3D posxyz_ = QVector3D(v_f[0], v_f[1], v_f[2]);
                   pos_sensor_simple.push_back(posxyz_);
                   Component2 = Component2.nextSibling().toElement();
               }
           }

           if (Component22.tagName()=="RPYdata")
           {
               QDomElement Component2=Component22.firstChild().toElement();
               while (Component2.tagName()=="RPY")
               {
                   QString values =Component2.firstChild().toText().data();
                   QVector<float> v_f = fromString2(values);

                   QVector3D rpy_ = QVector3D(v_f[0], v_f[1], v_f[2]);
                  // QVector3D rpy_ = QVector3D(0,90,-3.5);

                   rpy_sensor_simple.push_back(rpy_);
                   Component2 = Component2.nextSibling().toElement();
               }

           }


           if (Component22.tagName()=="FPS") _frames = Component22.text().toDouble();
           if (Component22.tagName()=="Velocity") _vel = Component22.text().toDouble();
           if (Component22.tagName()=="FOV") _fov = Component22.text().toDouble();
           if (Component22.tagName()=="Resolution") _resolution = Component22.text().toInt();
           if (Component22.tagName()=="Uncertainty") _uncertainty = Component22.text().toDouble();
           Component22 = Component22.nextSibling().toElement();


       }


       for(int i=0; i<pos_sensor_simple.size();i++)
       {
           pointTraj aux;
           aux.positionXYZ = pos_sensor_simple[i];
           aux.orientationRPY = rpy_sensor_simple[i];
           _traj_simple.push_back(aux);
       }


    ///-----------

    // Cargamos los datos--------------------------------------------------------------------------------
       // Trayectoria del sensor
       std::string path_file = path_global + "traj_sensor00.xml";

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
              // QVector3D rpy_ = QVector3D(0,90,-3.5);

               rpy_sensor.push_back(rpy_);
               Component2 = Component2.nextSibling().toElement();
           }

       }
       for(int i=0; i<pos_sensor.size();i++)
       {
           pointTraj aux;
           aux.positionXYZ = pos_sensor[i];
           aux.orientationRPY = rpy_sensor[i];
           _traj_interpolated.push_back(aux);
       }



       // Puntos
       std::string path_1 = path_global+"step_00_real.txt";
       QFile file1(path_1.c_str());
       if (!file1.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
       while(!file1.atEnd())
       {
           QString line = file1.readLine();
           QVector<float> v_f = fromString(line);
           QVector3D points_ = QVector3D(v_f[0], v_f[1], v_f[2]);
           _pointcloud.push_back(points_);
       }


       // Normales
      // std::string path_2 = "/home/sara/Descargas/prueba/normal_data0" + std::to_string(id_string) +".txt";
       std::string path_2 = path_global+"normal_data00.txt";

       QFile file2(path_2.c_str());
       if (!file2.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
       while(!file2.atEnd())
       {
           QString line = file2.readLine();
           QVector<float> v_f = fromString(line);
           QVector3D normals_ = QVector3D(v_f[0], v_f[1], v_f[2]);
           _normal_map.push_back(normals_);
       }


       // Medidas
       std::string path_3 = path_global+"step_error00.txt";
       QFile file3(path_3.c_str());
       if (!file3.open(QIODevice::ReadOnly )){qWarning("Error while loading file"); return;}
       while(!file3.atEnd())
       {
           QString line = file3.readLine();
           QVector<float> v_f = fromString(line);
           QVector3D measurements_ = QVector3D(v_f[0], v_f[1], v_f[2]);
           _scan_image.push_back(measurements_);

       }

       qInfo() << "PRECALCULATE: Data loaded.";


       //Mapa de normales de escaneo
       _normal_scan_image = cv::Mat(_traj_interpolated.size(), points_per_profile, CV_64FC1);
       cv::Mat _normal_image(_traj_interpolated.size(), points_per_profile, CV_64FC1);;
       cv::Mat _normal_sensor_image(_traj_interpolated.size(), points_per_profile, CV_64FC1);;

       for(int i=0; i<_traj_interpolated.size(); i++)
       {
           QVector<QVector3D> normals_p = _normal_map.mid(points_per_profile*i, points_per_profile);
           QVector3D rpy_aux = _traj_interpolated[i].orientationRPY;
           if(rpy_aux.z() !=0 )
           {
               rpy_aux.setY(180-rpy_aux.y());
               rpy_aux.setZ(0);
           }
           if(rpy_aux.x() !=0 )
           {
               rpy_aux.setX(0);
           }
           double angle_aux = 90-rpy_aux.y();
           if(angle_aux<0)angle_aux*=-1;
          QVector3D normal_sensor = QVector3D(0,cos(angle_aux/180*M_PI), sin(angle_aux/180*M_PI));

           for(int j=0; j<points_per_profile; j++)
           {
               //.y porque me interesa más el pitch en este caso ,pero dependería
               _normal_image.at<double>(i,j) = normals_p[j].y();
               _normal_sensor_image.at<double>(i,j) = normal_sensor.y();
           }
       }
       _normal_scan_image = _normal_sensor_image - _normal_image;

       std::string name_raw = path_global+"normal_scan.raw";
       writeMatRaw(QString::fromStdString(name_raw),'w',_normal_scan_image);




}

void TrajectoryGeneratorPlugin::calculate()
{
    bool first_it = _isFirstIteration;

    if(first_it) qInfo() << "Primera iteración";
    else qInfo() << "NO primera iteración";


    int id_string = 0;
    QVector3D normal_sensor(0,1,0);
    QVector3D scan_dir(0,0,1);

    qInfo() << "CALCULATE: Calculating density map...";
    // Calculamos densidades sobre la medida---------------------------------------------------------------

    float radio_vecindad = 3.0;
    QVector<QVector<float>> densidades_total;
    QVector<float> mean_density_profiles;

    QVector<QVector3D> mean_normals;
    QVector<double> mean_measurements;
    QVector<QVector3D> mean_pointcloud;

    QVector<double> mean_scan_normals;


    for(int p=0; p<_traj_interpolated.size(); p++)
    {


        //--- DIFERENCIA DE NORMALES---
        double mean_aux=0;
        int n_ma=0;
        for(int i=0; i<_normal_scan_image.cols; i++)
        {
            if(_normal_scan_image.at<double>(p,i) > 0.8)
                continue;
            mean_aux += _normal_scan_image.at<double>(p,i);
            n_ma ++;
        }
        if(n_ma==0)n_ma=1;
        mean_scan_normals.push_back(mean_aux/n_ma);

        //-----------------------------

        QVector<QVector3D> points_p = _pointcloud.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> normals_p = _normal_map.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> measurements_p = _scan_image.mid(points_per_profile*p, points_per_profile);

        QVector<float> densidades(points_p.size(), 0.0);
        QVector3D mean_n(0,0,0);
        int n_n=0;
        double mean_m=0;
        int n_m=0;
        QVector3D mean_p(0,0,0);

        for (int i = 0; i < points_p.size(); i++) {
            QVector3D normal = normals_p[i];
            QVector3D measurement = measurements_p[i];
            QVector3D point = points_p[i];

            if (normal.length() > 0) {
                mean_n += normal;
                mean_p += point;
                n_n++;
            }

            if (measurement.z() > 0) {
                mean_m += measurement.z();
                n_m++;
            }
        }

        if (n_n == 0) n_n = 1;
        if (n_m == 0) n_m = 1;

        mean_n /= n_n;
        mean_n = mean_n.normalized();
        mean_normals.push_back(mean_n);

        mean_m /= n_m;
        mean_measurements.push_back(mean_m);

        mean_p /= n_n;
        mean_pointcloud.push_back(mean_p);

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

           QVector<int> puntos_vecindad = findNeighborsInRadius(_pointcloud, punto_actual, radio_vecindad);

           // Calcular distancias
           float distancia_total = 0.0;

           for (int j : puntos_vecindad) {

               const QVector3D& vecino = _pointcloud[j];
               float distancia = (vecino - punto_actual).length(); /*std::sqrt((vecino.x() - punto_actual.x()) * (vecino.x() - punto_actual.x()) +
                                           (vecino.y() - punto_actual.y()) * (vecino.y() - punto_actual.y()) +
                                           (vecino.z() - punto_actual.z()) * (vecino.z() - punto_actual.z()));*/
               distancia_total += distancia;
           }
           // Calcular densidad (puedes ajustar esta función según tus necesidades)
           //float densidad_punto = 1.0 / (distancia_total / puntos_vecindad.size());
         //  float densidad_punto = puntos_vecindad.size()/(3.141592*radio_vecindad*radio_vecindad);
           float densidad_punto = puntos_vecindad.size();
           densidades[i] = densidad_punto;
           mean_density += densidad_punto;
           n++;
        }
        if(n==0)n=1;
        mean_density_profiles.push_back(mean_density/n);
        densidades_total.push_back(densidades);
        _density_map.push_back(densidades);

    }
    //Guardar mapa de densidades-----
    cv::Mat raw_density(_traj_interpolated.size(), points_per_profile, CV_64FC1);
    for (int i=0; i<_traj_interpolated.size();i++)
    {
        for (int j=0; j<points_per_profile; j++)
        {
            raw_density.at<double>(i,j) = _density_map[i][j];
        }
    }
    _density_map_raw = raw_density;

    qInfo() << "CALCULATE: Density map calculated.";


    QVector<QVector<int>>areas_low_density;
    if (first_it)
    {
        //Normalizamos las densidades
        float max_density = *std::max_element(mean_density_profiles.begin(), mean_density_profiles.end());
        float min_density = findMinExcludingZero(mean_density_profiles);
        double umbral = (max_density-min_density)/1.5;
//        qInfo() << "UMBRAL: " << umbral;
//        qInfo() << "max: " << max_density;
//        qInfo() << "min: " << min_density;


        QVector<int> low_density_ids;
        for(int i=0; i<mean_density_profiles.size(); i++)
        {
            if(mean_density_profiles[i] == 0) continue;
            QVector3D resta = normal_sensor - mean_normals[i];
//            qInfo() << "RESTA: " << resta;
//            qInfo() << "lenght: " <<resta.length();
//            qInfo() << "----------";
            if(/*mean_density_profiles[i] < umbral &&*/ (resta.length()>0.5))
            {
                low_density_ids.push_back(i);
            }
        }

        //Buscamos áreas de baja densidad
        QVector<int> area_i;
        area_i.push_back(low_density_ids[0]);
        for (int i=0; i<low_density_ids.size()-1 ;i++)
        {
            if((low_density_ids[i+1]-low_density_ids[i])<2)
            {
             //  qInfo() << _traj_interpolated[low_density_ids[i+1]].positionXYZ;
                area_i.push_back(low_density_ids[i+1]);
            }
            else
            {
                if (area_i.size()>4)
                    areas_low_density.push_back(area_i);
                area_i.clear();
                area_i.push_back(low_density_ids[i+1]);
            }
        }
        if(area_i.size()>4)areas_low_density.push_back(area_i);
    }


    if (!first_it)
    {
        //Retocar los puntos y añadir puntos intermedios

        qInfo()<<"Traj simple: " <<_traj_simple.size();
        for (int i=0; i<_traj_simple.size()-1; i++)
        {
            QVector3D pos_ini = _traj_simple[i].positionXYZ;
            QVector3D pos_end = _traj_simple[i+1].positionXYZ;
            QVector3D diff = pos_end -pos_ini;

            _pos_sensor.push_back( _traj_simple[i].positionXYZ);
            _rpy_sensor.push_back( _traj_simple[i].orientationRPY);


            //AQUÍ METO PUNTOS SI HAY MUCHA DISTANCIA
            //TENGO QUE MIRAR A VER LAS ZONAS SEGUN EL MAPA DE NORMALES
            //ASEGURAR QUE SIEMPRE TIENE MOVIMIENTO DE AVANCE?
            if(diff.length()>100)
            {

                // Si están muy separados entre sí los puntos meto un punto intermedio
                int id_ini =0;
                int id_end =0;
                for(int j=0; j<_traj_interpolated.size(); j++)
                {
                    if(_traj_interpolated[j].positionXYZ == pos_ini)
                        id_ini = j;
                    else if(_traj_interpolated[j].positionXYZ == pos_end)
                    {
                        id_end = j;
                        break;
                    }
                }
                if (id_end==0)id_end =_traj_interpolated.size()-1;
                int id_mid = id_ini + (id_end-id_ini)/2;

                //Actualizar posición sensor en esos puntos intermedios---
                QVector3D mean_n(0,0,0);
                double mean_m = 0;

                for (int j=id_ini; j<id_end;j++)
                {
                    mean_n += mean_normals[j];
                    mean_m += mean_measurements[j];
                }
                mean_n = mean_n/(id_end-id_ini);
                mean_m = mean_m/(id_end-id_ini);


                QVector3D rpy_aux = _traj_interpolated[id_mid].orientationRPY;
                if(rpy_aux.z() !=0 )
                {
                    rpy_aux.setY(180-rpy_aux.y());
                    rpy_aux.setZ(0);
                }
                if(rpy_aux.x() !=0 )
                {
                    rpy_aux.setX(0);
                }


                double angle_aux = 90-rpy_aux.y();
                if(angle_aux<0)angle_aux*=-1;
                normal_sensor = QVector3D(0,cos(angle_aux/180*M_PI), sin(angle_aux/180*M_PI));

                if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
                {
                    normal_sensor.setZ(-normal_sensor.z());
                }

                double desf_wd = working_distance - mean_m;

                double dot_product = QVector3D::dotProduct(normal_sensor,mean_n.normalized());
                double angle=std::acos(dot_product);
                double angle_degrees = (angle*(180.0 / M_PI));

                if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
                    angle_degrees *= -1;

                if(angle_degrees>10) angle_degrees = 10;
                if(angle_degrees<10) angle_degrees = -10;

                ///------
                double incX = working_distance*sin(angle_degrees/180*M_PI);
                double c = 2*working_distance*sin((angle_degrees/2)/180*M_PI);
                double incY = sqrt(c*c - incX*incX);

                double alpha = (rpy_aux.y()+angle_degrees) * M_PI / 180.0;
                double new_incX= incX*cos(alpha) + incY*sin(alpha);
                double new_incY= -incX*sin(alpha) + incY*cos(alpha);


                QVector3D pos_i = _traj_interpolated[id_mid].positionXYZ;
                QVector3D rpy_i = rpy_aux;//_traj_interpolated[id_mid].orientationRPY;


                rpy_i.setY(rpy_aux.y()+angle_degrees);

                pos_i.setZ(pos_i.z()+incX+desf_wd*normal_sensor.z());
                pos_i.setY(pos_i.y()+desf_wd*normal_sensor.y()-incY);


                if(QVector3D::dotProduct((pos_i-pos_ini), scan_dir) < 0)
                {
//                    qInfo() << "MIRA____________________________";
//                    qInfo() << pos_ini;
//                    qInfo() << pos_i;
//                    qInfo() << pos_end;

                }

                _pos_sensor.push_back(pos_i);
                _rpy_sensor.push_back(rpy_i);
            }

        }

        _pos_sensor.push_back(_traj_simple[_traj_simple.size()-1].positionXYZ);
        _rpy_sensor.push_back(_traj_simple[_traj_simple.size()-1].orientationRPY);


        //SEGÚN MAPA DIFERENCIA DE NORMALES-------------------------------------
        QVector<int> bad_areas;
        for(int i=0; i<mean_scan_normals.size(); i++)
        {
             if (fabs(mean_scan_normals[i])>0.45)
             {
                 bad_areas.push_back(i);

             }
        }
        QVector<QVector<int>>areas_high_diff_normals;
        QVector<int> bad_area_i;

        if(bad_area_i.size()>0)
        {
            bad_area_i.push_back(bad_areas[0]);
            for(int i=0; i<bad_areas.size()-1;i++)
            {
                if(bad_areas[i+1] - bad_areas[i] < 2)
                {
                    bad_area_i.push_back(bad_areas[i+1]);
                }
                else
                {
                    if (bad_area_i.size()>4)
                        areas_high_diff_normals.push_back(bad_area_i);
                    bad_area_i.clear();
                    bad_area_i.push_back(bad_areas[i+1]);
                }
            }
            if(bad_area_i.size()>4)areas_high_diff_normals.push_back(bad_area_i);
        }
        else
            qInfo()<<"Not bad areas founded in difference normal map";

        for(int i=0; i<areas_high_diff_normals.size();i++)
        {
            if(areas_high_diff_normals[i].size()==0)continue;

            int id_ini = areas_high_diff_normals[i][0];
            int id_end = areas_high_diff_normals[i][areas_high_diff_normals[i].size()-1];

            QVector3D mean_n(0,0,0);
            for (int j=id_ini; j<id_end;j++)
            {
                mean_n += mean_normals[j];
            }
            mean_n = mean_n/(id_end-id_ini);

            QVector3D pos_area_ini = _traj_interpolated[id_ini].positionXYZ;
            QVector3D pos_area_end = _traj_interpolated[id_end].positionXYZ;

            for(int j=0; j<_pos_sensor.size()-1; j++)
            {
                QVector3D pos_ini = _pos_sensor[j];
                QVector3D pos_end = _pos_sensor[j+1];
                QVector3D pos_mid = pos_ini + (pos_end-pos_ini)/2;

                QVector3D rpy_ini = _rpy_sensor[j];
                QVector3D rpy_end = _rpy_sensor[j+1];
                QVector3D rpy_mid = rpy_ini + (rpy_end-rpy_ini)/2;

                double diff_ini = (pos_ini-pos_area_ini).length();
                double diff_end = (pos_end-pos_area_end).length();

                if(diff_ini<50 && diff_end<50)
                {

                    //Actualizar esas poses!
                    QVector3D rpy_aux = _traj_interpolated[id_ini+(id_end-id_ini)/2].orientationRPY;
                    normal_sensor = QVector3D(0,1,0);
                    double dot_product = QVector3D::dotProduct(normal_sensor,mean_n.normalized());
                    double angle=std::acos(dot_product);
                    double angle_degrees = (angle*(180.0 / M_PI));
                    if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
                        angle_degrees *= -1;


                    double dif_angle =-(rpy_aux.y() - (90+angle_degrees));
                    if(dif_angle>25)dif_angle=25;
                    else if(dif_angle<-25)dif_angle=-25;

                    double desf_wd = working_distance; //mean_measurements[id_ini+(id_end-id_ini)/2];
                    double incX, incY;
                    calculateDisplacement(dif_angle,desf_wd,incX,incY);
                    double alpha = (rpy_aux.y()+dif_angle) * M_PI / 180.0;
                    double new_incX= incX*cos(alpha) + incY*sin(alpha);
                    double new_incY= -incX*sin(alpha) + incY*cos(alpha);

                    QVector3D new_pos, new_rpy;
                    new_pos = pos_mid + QVector3D(0,-new_incY, new_incX);
                    new_rpy = rpy_aux + QVector3D(0,dif_angle,0);

                    QVector3D diff = new_pos - _pos_sensor[j];
                    if(QVector3D::dotProduct(diff, scan_dir) < 0)
                    {
                        _pos_sensor[j]=new_pos;
                        _rpy_sensor[j]=new_rpy;
                    }

                }

            }

            //Buscamos los puntos de la trayectoria simple más cercanos a esto y los movemos.
            //Si no los hay lo suficientemente cercanos, los añadimos

        }

        //----------------------------------------------------------------------

        for(int i=0; i<_pos_sensor.size(); i++)
        {
//            if(_rpy_sensor[i].z() !=0 )
//            {
//                _rpy_sensor[i].setY(180-_rpy_sensor[i].y());
//                _rpy_sensor[i].setZ(0);
//            }
//            if(_rpy_sensor[i].x() !=0 )
//            {
//                _rpy_sensor[i].setX(0);
//            }

        }

        normal_sensor = QVector3D(0,1,0);
        for(int i=1; i<_pos_sensor.size()-1; i++)
        {

            QVector3D pos_ant = _pos_sensor[i-1];
            QVector3D pos_i = _pos_sensor[i];
            QVector3D pos_post = _pos_sensor[i+1];
            QVector3D rpy_ant = _rpy_sensor[i-1];
            QVector3D rpy_i = _rpy_sensor[i];
            QVector3D rpy_post = _rpy_sensor[i+1];


            if(QVector3D::dotProduct((pos_i-pos_ant), scan_dir) < 0)
            {
//                if (i==1)
//                {
//                qInfo() << "MIRA____________________________";
//                qInfo() << pos_ant;
//                qInfo() << pos_post;
                QVector3D desf_pos = pos_post - pos_ant;
                QVector3D desf_rpy = rpy_post - rpy_ant;
//                _pos_sensor[i] = pos_ant+desf_pos/2;
//                _rpy_sensor[i] = rpy_ant+desf_rpy/2;
//                }
//                else
//                {
//                    QVector3D desf_pos = pos_post - pos_ant;
//                    QVector3D desf_rpy = rpy_post - rpy_ant;
//                    _pos_sensor[i] = pos_ant+desf_pos/4;
//                    _rpy_sensor[i] = rpy_ant+desf_rpy/4;

//                    _pos_sensor[i-1] =_pos_sensor[i-1] - (pos_ant-_pos_sensor[i-2])/4;
//                    _rpy_sensor[i-1] = _rpy_sensor[i-1] - (rpy_ant-_rpy_sensor[i-2])/4;
//                }


            }

        }


    }

    else
    {

    //------------------------------------------------------------------------------------------------------------------
        //Identificar por qué tiene baja densidad --> UTILIZAR ESTO PARA PONER ALGÚN TIPO DE CÓDIGO O ALGO? ---------------
        QVector<QVector3D> mean_normals_area;
        for (int id_zona=0; id_zona<areas_low_density.size(); id_zona++)
        {

            //Area id_zona
            QVector3D mean_normals_area_(0,0,0);
            int n_n=0;
            for(int i = 0; i<areas_low_density[id_zona].size(); i++)
            {
                mean_normals_area_ += mean_normals[areas_low_density[id_zona][i]];
                n_n++;
            }
            mean_normals_area_ /= n_n;
            mean_normals_area.push_back(mean_normals_area_);
            QVector3D resta = normal_sensor - mean_normals_area_;

            if (resta.length()>0.3)
            {
                qInfo() << "AJUSTAR ORIENTACIÓN NECESARIA";
            }
            else
            {
                qInfo() << "NO ES PROBLEMA DE LA ORIENTACIÓN";
            }
        }
        qInfo()<< " N areas: " << areas_low_density.size();
    //------------------------------------------------------------------------------------------------------------------
        QVector<int> id_nodes;
        id_nodes.push_back(0);
        for(int id_zona=0; id_zona < areas_low_density.size(); id_zona++)
        {
            id_nodes.push_back(areas_low_density[id_zona][0]);
            id_nodes.push_back(areas_low_density[id_zona][areas_low_density[id_zona].size()-1]);
        }
        id_nodes.push_back(_traj_interpolated.size()-1);


        for(int i=0; i<id_nodes.size(); i++)
        {
            //Ahora para cada una de estas zonas, coger el punto medio y utilizarlo como nodo
            int id_ini = id_nodes[i];
            int id_end;
            if (i<id_nodes.size()-1)
                id_end = id_nodes[i+1];
            else
                id_end = id_ini;

            int ini = id_ini-10; if(ini<0)ini=0;
            int end = id_ini+10; if(end>id_end)end=id_end;
            QVector3D mean_n(0,0,0);
            for (int j=ini; j<end;j++)
            {
                mean_n += mean_normals[j];
            }
            mean_n = mean_n/(end-ini);
            double desf_wd = working_distance - mean_measurements[id_ini];
            double dot_product = QVector3D::dotProduct(normal_sensor,mean_n.normalized());
            double angle=std::acos(dot_product);
            double angle_degrees = (angle*(180.0 / M_PI));
            if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
                angle_degrees *= -1;

            ///------
            double incX = working_distance*sin(angle_degrees/180*M_PI);
            double c = 2*working_distance*sin((angle_degrees/2)/180*M_PI);
            double incY = sqrt(c*c - incX*incX);

            QVector3D pos_i = _traj_interpolated[id_ini].positionXYZ;
            QVector3D rpy_i = _traj_interpolated[id_ini].orientationRPY;

            //------

            rpy_i.setY(90+angle_degrees);

            pos_i.setZ(pos_i.z()+incX);
            pos_i.setY(pos_i.y()+desf_wd-incY);
            pos_i.setX(pos_i.x()+10);

            _pos_sensor.push_back(pos_i);
            _rpy_sensor.push_back(rpy_i);
        }
    }
}

void TrajectoryGeneratorPlugin::postcalculate()
{

    std::string path_global = _path.toStdString();

   qInfo() << "POSTCALCULATE: Saving data";
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
        for (const QVector<float>& densidades : _density_map) {
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

    std::string name_raw_density = path_global+"densidades.raw";
    writeMatRaw(QString::fromStdString(name_raw_density),'w',_density_map_raw);


    std::string path_new_traj = "step_00_new_traj.xml";
    saveTraj(path_global.c_str(),
             path_new_traj.c_str(),_pos_sensor, _rpy_sensor, _frames,_vel,_fov,_resolution,_uncertainty); //new_rpy_sensor_orientation


    std::string path_new_simple = "step_00_simple.xml";
    saveTraj((path_global+"new_traj/").c_str(),
             path_new_simple.c_str(),_pos_sensor, _rpy_sensor, _frames,_vel,_fov,_resolution,_uncertainty); //new_rpy_sensor_orientation



}

void TrajectoryGeneratorPlugin::initPlugin(int argc, char **argv, QVector<QVector3D> pos_sensor, QVector<QVector3D> rpy_sensor)
{
    std::cout << "INIT" << std::endl;
    _isFirstIteration = true;

}

void TrajectoryGeneratorPlugin::getTrajectory(QVector<QVector3D> &pos_sensor, QVector<QVector3D> &rpy_sensor, double &fov, double &vel, double &frames, int &resolution, double &uncertainty)
{
    pos_sensor = _pos_sensor;
    rpy_sensor = _rpy_sensor;
    fov = _fov;
    frames = _frames;
    resolution = _resolution;
    uncertainty = _uncertainty;
    vel = _vel;
}

void TrajectoryGeneratorPlugin::setCustomFlag(bool isFirstIteration)
{
    _isFirstIteration = isFirstIteration;
}

void TrajectoryGeneratorPlugin::setPath(QString path)
{
    _path = path;
}
