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

void saveTraj(QString path, QString file_name, QVector<QVector3D> pos_data, QVector<QVector3D> rpy_data, bool complete =  true)
{
    QDir dir;

    if(!dir.exists(path))
        dir.mkdir(path);

    QFile file(path + file_name);

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

std::string path_global = "/home/sara/Descargas/PRUEBAS_DENSIDAD/traj_100/new_traj/"; //portaRotu
void TrajectoryGeneratorPlugin::precalculate()
{

    qInfo() << "PRECALCULATE: Loading data...";

    int points_per_profile = 100;
    double working_distance = 270;
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
       Component22 = Component22.nextSibling().toElement();
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



    ///-----------

    // Cargamos los datos--------------------------------------------------------------------------------
       // Trayectoria del sensor
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
       std::string path_1 = path_global+"step_0" + std::to_string(id_string) +"_real.txt";
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
       std::string path_2 = path_global+"normal_data0" + std::to_string(id_string) +".txt";

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
       std::string path_3 = path_global+"step_error0" + std::to_string(id_string) +".txt";
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


}


void TrajectoryGeneratorPlugin::calculate()
{
    int points_per_profile = 100;
    double working_distance = 400;
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
    for(int p=0; p<_traj_interpolated.size(); p++)
    {
        QVector<QVector3D> points_p = _pointcloud.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> normals_p = _normal_map.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> measurements_p = _scan_image.mid(points_per_profile*p, points_per_profile);

        QVector<float> densidades(points_p.size(), 0.0);
        QVector3D mean_n(0,0,0);
        int n_n=0;

        for (QVector3D m : normals_p){
            if (m.length() > 0)
            {
                mean_n += m;
                n_n++;
            }
        }

        if(n_n==0)n_n=1;
        mean_n /= n_n;
        mean_n = mean_n.normalized();
        mean_normals.push_back(mean_n);

        double mean_m=0;
        int n_m=0;

        for (QVector3D m : measurements_p){
            if (m.z() > 0)
            {
                mean_m += m.z();
                n_m++;
            }
        }

        if(n_m==0)n_m=1;
        mean_m /= n_m;
        mean_measurements.push_back(mean_m);

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
            raw_density.at<double>(i,j) = densidades_total[i][j];
        }
    }
    _density_map_raw = raw_density;

    qInfo() << "CALCULATE: Density map calculated.";

    //Normalizamos las densidades
    float max_density = *std::max_element(mean_density_profiles.begin(), mean_density_profiles.end());
    float min_density = findMinExcludingZero(mean_density_profiles);
    double umbral = (max_density-min_density)/5; //1.5; CAMBIO
    qInfo() << "UMBRAL: " << umbral;
    qInfo() << "max: " << max_density;
    qInfo() << "min: " << min_density;


    QVector<int> low_density_ids;
    for(int i=0; i<mean_density_profiles.size(); i++)
    {
        if(mean_density_profiles[i] == 0) continue;
        //qInfo() <<  mean_normals[i];
        if(mean_density_profiles[i] < umbral/* && (mean_normals[i].y()<0.6 || mean_normals[i].z()>0.)*/)
        {
            qInfo() << _traj_interpolated[i].positionXYZ;
            qInfo() << _traj_interpolated[i].orientationRPY;
            qInfo() << "___________________";

            low_density_ids.push_back(i);
        }
    }

    //Buscamos áreas de baja densidad
    QVector<QVector<int>>areas_low_density;
    QVector<int> area_i;
    area_i.push_back(low_density_ids[0]);
    for (int i=0; i<low_density_ids.size()-1 ;i++)
    {
        if((low_density_ids[i+1]-low_density_ids[i])<2)
            area_i.push_back(low_density_ids[i+1]);
        else
        {
            if (area_i.size()>4)
                areas_low_density.push_back(area_i);
            area_i.clear();
            area_i.push_back(low_density_ids[i+1]);
        }

        if(i+1 == low_density_ids.size()-1)
        {

            if (area_i.size()>4)
                areas_low_density.push_back(area_i);
        }

     //   qInfo()<<_traj_interpolated[low_density_ids[i]].positionXYZ;
    //    qInfo() << "............";
    }


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
        double desf_wd = mean_measurements[id_ini];
        double dot_product = QVector3D::dotProduct(normal_sensor,mean_n.normalized());
        double angle=std::acos(dot_product);
        double angle_degrees = (angle*(180.0 / M_PI));
        if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
            angle_degrees *= -1;

        ///------
        double incX = desf_wd*sin(angle_degrees/180*M_PI);
        double c = 2*desf_wd*sin((angle_degrees/2)/180*M_PI);
        double incY = sqrt(c*c - incX*incX);

        QVector3D pos_i = _traj_interpolated[id_ini].positionXYZ;
        QVector3D rpy_i = _traj_interpolated[id_ini].orientationRPY;

        qInfo() << pos_i;


        rpy_i.setY(90+angle_degrees);

        pos_i.setZ(pos_i.z()+incX);
        pos_i.setY(pos_i.y()-incY);
        pos_i.setX(pos_i.x()+10);


        qInfo() << pos_i;
        qInfo() << angle_degrees;
        qInfo() << "..............";

        _pos_sensor.push_back(pos_i);
        _rpy_sensor.push_back(rpy_i);
    }

}

/*
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
          // QVector3D rpy_ = QVector3D(0,90,-3.5);

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

    QVector<QVector3D> mean_normals;
    QVector<double> mean_measurements;
    for(int p=0; p<pos_sensor.size(); p++)
    {
        QVector<QVector3D> points_p = points.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> normals_p = normals.mid(points_per_profile*p, points_per_profile);
        QVector<QVector3D> measurements_p = measurements.mid(points_per_profile*p, points_per_profile);
        QVector<float> densidades(points_p.size(), 0.0);

        //----------
        QVector3D mean_n(0,0,0); int n_n=0;
        for (QVector3D m : normals_p){
            if (m.length() == 0) continue;
            mean_n += m;
            n_n++;
           // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if(n_n==0)n_n=1;
        mean_n /= n_n;
        mean_n = mean_n.normalized();
        mean_normals.push_back(mean_n);

        double mean_m=0; int n_m=0;
        for (QVector3D m : measurements_p){
            if (m.z() == 0) continue;
            mean_m += m.z();
            n_m++;
           // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if(n_m==0)n_m=1;
        mean_m /= n_m;
        mean_measurements.push_back(mean_m);

        //-----

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
         //  float densidad_punto = puntos_vecindad.size()/(3.141592*radio_vecindad*radio_vecindad);
           float densidad_punto = puntos_vecindad.size();

           // Almacenar la densidad en el vector
           densidades[i] = densidad_punto;
           mean_density += densidad_punto;
           n++;

        }

        if(n==0)n=1;
        mean_density_profiles.push_back(mean_density/n);
        densidades_total.push_back(densidades);

    }

    //Guardar mapa de densidades-----

    cv::Mat raw_density(pos_sensor.size(), points_per_profile, CV_64FC1);
    for (int i=0; i<pos_sensor.size();i++)
    {
        for (int j=0; j<points_per_profile; j++)
        {
            raw_density.at<double>(i,j) = densidades_total[i][j];
        }
    }

    //-------------------------------


    //Normalizamos las densidades
    float max_density = *std::max_element(mean_density_profiles.begin(), mean_density_profiles.end());
    float min_density = findMinExcludingZero(mean_density_profiles);


 //   mean_density_profiles = normVector(mean_density_profiles);

//    //Calculamos desviación estándar
//    double sum = std::accumulate(mean_density_profiles.begin(), mean_density_profiles.end(), 0.0);
//    double m =  sum / mean_density_profiles.size();

//    double accum = 0.0;
//    std::for_each (mean_density_profiles.begin(), mean_density_profiles.end(), [&](const double d) {
//        accum += (d - m) * (d - m);
//    });

//    double stdev = sqrt(accum / (mean_density_profiles.size()-1));

//    std::cout << "STD: " << stdev << std::endl;

    // A nivel de frame?
  //  std::cout << "MAX, MIN: " << min_density << ", " << max_density << std::endl;
    double umbral = (max_density-min_density)/1.5;

    QVector<int> low_density_ids;
    for(int i=0; i<mean_density_profiles.size(); i++)
    {
        if(mean_density_profiles[i] == 0) continue;
        if(mean_density_profiles[i] < umbral)
            low_density_ids.push_back(i);
    }

    QVector<QVector<int>>areas_low_density;
    QVector<int> area_i;
    area_i.push_back(low_density_ids[0]);
    for (int i=0; i<low_density_ids.size()-1 ;i++)
    {
        if((low_density_ids[i+1]-low_density_ids[i])<2)
            area_i.push_back(low_density_ids[i+1]);
        else
        {
            if (area_i.size()>4)
                areas_low_density.push_back(area_i);
            area_i.clear();
            area_i.push_back(low_density_ids[i+1]);

        }
    }


//    for(int i=0; i<areas_low_density.size();i++)
//    {
//        std::cout << "Area "<< i << std::endl;
//        for (int j=0; j<areas_low_density[i].size();j++)
//        {
//            std::cout << "NORMALES: " << mean_normals[areas_low_density[i][j]].x() << ", "
//                                      << mean_normals[areas_low_density[i][j]].y() << ", "
//                                      << mean_normals[areas_low_density[i][j]].z() << std::endl;

//        }
//    }

    //Identificar por qué tiene baja densidad------------------------------------------------
    QVector<QVector3D> new_sensor_position = pos_sensor;
    QVector<QVector3D> new_sensor_orientation = rpy_sensor;

    QVector3D normal_sensor(0,1,0);
    QVector3D scan_dir(0,0,1);

    QVector<int> id_nodes;
    id_nodes.push_back(0);
    id_nodes.push_back(areas_low_density[0][0]/2);
    for(int id_zona=0; id_zona < areas_low_density.size(); id_zona++)
    {
        //Comprobar si la zona mala empieza al principio!!
        int id_medio = areas_low_density[id_zona][0+areas_low_density[id_zona].size()/2];
        id_nodes.push_back(id_medio);

        if(id_zona+1 < areas_low_density.size())
        {
            int aux = areas_low_density[id_zona][areas_low_density[id_zona].size()-1] + (areas_low_density[id_zona+1][0] - areas_low_density[id_zona][areas_low_density[id_zona].size()-1])/2;
            id_nodes.push_back(aux);
        }
        else
        {
            int aux = areas_low_density[id_zona][0] + (pos_sensor.size()-1 - areas_low_density[id_zona][areas_low_density[id_zona].size()-1]);
            id_nodes.push_back(aux);
        }
    }

    id_nodes.push_back(pos_sensor.size()-1);
  //  _pos_sensor = pos_sensor;
  //  _rpy_sensor = rpy_sensor;

    //Recorrer áreas y calcular la media de las normales en esas áreas!-------------------------
    QVector<QVector3D> mean_normals_area;
    for(int i=0; i< id_nodes.size()-1; i++)
    {
        int i_ini = id_nodes[i];
        int i_end = id_nodes[i+1];
        QVector3D mean_n(0,0,0);
        for (int j=i_ini; j<i_end;j++)
        {
            mean_n += mean_normals[j];
//            if (i==2)
//                std::cout <<mean_normals[j].x() <<", " << mean_normals[j].y() <<", " << mean_normals[j].z() << std::endl;

        }
        mean_normals_area.push_back(mean_n/(i_end-i_ini));


        if(i==id_nodes.size()-2)
            mean_normals_area.push_back(mean_n/(i_end-i_ini));

//        std::cout << mean_normals_area[i].x() <<", " << mean_normals_area[i].y() <<", " << mean_normals_area[i].z() << std::endl;


    }

    ;
    //-------------------------------------------------------------------------------------------

    for (int i=0; i<id_nodes.size(); i++)
    {
        int id_m=i;

//        //----PROVISIONAL---
//        //TA MAAAAL
//        for (int j=0; j<id_nodes.size()-1; j++)
//        {
//            if (i>id_nodes[j] & i<=id_nodes[j+1])
//            {
//                id_m=j;
//            }
//        }

        //------------------
        int id = id_nodes[i];
        QVector3D pos_i = pos_sensor[id];
        QVector3D rpy_i = rpy_sensor[id];

        double desf_wd = mean_measurements[id];
        while(desf_wd==0)desf_wd=mean_measurements[id++];
        id = id_nodes[i];

//        pos_i.setY(-desf_wd+500);

        //UTILIZAR LA MEDIA DE LAS NORMALESSS!

        double dot_product = QVector3D::dotProduct(normal_sensor,mean_normals_area[id_m].normalized());
        double angle=std::acos(dot_product);
        double angle_degrees = (angle*(180.0 / M_PI));
        if (QVector3D::dotProduct(scan_dir,mean_normals_area[id_m].normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
            angle_degrees *= -1;


//        std::cout <<"i: " << i <<" : "<<  mean_normals_area[id_m].x() <<", " <<
//                     mean_normals_area[id_m].y() <<", " <<
//                     mean_normals_area[id_m].z()<< std::endl;
        if(std::find(low_density_ids.begin(), low_density_ids.end(), id_nodes[i]) != low_density_ids.end())
        {
            rpy_i.setY(90+angle_degrees);

            ///------
            double incX = desf_wd*sin(angle_degrees/180*M_PI);
            double c = 2*desf_wd*sin((angle_degrees/2)/180*M_PI);
            double incY = sqrt(c*c - incX*incX);

            std::cout << "POSX: " << pos_i.z() << std::endl;
            std::cout << "POSY: " << pos_i.y() << std::endl;
            std::cout << "ANGLE: " << angle_degrees << std::endl;
            std::cout << "d: " << desf_wd << std::endl;

            std::cout << "INCX: " << incX << std::endl;
            std::cout << "INCY: " << incY << std::endl;
            pos_i.setZ(pos_i.z()+incX);
            pos_i.setY(pos_i.y()+incY);
        }

        ///------
        _pos_sensor.push_back(pos_i);
        _rpy_sensor.push_back(rpy_i);

    }

//    for(int id_zona=0; id_zona < areas_low_density.size(); id_zona++)
//    {
//        //Area id_zona
//        QVector3D mean_normals_area(0,0,0);
//        int n_n=0;
//        for(int i = 0; i<areas_low_density[id_zona].size(); i++)
//        {
//            mean_normals_area += mean_normals[areas_low_density[id_zona][i]];
//            n_n++;
//        }
//        mean_normals_area /= n_n;
//        QVector3D resta = normal_sensor - mean_normals_area;
//        if (resta.length()>0.3)
//        {

//            std::cout << "AJUSTAR ORIENTACIÓN NECESARIA" <<std::endl;
//            QVector3D pos_ini = pos_sensor[areas_low_density[id_zona][0]];
//            QVector3D pos_end = pos_sensor[areas_low_density[id_zona][areas_low_density[id_zona].size()-1]];
//            QVector3D pos_mid = pos_sensor[areas_low_density[id_zona][0+areas_low_density[id_zona].size()/2]];

//            QVector3D rpy_ini = rpy_sensor[areas_low_density[id_zona][0]];
//            QVector3D rpy_end = rpy_sensor[areas_low_density[id_zona][areas_low_density[id_zona].size()-1]];
//            QVector3D rpy_mid = rpy_sensor[areas_low_density[id_zona][0+areas_low_density[id_zona].size()/2]];

//            QVector3D step_pos1 = (pos_mid - pos_ini)/areas_low_density[id_zona].size()/2;
//            QVector3D step_pos2 = (pos_end - pos_mid)/areas_low_density[id_zona].size()/2;


//            double dot_product = QVector3D::dotProduct(normal_sensor,mean_normals_area.normalized());
//            double angle=std::acos(dot_product);
//            double angle_degrees = (angle*(180.0 / M_PI));

//            if (QVector3D::dotProduct(scan_dir,mean_normals_area) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
//            {
//                angle_degrees *= -1;
//                //scan_dir=QVector3D(0,0,-1);

//            }
//            for(int i = 0; i<areas_low_density[id_zona].size(); i++)
//            {

//                new_sensor_orientation[areas_low_density[id_zona][i]].setY(90+angle_degrees);
//              //  std::cout << mean_measurements[areas_low_density[id_zona][i]] << std::endl;

//                double c = 2*mean_measurements[areas_low_density[id_zona][i]]*sin(angle/2);
//                double incX = (sin(angle)*mean_measurements[areas_low_density[id_zona][i]]);
//                double incY = sqrt(c*c + incX*incX);

////                new_sensor_position[areas_low_density[id_zona][i]] = new_sensor_position[areas_low_density[id_zona][i]]
////                        + incX*scan_dir - incY*normal_sensor;

//            }


//        }


//    }
    //--------------------------------------------------------------------------------------


//    for (int i=0; i<pos_sensor.size();i++)
//    {
//        if (i==areas_low_density[0][0])
//        {
//            QVector3D pos_ini = pos_sensor[i-1];
//            QVector3D pos_end = pos_sensor[areas_low_density[0][areas_low_density[0].size()-1]];
//            QVector3D step = (pos_end-pos_ini)/(areas_low_density[0].size()*2);
//            for(int j=0; j<areas_low_density[0].size()*2; j++)
//            {
//                new_sensor_position.push_back(pos_ini);
//                pos_ini = pos_ini+step;
//                new_sensor_orientation.push_back(rpy_sensor[i]);
//            }
//        }
//        else if(i>areas_low_density[0][0] && i<areas_low_density[0][areas_low_density[0].size()-1])
//            continue;
//        else
//        {
//            new_sensor_position.push_back(pos_sensor[i]);
//            new_sensor_orientation.push_back(rpy_sensor[i]);
//        }
//    }




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

    std::string name_raw_density = path_global+"densidades.raw";
    writeMatRaw(QString::fromStdString(name_raw_density),'w',raw_density);


    std::string path_new_traj = "/step_0"+std::to_string(id_string)+"_new_traj.xml";
    saveTraj(path_global.c_str(),
             path_new_traj.c_str(),_pos_sensor, _rpy_sensor); //new_rpy_sensor_orientation


}

*/
void TrajectoryGeneratorPlugin::postcalculate()
{

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


    std::string path_new_traj = "/step_0"+std::to_string(0)+"_new_traj.xml";
    saveTraj(path_global.c_str(),
             path_new_traj.c_str(),_pos_sensor, _rpy_sensor); //new_rpy_sensor_orientation
}

void TrajectoryGeneratorPlugin::initPlugin(int argc, char **argv, QVector<QVector3D> pos_sensor, QVector<QVector3D> rpy_sensor)
{
    std::cout << "INIT" << std::endl;

}

void TrajectoryGeneratorPlugin::getTrajectory(QVector<QVector3D> &pos_sensor, QVector<QVector3D> &rpy_sensor)
{
//    pos_sensor = -pos_sensor;
//    rpy_sensor = _rpy_sensor;
}

