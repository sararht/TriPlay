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

#include <QElapsedTimer>

// Función para convertir ángulos de grados a radianes
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}


// Función para calcular la matriz de rotación a partir de los ángulos RPY
Eigen::Matrix3d rpy_to_rotation_matrix(double roll, double pitch, double yaw) {
    // Convertir ángulos a radianes
    roll = deg2rad(roll);
    pitch = deg2rad(pitch);
    yaw = deg2rad(yaw);

    // Calcular las matrices de rotación individuales
    Eigen::Matrix3d R_roll;
    R_roll << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);

    Eigen::Matrix3d R_pitch;
    R_pitch << cos(pitch), 0, sin(pitch),
               0, 1, 0,
               -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0,
             sin(yaw), cos(yaw), 0,
             0, 0, 1;

    // Calcular la matriz de rotación total
    Eigen::Matrix3d R_total = R_yaw * R_pitch * R_roll;

    return R_total;
}

// Función para calcular un vector unitario en la dirección de los ángulos RPY
Eigen::Vector3d unit_vector_from_rpy(double roll, double pitch, double yaw) {
    // Obtener la matriz de rotación
    Eigen::Matrix3d R = rpy_to_rotation_matrix(roll, pitch, yaw);

    // Extraer el primer vector de la matriz de rotación (primera columna)
    Eigen::Vector3d unit_vector = R.col(0);

    return unit_vector;
}

// Función para calcular un vector perpendicular a la orientación RPY
Eigen::Vector3d perpendicular_vector_from_rpy(double roll, double pitch, double yaw) {
    // Obtener la matriz de rotación
    Eigen::Matrix3d R = rpy_to_rotation_matrix(roll, pitch, yaw);

    // Extraer el tercer vector de la matriz de rotación (tercera columna)
    Eigen::Vector3d perpendicular_vector = R.col(2);

    return perpendicular_vector;
}

void saveTraj(QString path, QString file_name, QVector<QVector3D> pos_data, QVector<QVector3D> rpy_data,
              double FPS, double vel, double FOV, int PPP, double uncertainty,
              bool complete =  true)
{
    QDir dir;
    if(!dir.exists(path))
    {
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

    qInfo() << "TIPO RAW QUE ESCRIBO: " << m_in.type();
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

cv::Mat readMatRaw(const QString &filename)
{

    qInfo() << filename;
    FILE* fid = fopen(filename.toLatin1().constData(), "rb");
    if (fid == nullptr)
    {
        qWarning() << ".raw not found";
        return cv::Mat();
    }

    char text[8];
    fread(text, 1, 8, fid);

    int rows, cols;
    fread(&rows, sizeof(int), 1, fid);
    fread(&cols, sizeof(int), 1, fid);

    char type[4];
    fread(type, 1, 4, fid);

    cv::Mat mat;
    int type_code;
    if (strcmp(type, "FL4B") == 0)
    {
        type_code = CV_32FC1;
    }
    else if (strcmp(type, "SI4B") == 0)
    {
        type_code = CV_32SC1;
    }
    else if (strcmp(type, "UI1B") == 0)
    {
        type_code = CV_8UC1;
    }
    else if (strcmp(type, "UI2B") == 0)
    {
        type_code = CV_16UC1;
    }
    else if (strcmp(type, "FL8B") == 0)
    {
        type_code = CV_64FC1;
    }
    else if (strcmp(type, "F38B") == 0)
    {
        type_code = CV_64FC3;
    }
    else if (strcmp(type, "F34B") == 0)
    {
        type_code = CV_32FC3;
    }
    else if (strcmp(type, "F48B") == 0)
    {
        type_code = CV_64FC4;
    }
    else if (strcmp(type, "F44B") == 0)
    {
        type_code = CV_32FC4;
    }
    else
    {
        fclose(fid);
        return cv::Mat();
    }

    mat = cv::Mat(rows, cols, type_code);

    for (int i_row = 0; i_row < rows; i_row++)
        fread(mat.ptr(i_row), 1, mat.step[0], fid);

    fclose(fid);

    qInfo() << mat.cols << " - " << mat.rows;

    return mat;
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

//        float distance = std::sqrt((point.x() - center.x()) * (point.x() - center.x()) +
//                                   (point.y() - center.y()) * (point.y() - center.y()) +
//                                   (point.z() - center.z()) * (point.z() - center.z()));
         float distance = (point - center).lengthSquared();

        if (distance <= radius*radius) {
            #pragma omp critical
            {
                 neighbors.append(i);
            }
        }

    }

    return neighbors;
}

void findMinMaxExcludingZero(const QVector<QVector<float>>& values, float& min_result, float& max_result) {
    float minNonZero = std::numeric_limits<float>::max();  // Inicializar con un valor grande
    float maxNonZero = std::numeric_limits<float>::lowest();  // Inicializar con un valor pequeño
    bool foundNonZero = false;

    for (const auto& innerVector : values) {
        for (float value : innerVector) {
            if (value != 0) {
                foundNonZero = true;
                minNonZero = std::min(minNonZero, value);
                maxNonZero = std::max(maxNonZero, value);
            }
        }
    }

    if (foundNonZero) {
        min_result = minNonZero;
        max_result = maxNonZero;
    } else {
        // Todos los valores son 0
        min_result = 0.0;  // O algún otro valor predeterminado según tus necesidades
        max_result = 0.0;  // O algún otro valor predeterminado según tus necesidades
    }
}

//Angle en grados
void calculateDisplacement(double angle_deg, double distance, double& displacementX, double& displacementY) {
    // Calcula los desplazamientos en X e Y
    displacementX = distance * sin(angle_deg / 180.0 * M_PI);
    displacementY = distance * (1 - cos(angle_deg / 180.0 * M_PI));
}

QVector3D fixRPYOrientation(QVector3D rpy)
{
    QVector3D rpy_new = rpy;
    if(rpy_new.z() !=0 )
    {
        rpy_new.setY(180-rpy.y());
        rpy_new.setZ(0);
    }
    if(rpy_new.x() !=0 )
    {
        rpy_new.setX(0);
    }
    return rpy_new;
}

void updatePosFromDifAngle(QVector3D pos_ini, QVector3D rpy_ini, QVector3D rpy_aux, QVector3D scan_dir, QVector3D normal_sensor, QVector3D surface_normal, double max_angle, double working_distance, QVector3D &pos_updated, QVector3D &rpy_updated, double desf_wd=0)
{

//    qInfo()<<"FUNC: pos_ini: " << pos_ini << " - " << rpy_ini;
//    qInfo()<<"FUNC: rpy_aux: " << rpy_aux;

    double dot_product = QVector3D::dotProduct(normal_sensor,surface_normal.normalized());
    double angle=std::acos(dot_product);
    double angle_degrees = (angle*(180.0 / M_PI));
    if (QVector3D::dotProduct(scan_dir,surface_normal.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
        angle_degrees *= -1;

    double dif_angle =-(rpy_aux.y() - (90+angle_degrees));
//    qInfo()<<"FUNC: ANGLE: " << dif_angle;

    if(dif_angle>max_angle)dif_angle=max_angle;
    else if(dif_angle<-max_angle)dif_angle=-max_angle;
//    qInfo()<<"FUNC: ANGLE limited: " << dif_angle;



    double incrementoX = working_distance*sin(dif_angle*M_PI/180); //Esto en la direccion perpendicular a la nueva orientación
    double incrementoY =  working_distance*(1-std::cos(dif_angle*M_PI/180)); //Esto en la dirección de la nueva orientación

    rpy_updated = rpy_ini+ QVector3D(0,dif_angle,0);
    rpy_updated = fixRPYOrientation(rpy_updated);

//    qInfo() << "FUNC: normal" << surface_normal;

    Eigen::Vector3d direction_vector = unit_vector_from_rpy(rpy_updated.x(), rpy_updated.y(), rpy_updated.z());
    QVector3D paralel_vector =QVector3D(fabs(direction_vector[1]), fabs(direction_vector[2]), fabs(direction_vector[0]));

    if(QVector3D::dotProduct(rpy_updated, normal_sensor) < 90) paralel_vector.setZ(-paralel_vector.z());
    QVector3D perp_vector(0, fabs(paralel_vector.z()), fabs(paralel_vector.y()));

    if(QVector3D::dotProduct(rpy_updated, normal_sensor) > 90)
    {
        if(dif_angle>0)
            perp_vector.setY(-perp_vector.y());
        else
            perp_vector.setZ(-perp_vector.z());
    }
    else if(QVector3D::dotProduct(rpy_updated, normal_sensor) < 90)
    {
        if(dif_angle<0)
        {
            perp_vector.setY(-perp_vector.y());
            perp_vector.setZ(-perp_vector.z());
        }

    }


  //  qInfo() << "FUNC: iNCREMENTOS: " << incrementoX << " - " << incrementoY;
  //  qInfo() << "FUNC:Perpendicular vector: " << perp_vector ;
  //  qInfo() << "FUNC:Direction vector: " << paralel_vector ;

    pos_updated = pos_ini + fabs(incrementoX)*perp_vector
                      + fabs(incrementoY)*(paralel_vector)
                      + desf_wd*paralel_vector;

 //   qInfo() << "FUNC: POS_UPDATED" << pos_updated << " - " << rpy_updated;

}




//std::string path_global = "/home/sara/Descargas/PRUEBAS_DENSIDAD/traj_100/"; //portaRotu
int points_per_profile = 100;
double working_distance = 100;
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
    qInfo() << "TRAJECTORY GENERATOR PLUGIN. PRECALCULATE: Loading data...";

    int id_string = 0;
    QVector<QVector3D> pos_sensor, rpy_sensor;
    QVector<QVector3D> pos_sensor_simple, rpy_sensor_simple;

    ///-----------

    // Cargamos los datos--------------------------------------------------------------------------------
       // Trayectoria del sensor
       std::string path_file22 = path_global + "step_00_simple.xml";

       QFile file22(path_file22.c_str());
       QDomDocument xmlBOM22;
       if (!file22.open(QIODevice::ReadOnly )){qWarning("TRAJECTORY GENERATOR PLUGIN: Error while loading file"); return;}
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
               if (normals_p[j]==QVector3D(0,0,0))
                   _normal_sensor_image.at<double>(i,j) = 1;

           }
       }
       _normal_scan_image = _normal_sensor_image - _normal_image;

       std::string name_raw = path_global+"normal_scan.raw";
       writeMatRaw(QString::fromStdString(name_raw),'w',_normal_scan_image);




}

void TrajectoryGeneratorPlugin::calculate()
{
    bool first_it = _isFirstIteration;

    int id_string = 0;
    QVector3D normal_sensor(0,1,0);
    QVector3D scan_dir(0,0,1);

    qInfo() << "TRAJECTORY GENERATOR PLUGIN. CALCULATE: Calculating density map...";
    // Calculamos densidades sobre la medida---------------------------------------------------------------

    float radio_vecindad = 3.0;
    QVector<float> mean_density_profiles;

    QVector<QVector3D> mean_normals;
    QVector<double> mean_measurements;
    QVector<QVector3D> mean_pointcloud;

    QVector<double> mean_scan_normals;

    if(_isROI)
    {
        qInfo() << "ROI: from" << _ROIs[0] << " - To: " << _ROIs[1];

    }

    ///---------------------------------------------------------------------------------------------------------------------
    //Cálculo densidades y cosas varias del escaneo anterior---------------------------------------------------------------
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

        #pragma omp parallel for
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
           float densidad_punto = puntos_vecindad.size();
           densidades[i] = densidad_punto;
        }

        _density_map.push_back(densidades);
    }


    //Normalizamos las densidades
    float max_density, min_density;
    findMinMaxExcludingZero(_density_map, min_density, max_density); // Obtener el mínimo y el máximo valor en values
    for(int i=0; i<_density_map.size(); i++)
    {
        float mean_density = 0;
        int n=0;
        for(int j=0; j<_density_map[i].size(); j++)
        {
            if(_density_map[i][j]==0)continue;

            _density_map[i][j] = _density_map[i][j]/(max_density-min_density);
            mean_density += _density_map[i][j];
            n++;
        }
        if(n==0)n=1;
        mean_density_profiles.push_back(mean_density/n);
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
    qInfo() << "TRAJECTORY GENERATOR PLUGIN. CALCULATE: Density map calculated.";

    //---------------------------------------------------------------------------------------------------------------------
    ///---------------------------------------------------------------------------------------------------------------------


    ///---------------------------------------------------------------------------------------------------------------------
    //Primera iteración----------------------------------------------------------------------------------------------------
    bool problem2 = false;
    QVector<QVector<int>>areas_low_density;
    if (first_it)
    {
        if(_isROI)
        {
            //Buscamos los puntos que están dentro del ROI
            QVector<int> ids_inRoi;
            for(int p=0; p<_traj_interpolated.size(); p++)
            {
                //-----------------------------
                QVector<QVector3D> points_p = _pointcloud.mid(points_per_profile*p, points_per_profile);
                QVector3D minPos = _ROIs[0];
                QVector3D maxPos = _ROIs[1];
                int p_inRoi_aux=0;
                for(int r=0; r<_resolution; r++)
                {
                    QVector3D point = points_p[r];
                    if (point.x() >= minPos.x() && point.x() <= maxPos.x() &&
                       point.y() >= minPos.y() && point.y() <= maxPos.y() &&
                       point.z() >= minPos.z() && point.z() <= maxPos.z()) {
                        p_inRoi_aux++;
                    }
                    if (p_inRoi_aux > _resolution/10)
                    {
                        ids_inRoi.push_back(p);
                        break;
                    }
                 }
            }

            //Unir esto con el resto de zonas complicadas...
            QVector<int> low_density_ids;
            for(int i=0; i<mean_density_profiles.size(); i++)
            {
                if(mean_density_profiles[i] == 0) continue;
                QVector3D resta = normal_sensor - mean_normals[i];

                if(resta.length()>0.5) //(resta.length()>0.5)
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
            QVector<int> id_nodes;
            id_nodes.push_back(0);
            for(int id_zona=0; id_zona < areas_low_density.size(); id_zona++)
            {
                id_nodes.push_back(areas_low_density[id_zona][0]);
                id_nodes.push_back(areas_low_density[id_zona][areas_low_density[id_zona].size()-1]);
            }
            id_nodes.push_back(_traj_interpolated.size()-1);

//            double desf_wd = working_distance - mean_measurements[0];
//           _pos_sensor.push_back(_traj_interpolated[0].positionXYZ+QVector3D(10,0,0) + desf_wd*normal_sensor);
//           _rpy_sensor.push_back(_traj_interpolated[0].orientationRPY);
            for(int i=0; i<id_nodes.size()-1; i++)
            {
                int id_ini=id_nodes[i];
                int id_end=id_nodes[i+1];
                for(int j=0; j<ids_inRoi.size(); j++)
                {
                    if(ids_inRoi[j]>id_ini && ids_inRoi[j]<id_end)
                    {
                            QVector3D pos_ini = _traj_interpolated[ids_inRoi[j]].positionXYZ;
                            QVector3D rpy_ini = _traj_interpolated[ids_inRoi[j]].orientationRPY;
                            rpy_ini = fixRPYOrientation(rpy_ini);
                            double desf_wd = working_distance - mean_measurements[ids_inRoi[j]];
                            QVector3D mean_n = mean_normals[ids_inRoi[j]];
                            double dot_product = QVector3D::dotProduct(normal_sensor,mean_n.normalized());
                            double angle=std::acos(dot_product);
                            double angle_degrees = (angle*(180.0 / M_PI));
                            if (QVector3D::dotProduct(scan_dir,mean_n.normalized()) < 0) //ESTO CAMBIARLO SEGÚN LA DIRECCIÓN DE ESCANEO!!
                                angle_degrees *= -1;

                            if(angle_degrees>50)angle_degrees-=10;
                            if(angle_degrees<-50)angle_degrees+=10;

                            double incX = working_distance*sin(angle_degrees/180*M_PI);
                            double c = 2*working_distance*sin((angle_degrees/2)/180*M_PI);
                            double incY = sqrt(c*c - incX*incX);

                            QVector3D new_pos = pos_ini;
                            QVector3D new_rpy = rpy_ini;
                            new_rpy.setY(90+angle_degrees);
                            new_pos.setZ(new_pos.z()+incX);
                            new_pos.setY(new_pos.y()+desf_wd-incY);
                            new_pos.setX(new_pos.x()+10);


                            qInfo() << "INI: " << pos_ini << " - " << rpy_ini;
                            qInfo() << "END: " << new_pos << " - " << new_rpy;
                            qInfo() << "normal: " << mean_n;


                            _pos_sensor.push_back(new_pos);
                            _rpy_sensor.push_back(new_rpy);

                    }
                    else
                    {
                        //Calculamos los puntos para la zona
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

                        double incX = working_distance*sin(angle_degrees/180*M_PI);
                        double c = 2*working_distance*sin((angle_degrees/2)/180*M_PI);
                        double incY = sqrt(c*c - incX*incX);

                        QVector3D pos_i = _traj_interpolated[id_ini].positionXYZ;
                        QVector3D rpy_i = _traj_interpolated[id_ini].orientationRPY;

                        rpy_i.setY(90+angle_degrees);

                        pos_i.setZ(pos_i.z()+incX);
                        pos_i.setY(pos_i.y()+desf_wd-incY);
                        pos_i.setX(pos_i.x()+10);


                        _pos_sensor.push_back(pos_i);
                        _rpy_sensor.push_back(rpy_i);
                        break;
                    }
                }

            }

            double desf_wd = working_distance - mean_measurements[_traj_interpolated.size()-1];
           _pos_sensor.push_back(_traj_interpolated[_traj_interpolated.size()-1].positionXYZ+QVector3D(10,0,0) + desf_wd*normal_sensor);
           _rpy_sensor.push_back(_traj_interpolated[_traj_interpolated.size()-1].orientationRPY);
           problem2 = true;

        }

        //--------------------------------------------------------------------------------------------------------------------------
        //------------------------------------------SIN ROIS------------------------------------------------------------------------
        else
        {

            QVector<int> low_density_ids;
            for(int i=0; i<mean_density_profiles.size(); i++)
            {
                if(mean_density_profiles[i] == 0) continue;
                QVector3D resta = normal_sensor - mean_normals[i];

                if(/*mean_density_profiles[i] < 0.6 && (resta.length()>0.3)*/(resta.length()>0.5)) //(resta.length()>0.5)
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

            //------------------------------------------------------------------------------------------------------------------
                //Identificar por qué tiene baja densidad --> UTILIZAR ESTO PARA PONER ALGÚN TIPO DE CÓDIGO O ALGO?
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

                double desf_wd = working_distance - mean_measurements[0];
                int id_pos_ant = 0;
                for(int i=0; i<id_nodes.size()-1; i++)
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

                    double incX = working_distance*sin(angle_degrees/180*M_PI);
                    double c = 2*working_distance*sin((angle_degrees/2)/180*M_PI);
                    double incY = sqrt(c*c - incX*incX);

                    QVector3D pos_i = _traj_interpolated[id_ini].positionXYZ;
                    QVector3D rpy_i = _traj_interpolated[id_ini].orientationRPY;

                    rpy_i.setY(90+angle_degrees);

                    pos_i.setZ(pos_i.z()+incX);
                    pos_i.setY(pos_i.y()+desf_wd-incY);
                    pos_i.setX(pos_i.x()+10);

                    _pos_sensor.push_back(pos_i);
                    _rpy_sensor.push_back(rpy_i);
                }
                 desf_wd = working_distance - mean_measurements[_traj_interpolated.size()-1];
                _pos_sensor.push_back(_traj_interpolated[_traj_interpolated.size()-1].positionXYZ+QVector3D(10,0,0) + desf_wd*normal_sensor);
                _rpy_sensor.push_back(_traj_interpolated[_traj_interpolated.size()-1].orientationRPY);
                problem2 = true;
        }

    }

    ///---------------------------------------------------------------------------------------------------------------------
    //NO Primera iteración---------------------------------------------------------------------------------------------------------------------
    QVector<int> id_sensors_pose_roi;

    if (!first_it)
    {       

        //-------------------------------
        //-------------------------------
        //Buscamos los puntos que están dentro del ROI
        QVector<int> ids_inRoi;
        ids_inRoi.reserve(_traj_interpolated.size());
        ids_inRoi.resize(_traj_interpolated.size());
        qInfo() << ids_inRoi.size();
        if(_isROI)
        {
            for(int p=0; p<_traj_interpolated.size(); p++)
            {
                //-----------------------------
                QVector<QVector3D> points_p = _pointcloud.mid(points_per_profile*p, points_per_profile);
                QVector3D minPos = _ROIs[0];
                QVector3D maxPos = _ROIs[1];
                int p_inRoi_aux=0;
                for(int r=0; r<_resolution; r++)
                {
                    QVector3D point = points_p[r];
                    if (point.x() >= minPos.x() && point.x() <= maxPos.x() &&
                       point.y() >= minPos.y() && point.y() <= maxPos.y() &&
                       point.z() >= minPos.z() && point.z() <= maxPos.z()) {
                        p_inRoi_aux++;
                    }
                    if (p_inRoi_aux > _resolution/10)
                    {

                        ids_inRoi[p]=1;
                        break;
                    }
                    ids_inRoi[p]=0;
                 }
             }

        }
//        //-------------------------------
//        //-------------------------------
//        bool problem = false;
//        for(int i=0; i<_traj_simple.size()-1;i++)
//        {
//            QVector3D pos_ini=_traj_simple[i].positionXYZ;
//            QVector3D pos_end=_traj_simple[i+1].positionXYZ;
//            QVector3D rpy_ini=_traj_simple[i].orientationRPY;
//            QVector3D rpy_end=_traj_simple[i+1].orientationRPY;
//            QVector3D dif_pos= pos_end-pos_ini;

//            //Buscar estas posiciones en _traj_interpolated -> sacar el punto en el pointcloud
//            int id_ini =0;
//            int id_end =0;
//            for(int j=0; j<_traj_interpolated.size(); j++)
//            {
//                if(_traj_interpolated[j].positionXYZ == pos_ini)
//                    id_ini = j;
//                else if(_traj_interpolated[j].positionXYZ == pos_end)
//                {
//                    id_end = j;
//                    break;
//                }
//            }

//            QVector3D pos_ini_scan= mean_pointcloud[id_ini];
//            QVector3D pos_end_scan= mean_pointcloud[id_end];
//            QVector3D dif_scan = pos_end_scan - pos_ini_scan;

//            int id_mid = id_ini + (id_end-id_ini)/2;
//            QVector3D pos_mid=_traj_interpolated[id_mid].positionXYZ;
//            QVector3D rpy_mid=_traj_interpolated[id_mid].orientationRPY;
//            if(rpy_mid.z() !=0 )
//            {
//                rpy_mid.setY(180-rpy_mid.y());
//                rpy_mid.setZ(0);
//            }
//            if(rpy_mid.x() !=0 )
//            {
//                rpy_mid.setX(0);
//            }


//            if(fabs(QVector3D::dotProduct(dif_pos, scan_dir)<10) )
//            {
//                QVector3D rpy_aux =  rpy_end + (rpy_ini - rpy_end)/2;
//                normal_sensor = QVector3D(0,1,0);
//                QVector3D mean_n = QVector3D(0,0,0);
//                for (int j=id_mid-2; j<id_mid+2;j++)
//                {
//                    mean_n += mean_normals[j];
//                }
//                mean_n = mean_n/4;
//                QVector3D new_pos, new_rpy;
//                updatePosFromDifAngle(pos_mid, rpy_mid, rpy_aux, scan_dir,normal_sensor,mean_n,10,working_distance,new_pos,new_rpy);

////                qInfo() << "pos updated: " << new_pos << " - " << new_rpy;

//                QVector3D dif_pos = new_pos-pos_ini;
//                if(QVector3D::dotProduct(dif_pos, scan_dir)<0)
//                {
//                    double incX_max = fabs(QVector3D::dotProduct(pos_mid - pos_ini, scan_dir))*0.2;
//                    double max_alpha = std::asin(incX_max / working_distance)*180/M_PI;
////                    qInfo() << "MAX_ALPHA: "<< max_alpha;
////                    qInfo() << "dif_pos: "<< pos_end - pos_ini;
////                    qInfo() << "incX_max: "<< incX_max;
////                    qInfo() << "scan_dir: "<< scan_dir;

//                    updatePosFromDifAngle(pos_mid, rpy_mid, rpy_aux, scan_dir,normal_sensor,mean_n,fabs(max_alpha),working_distance,new_pos,new_rpy);
//                    new_rpy = fixRPYOrientation(new_rpy);
//                }


////                pointTraj point;
////                point.positionXYZ = new_pos;
////                point.orientationRPY = new_rpy;
////                _traj_simple[i] = point;

////                if(i+1 < _traj_simple.size()-1)
////                    _traj_simple.erase(_traj_simple.begin() + (i+1));

////                problem =true;

//            }

//            //SEGUIR POR AQUÍ!!!!
//            else if(/*QVector3D::dotProduct(dif_pos, scan_dir)<0 ||*/ dif_scan.length() <30 || fabs(QVector3D::dotProduct(dif_pos, scan_dir)<20))
//            {

//                normal_sensor = QVector3D(0,1,0);
//                QVector3D mean_n = QVector3D(0,0,0);
//               for (int j=id_mid-2; j<id_mid+2;j++)
//               {
//                   mean_n += mean_normals[j];
//               }
//               mean_n = mean_n/4;
//               QVector3D new_pos, new_rpy;
//                updatePosFromDifAngle(pos_mid, rpy_mid, rpy_mid, scan_dir,normal_sensor,mean_n,10,working_distance,new_pos,new_rpy);


////                qInfo() << "---POS_INI: "<< pos_ini <<" - " << rpy_ini;
////                qInfo() << "---POS_END: "<< pos_end <<" - " << rpy_end;
////                qInfo() << "---POS_MID: "<< pos_mid <<" - " << rpy_mid;
////                qInfo() << "---POS_UPD: "<< new_pos <<" - " << new_rpy;
//               // qInfo() << "dif_angle: " << dif_angle;

//                pointTraj point;
//                point.positionXYZ = new_pos;
//                point.orientationRPY = new_rpy;
////                _traj_simple[i] = point;

////                qInfo() << "BORRANDO:  " << _traj_simple[i+1].positionXYZ << " / " << _traj_simple[i+1].orientationRPY;
////                if(i+1 < _traj_simple.size()-1)
////                    _traj_simple.erase(_traj_simple.begin() + (i+1));

////                problem =true;
//            }
//            else
//            {
//              //  nueva_traj_simple.push_back(_traj_simple[i]);
//            }

//        }

//        //-------------------------------
//        //-------------------------------


        //----------------------------------------------------------------------------

        //Retocar los puntos y añadir puntos intermedios
        qInfo()<<"Traj simple: " <<_traj_simple.size();
        for (int i=0; i<_traj_simple.size()-1; i++)
        {

            QVector3D pos_ini = _traj_simple[i].positionXYZ;
            QVector3D pos_end = _traj_simple[i+1].positionXYZ;
            QVector3D rpy_ini = _traj_simple[i].orientationRPY;
            QVector3D rpy_end = _traj_simple[i+1].orientationRPY;
            QVector3D diff = pos_end -pos_ini;

            _pos_sensor.push_back( _traj_simple[i].positionXYZ);
            _rpy_sensor.push_back( _traj_simple[i].orientationRPY);


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

            if(ids_inRoi[id_ini]==1) id_sensors_pose_roi.push_back(1);
            else id_sensors_pose_roi.push_back(0);

            if(id_end<id_ini)id_end = _traj_interpolated.size()-1;

            QVector3D pos_ini_scan= mean_pointcloud[id_ini];
            QVector3D pos_end_scan= mean_pointcloud[id_end];
            QVector3D dif_scan = pos_end_scan - pos_ini_scan;

            qInfo() <<"PUNTOS: "<< pos_ini << " - " << pos_end;
//            qInfo() <<"SCAN: "<< pos_ini_scan << " - " << pos_end_scan;


//            qInfo() << "DITANCIA PUNTO PC: " <<dif_scan.length();
//            qInfo() << "DITANCIA POS SENSOR: " <<diff.length();

           // problem = true;
            if((diff.length()>50 || dif_scan.length()>40) /*&& !problem*/) //100
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

                QVector3D pos_mid_scan= mean_pointcloud[id_mid];
                if((pos_mid_scan - pos_ini_scan).length()<15)continue;

                QVector3D pos_mid, rpy_mid, rpy_aux;
                pos_mid = _traj_interpolated[id_mid].positionXYZ;
                rpy_mid = _traj_interpolated[id_mid].orientationRPY;
                rpy_aux = rpy_end + (rpy_ini - rpy_end)/2;

                rpy_aux = fixRPYOrientation(rpy_aux);
                rpy_mid = fixRPYOrientation(rpy_mid);

                normal_sensor = QVector3D(0,1,0);
                QVector3D mean_n = QVector3D(0,0,0);
                if(id_mid-2<0)id_mid=2;
                if(id_mid+2>_traj_interpolated.size()-1)id_mid=_traj_interpolated.size()-3;
                for (int j=id_mid-2; j<id_mid+2;j++)
                {
                    mean_n += mean_normals[j];
                }
                mean_n = mean_n/4;


                double desf_wd = working_distance - mean_measurements[id_mid];
                QVector3D new_pos, new_rpy;
                updatePosFromDifAngle(pos_mid, rpy_mid, rpy_aux, scan_dir,normal_sensor,mean_n,60,working_distance,new_pos,new_rpy, desf_wd);
                new_rpy = fixRPYOrientation(new_rpy);

                QVector3D dif_pos = new_pos-pos_ini;
                qInfo() << "MID: " << pos_mid << " - " << rpy_mid;
                qInfo() << "NEW: " << new_pos << " - " << new_rpy;



                if(QVector3D::dotProduct(dif_pos, scan_dir)<0 || fabs(QVector3D::dotProduct(dif_pos, scan_dir))<10)
                {
                    double incX_max = fabs(QVector3D::dotProduct(pos_mid - pos_ini, scan_dir))*0.1;
                    double max_alpha = std::asin(incX_max / working_distance)*180/M_PI;
                    updatePosFromDifAngle(pos_mid, rpy_mid, rpy_mid, scan_dir,normal_sensor,mean_n,fabs(max_alpha),working_distance,new_pos,new_rpy, desf_wd);
                    new_rpy = fixRPYOrientation(new_rpy);
                }

                dif_pos = pos_end-new_pos;
                if(QVector3D::dotProduct(dif_pos, scan_dir)<0 || fabs(QVector3D::dotProduct(dif_pos, scan_dir))<10)
                {
                    double incX_max = fabs(QVector3D::dotProduct(pos_end-pos_mid, scan_dir))*0.1;
                    double max_alpha = std::asin(incX_max / working_distance)*180/M_PI;
                    updatePosFromDifAngle(pos_mid, rpy_mid, rpy_mid, scan_dir,normal_sensor,mean_n,fabs(max_alpha),working_distance,new_pos,new_rpy, desf_wd);
                    new_rpy = fixRPYOrientation(new_rpy);
                }

                qInfo() << "NEW: " << new_pos << " - " << new_rpy;

                //Solo meterlo si la diferencia de los PC no es muy pequeña

                _pos_sensor.push_back(new_pos);
                _rpy_sensor.push_back(new_rpy);
                qInfo() << "ID_MID: " << id_mid;
                if(ids_inRoi[id_mid]==1) id_sensors_pose_roi.push_back(1);
                else id_sensors_pose_roi.push_back(0);

                problem2 = true;
            }

        }

        _pos_sensor.push_back(_traj_simple[_traj_simple.size()-1].positionXYZ);
        _rpy_sensor.push_back(_traj_simple[_traj_simple.size()-1].orientationRPY);
        id_sensors_pose_roi.push_back(0);


        //SEGÚN MAPA DIFERENCIA DE NORMALES-------------------------------------
        QVector<int> bad_areas;
        for(int i=0; i<mean_scan_normals.size(); i++)
        {
             if (fabs(mean_scan_normals[i])>0.3)
             {
                 bad_areas.push_back(i);
             }
        }
        QVector<QVector<int>>areas_high_diff_normals;
        QVector<int> bad_area_i;

        //PROBLEMA AQUIII
      //  bad_areas.clear();
        if(bad_areas.size()>0 && !problem2)
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

      //  qInfo() << "BAD AREAS: " << areas_high_diff_normals.size();

        for(int i=0; i<areas_high_diff_normals.size();i++)
        {
        //    qInfo() << "BAD AREA: " << i+1;
            if(areas_high_diff_normals[i].size()==0)continue;

            int id_ini = areas_high_diff_normals[i][0];
            int id_end = areas_high_diff_normals[i][areas_high_diff_normals[i].size()-1];

            //------------------------------
           // if(id_end-id_ini < 20) continue;
            id_ini=id_ini;
            id_end=id_end;
            //--------------------------------


            QVector3D pos_area_ini = _traj_interpolated[id_ini].positionXYZ;
            QVector3D pos_area_end = _traj_interpolated[id_end].positionXYZ;

            QVector3D point_scan_area_ini = mean_pointcloud[id_ini];
            QVector3D point_scan_area_end = mean_pointcloud[id_end];

        //    qInfo() << "From: " <<point_scan_area_ini << "- To: " << point_scan_area_end;
           // qInfo() << "RPYS: " <<_traj_interpolated[id_ini].orientationRPY << "- To: " << _traj_interpolated[id_end].orientationRPY;


            bool isPos = false;
            for(int j=0; j<_pos_sensor.size()-1; j++)
            {
                int id=0;
                for(int k=0; k<_traj_interpolated.size(); k++)
                {
                    if(_traj_interpolated[k].positionXYZ == _pos_sensor[j])
                        id = k;
                }
                QVector3D points_scan_i = mean_pointcloud[id];
//                qInfo()<<"Punto: "<< points_scan_i;
//                qInfo()<< QVector3D::dotProduct((points_scan_i-point_scan_area_ini),scan_dir);
//                qInfo()<< QVector3D::dotProduct((point_scan_area_end-points_scan_i),scan_dir);
                if(QVector3D::dotProduct((points_scan_i-point_scan_area_ini),scan_dir)> 0  && QVector3D::dotProduct((point_scan_area_end-points_scan_i),scan_dir)> 0)
                {
//                    qInfo() << "ENTRÉ";
                    //Update point
//                    QVector3D new_pos, new_rpy;
//                    updatePosFromDifAngle(_pos_sensor[j], _rpy_sensor[j], _rpy_sensor[j],scan_dir,normal_sensor,mean_normals[id],20,working_distance,new_pos,new_rpy);
//                    _pos_sensor[j]=new_pos;
//                    _rpy_sensor[j]=new_rpy;
//                    isPos = true;
                }
            }
            isPos=false;
            if(!isPos)
            {
                for(int j=0; j<_pos_sensor.size()-1; j++)
                {
                    int id=0;
                    for(int k=0; k<_traj_interpolated.size(); k++)
                    {
                        if(_traj_interpolated[k].positionXYZ == _pos_sensor[j])
                            id = k;
                    }
                    QVector3D points_scan_i = mean_pointcloud[id];
                    if(QVector3D::dotProduct((points_scan_i-point_scan_area_ini),scan_dir)>0)
                    {
                        //Metemos uno intermedio
                        QVector3D new_pos, new_rpy;
                        int id_mid = id_ini+(id_end-id_ini)/2;
                        QVector3D pos_aux = _traj_interpolated[id_mid].positionXYZ;
                        QVector3D rpy_aux = _traj_interpolated[id_mid].orientationRPY;
                        rpy_aux = fixRPYOrientation(rpy_aux);
                        updatePosFromDifAngle(pos_aux, rpy_aux, rpy_aux,scan_dir,normal_sensor,mean_normals[id_mid],50,working_distance,new_pos,new_rpy);

                        //Meter la posición entre medias
//                        qInfo() << "first: " <<pos_aux << "-" <<rpy_aux;
//                        qInfo() << "INTERMEDIO: " <<new_pos << "-" <<new_rpy;
                        _pos_sensor.insert(j, new_pos);
                        _rpy_sensor.insert(j, new_rpy);
                        if(ids_inRoi[id_mid]==1) id_sensors_pose_roi.insert(j,1);
                        else id_sensors_pose_roi.push_back(0);
                        qInfo() << "ID_MID: " << id_mid;


                        break;
                    }
                }

            }

            //--------------------------------------------------------------------
            //ANTIGUO
/*
            for(int j=0; j<_pos_sensor.size()-1; j++)
            {
                QVector3D pos_ini = _pos_sensor[j];
                QVector3D pos_end = _pos_sensor[j+1];
             //   QVector3D pos_mid = pos_ini + (pos_end-pos_ini)/2;

                QVector3D rpy_ini = _rpy_sensor[j];
                QVector3D rpy_end = _rpy_sensor[j+1];
           //     QVector3D rpy_mid = rpy_ini + (rpy_end-rpy_ini)/2;

                double diff_ini = (pos_ini-pos_area_ini).length();
                double diff_end = (pos_end-pos_area_end).length();

                int id_ini_aux =0;
                int id_end_aux =0;
                for(int j=0; j<_traj_interpolated.size(); j++)
                {
                    if(_traj_interpolated[j].positionXYZ == pos_ini)
                        id_ini_aux = j;
                    else if(_traj_interpolated[j].positionXYZ == pos_end)
                    {
                        id_end_aux = j;
                        break;
                    }
                }
                QVector3D point_scan_pos_ini = mean_pointcloud[id_ini_aux];
                QVector3D point_scan_pos_end = mean_pointcloud[id_end_aux];

                //EL PROBLEMA VA A ESTAR AQUÍ!
                //Si tengo solo un punto ahí debería de ajustarlo también
                if((fabs(diff_ini)+fabs(diff_end))/2 <36) //50 & 50
                {

                    //POSICIÓN INICIAL---------------------------------------------------------------------
                    QVector3D rpy_aux = _traj_interpolated[id_ini].orientationRPY;

                    ////
                    QVector3D mean_n =  mean_normals[id_ini];
                    mean_n = QVector3D(0,0,0);
                   for (int j=id_ini-5; j<id_ini+5;j++)
                   {
                       mean_n += mean_normals[j];
                   }
                   mean_n = mean_n/10;

                   qInfo() << "normals ini: " << mean_n;

                    ////

                   normal_sensor = QVector3D(0,1,0);
                   QVector3D new_pos, new_rpy;
                   updatePosFromDifAngle(pos_ini, rpy_ini, rpy_aux, scan_dir,normal_sensor,mean_n,10,working_distance,new_pos,new_rpy);


                    _pos_sensor[j]=new_pos;
                    _rpy_sensor[j]=new_rpy;



                    //POSICIÓN FINAL---------------------------------------------------------------------
                    rpy_aux = _traj_interpolated[id_end].orientationRPY;

                    ////
                    //mean_n =  mean_normals[id_end];
                     mean_n = QVector3D(0,0,0);
                    for (int j=id_end-5; j<id_end+5;j++)
                    {
                        mean_n += mean_normals[j];
                    }
                    mean_n = mean_n/10;
                    ////

                    qInfo() << "normals end: " << mean_n;

                    normal_sensor = QVector3D(0,1,0);

                    updatePosFromDifAngle(pos_end,rpy_end,rpy_aux,scan_dir,normal_sensor,mean_n,10,working_distance,new_pos, new_rpy);
                        _pos_sensor[j+1]=new_pos;
                        _rpy_sensor[j+1]=new_rpy;

                    break;

                }



                else if (point_scan_pos_ini.z()>point_scan_area_ini.z() && point_scan_pos_ini.z() < point_scan_area_end.z())
                {
                    qInfo() << "ESTOY ENTRAAAAAAAAAAAAAAAAANDO";

                    //Ajustamos el ini para que esté con el ángulo buscado
                    QVector3D rpy_aux = _traj_interpolated[id_ini].orientationRPY;

                    ////
                    QVector3D mean_n =  mean_normals[id_ini];
                    mean_n = QVector3D(0,0,0);
                   for (int j=id_ini-2; j<id_ini+2;j++)
                   {
                       mean_n += mean_normals[j];
                   }
                   mean_n = mean_n/4;

                   qInfo() << "normals ini: " << mean_n;

                    ////

                   normal_sensor = QVector3D(0,1,0);
                   QVector3D new_pos, new_rpy;
                   updatePosFromDifAngle(pos_ini, rpy_ini, rpy_aux, scan_dir,normal_sensor,mean_n,10,working_distance,new_pos,new_rpy);


                    _pos_sensor[j]=new_pos;
                    _rpy_sensor[j]=new_rpy;

                }
//                else if ()
//                {

//                }

            }
*/
          //---------------------------------------------------------------------

        }


    }



    //ASEGURAR QUE SIEMPRE TENGA MOVIMIENTO DE AVANCE
    //Pero esto que me dependa del ROI!!


    if (_isROI)
    {
        int veces=100;
        qInfo() << "SIZE POS: " << _pos_sensor.size();
        qInfo() << "SIZE IDS: " << id_sensors_pose_roi.size();

        while(veces>0 && !first_it)
        {
            veces=0;
            for(int i=0; i<_pos_sensor.size()-2; i++)
            {

                QVector3D pos_ini=_pos_sensor[i];
                QVector3D pos_end=_pos_sensor[i+1];
                QVector3D rpy_end=_rpy_sensor[i+1];
                QVector3D rpy_ini=_rpy_sensor[i];

                QVector3D dif_pos = pos_end-pos_ini;
                double incX = fabs(QVector3D::dotProduct(dif_pos, scan_dir));
                double max_alpha = std::asin(incX / working_distance)*180/M_PI;

                QVector3D dif_rpy = rpy_end - rpy_ini;
                double inc_alpha = fabs(QVector3D::dotProduct(dif_rpy, normal_sensor));

                if((QVector3D::dotProduct(dif_pos, scan_dir)<0) || fabs(QVector3D::dotProduct(dif_pos, scan_dir))<5 /*|| inc_alpha>max_alpha*/)
                {
                    qInfo() << "POS INI: " << pos_ini << " - " << rpy_ini << " is in roi: " << id_sensors_pose_roi[i];
                    qInfo() << "POS END: " << pos_end << " - " << rpy_end << " is in roi: " << id_sensors_pose_roi[i+1];
                    if(id_sensors_pose_roi[i+1]==1 && id_sensors_pose_roi[i]==0)
                    {
                        qInfo() << "ID NO ROI + ID ROI";

                        _pos_sensor.erase(_pos_sensor.begin()+i);
                        _rpy_sensor.erase(_rpy_sensor.begin()+i);
                        id_sensors_pose_roi.erase(id_sensors_pose_roi.begin()+i);

                        QVector3D pos_mid = (pos_ini+pos_end)/2;
                        QVector3D rpy_mid = (rpy_ini+rpy_end)/2;
                        rpy_mid=fixRPYOrientation(rpy_mid);
                        if(i==0)
                        {
                            //Poner al principio una que apunte al primer punto del traj_interpolated con el nuevo ángulo
                            QVector3D pos_pc = mean_pointcloud[0];
                            Eigen::Vector3d direction_vector = unit_vector_from_rpy(rpy_mid.x(), rpy_mid.y(), rpy_mid.z());
                            QVector3D paralel_vector =QVector3D(fabs(direction_vector[1]), fabs(direction_vector[2]), fabs(direction_vector[0]));
                            if(QVector3D::dotProduct(rpy_mid, normal_sensor) < 90) paralel_vector.setZ(-paralel_vector.z());

                            QVector3D new_pos = pos_pc + working_distance*paralel_vector;
                            _pos_sensor.push_front(new_pos);
                            _rpy_sensor.push_front(rpy_mid);
                            id_sensors_pose_roi.push_front(0);


                        }

                        //Adaptamos [i+1] con rpy.y + dif_alpha
                        //eliminamos[i] si no es i==0;

                    }
                    else if (id_sensors_pose_roi[i+1]==0 && id_sensors_pose_roi[i]==1)
                    {
                        qInfo() << "ID ROI + ID NO ROI";

                        //Adaptamos [i] con rpy.y + dif_alpha
                        //eliminamos[i+1] si no es i==_pos_sensor.size();
                        _pos_sensor.erase(_pos_sensor.begin()+i+1);
                        _rpy_sensor.erase(_rpy_sensor.begin()+i+1);
                        id_sensors_pose_roi.erase(id_sensors_pose_roi.begin()+i+1);


                    }
                    else
                    {
                        qInfo() << "AMBOS IGUALES";
                        //Punto medio
                        QVector3D pos_mid = (pos_ini+pos_end)/2;
                        QVector3D rpy_mid = (rpy_ini+rpy_end)/2;

                        rpy_mid=fixRPYOrientation(rpy_mid);

                        _pos_sensor[i]=pos_mid;
                        _rpy_sensor[i]=rpy_mid;


                        if (i+1>= _pos_sensor.size()-1) continue;

                        _pos_sensor.erase(_pos_sensor.begin()+i+1);
                        _rpy_sensor.erase(_rpy_sensor.begin()+i+1);
                        id_sensors_pose_roi.erase(id_sensors_pose_roi.begin()+i+1);


                        if(i==0)
                        {
                            //Poner al principio una que apunte al primer punto del traj_interpolated con el nuevo ángulo
                            QVector3D pos_pc = mean_pointcloud[0];
                            Eigen::Vector3d direction_vector = unit_vector_from_rpy(rpy_mid.x(), rpy_mid.y(), rpy_mid.z());
                            QVector3D paralel_vector =QVector3D(fabs(direction_vector[1]), fabs(direction_vector[2]), fabs(direction_vector[0]));
                            if(QVector3D::dotProduct(rpy_mid, normal_sensor) < 90) paralel_vector.setZ(-paralel_vector.z());

                            QVector3D new_pos = pos_pc + working_distance*paralel_vector;
                            _pos_sensor.push_front(new_pos);
                            _rpy_sensor.push_front(rpy_mid);
                            id_sensors_pose_roi.push_front(0);


                        }

                    }
                    veces++;
                }

            }
        }
    }

    else if(!_isROI)
    {
        int veces=100;
        while(veces>0 && !first_it)
        {
            veces=0;
            for(int i=0; i<_pos_sensor.size()-2; i++)
            {
                QVector3D pos_ini=_pos_sensor[i];
                QVector3D pos_end=_pos_sensor[i+1];
                QVector3D rpy_end=_rpy_sensor[i+1];
                QVector3D rpy_ini=_rpy_sensor[i];

                QVector3D dif_pos = pos_end-pos_ini;
                double incX = fabs(QVector3D::dotProduct(dif_pos, scan_dir));
                double max_alpha = std::asin(incX / working_distance)*180/M_PI;

                QVector3D dif_rpy = rpy_end - rpy_ini;
                double inc_alpha = fabs(QVector3D::dotProduct(dif_rpy, normal_sensor));

                if((QVector3D::dotProduct(dif_pos, scan_dir)<0) || fabs(QVector3D::dotProduct(dif_pos, scan_dir))<5 || inc_alpha>max_alpha)
                {

                    QVector3D pos_mid = (pos_ini+pos_end)/2;
                    QVector3D rpy_mid = (rpy_ini+rpy_end)/2;

                    rpy_mid=fixRPYOrientation(rpy_mid);

                    _pos_sensor[i]=pos_mid;
                    _rpy_sensor[i]=rpy_mid;


                    if (i+1>= _pos_sensor.size()-1) continue;

                    _pos_sensor.erase(_pos_sensor.begin()+i+1);
                    _rpy_sensor.erase(_rpy_sensor.begin()+i+1);

                    if(i==0)
                    {
                        //Poner al principio una que apunte al primer punto del traj_interpolated con el nuevo ángulo
                        QVector3D pos_pc = mean_pointcloud[0];
                        Eigen::Vector3d direction_vector = unit_vector_from_rpy(rpy_mid.x(), rpy_mid.y(), rpy_mid.z());
                        QVector3D paralel_vector =QVector3D(fabs(direction_vector[1]), fabs(direction_vector[2]), fabs(direction_vector[0]));
                        if(QVector3D::dotProduct(rpy_mid, normal_sensor) < 90) paralel_vector.setZ(-paralel_vector.z());

                        QVector3D new_pos = pos_pc + working_distance*paralel_vector;
                        _pos_sensor.push_front(new_pos);
                        _rpy_sensor.push_front(rpy_mid);

                    }
                    veces++;
                }
            }

        }


        veces=100;
        while(veces>0 && !first_it)
        {
            veces=0;
            for(int i=0; i<_pos_sensor.size()-1;i++)
            {
                if((_pos_sensor[i]-_pos_sensor[i+1]).length()<20 && (_rpy_sensor[i]-_rpy_sensor[i+1]).length()<7 )
                {
                    _pos_sensor.erase(_pos_sensor.begin()+i);
                    _rpy_sensor.erase(_rpy_sensor.begin()+i);
                    veces++;
                }
            }
        }
    }



    //ESTO POR AQUÍ
    int veces=100;
    while(veces>0 && first_it)
    {
        veces=0;
        bool is_id_ini=false;
        bool is_id_end=false;
        int id_ini=0, id_end=0;
        for(int i=0; i<_pos_sensor.size()-1;i++)
        {
            qInfo() << i << ": " <<_rpy_sensor[i];
            if((_rpy_sensor[i]-_rpy_sensor[i+1]).length()<5)
            {
                if(!is_id_ini)
                    id_ini=i;
           //     veces++;
                is_id_ini=true;
            }
            else if(is_id_ini && !is_id_end)
            {
                id_end=i;
                is_id_end=true;
            }
        }
        qInfo() << id_ini << " - " << id_end;

        int id_mid = (id_end+id_ini)/2;
        for(int i=id_end-1; i>id_ini; i--)
        {
            if(i==id_mid)continue;
            _pos_sensor.erase(_pos_sensor.begin()+i);
            _rpy_sensor.erase(_rpy_sensor.begin()+i);
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

double TrajectoryGeneratorPlugin::costFunction()
{
    qInfo() << "PROBANDO COST FUNCTION";
    precalculate();

   // Mapa diferencia de normales
    double rango_angle = 30;
    cv::Mat I_normal_norm = _normal_scan_image;
    auto normalize = [&](double& value, const int* position) -> void {
           double angle = std::acos(value) * (180 / CV_PI);
           value = 90 - angle;
           if (std::fabs(value) <= rango_angle)
               value /= rango_angle;
           else
               value = 1;
       };
    I_normal_norm.forEach<double>(normalize);


    double value_normals=0;
    double value_distances=0;
    int n_points=0;
    // Mapa diferencia distancias
    double rango_trabajo = working_distance/10;
    cv::Mat I_scan_norm = cv::Mat::zeros(I_normal_norm.rows, I_normal_norm.cols, CV_64FC1);
    cv::Mat maskROI = cv::Mat::zeros(I_normal_norm.rows, I_normal_norm.cols, CV_8UC1);

    QVector3D minPos, maxPos;
    if(_isROI)
    {
        minPos = _ROIs[0]; // posición mínima del ROI
        maxPos = _ROIs[1]; // posición máxima del ROI
    }

    // Copiar los datos del vector a la matriz cv::Mat
    for (int i = 0; i < _scan_image.size()/_resolution; i++) {
        QVector<QVector3D> scan_line = _scan_image.mid(points_per_profile*i, points_per_profile);
        QVector<QVector3D> points_pc = _pointcloud.mid(points_per_profile*i, points_per_profile);

        for(int r=0; r<_resolution; r++)
        {
            double value = working_distance - scan_line[r].z();
            if(value <= rango_trabajo)
                I_scan_norm.at<double>(i, r) = value / rango_trabajo;
            else
                I_scan_norm.at<double>(i, r) = 1;

            //Ir calculando el valor de cada una
            if(scan_line[r].z() > 0)
            {
                value_normals += I_normal_norm.at<double>(i,r);
                value_distances += I_scan_norm.at<double>(i,r);
                n_points++;
            }
            //Comprobar si el punto_pc está en el ROI y poner a 1 en caso afirmativo
            if(_isROI)
            {
                QVector3D point = points_pc[r];
                if (point.x() >= minPos.x() && point.x() <= maxPos.x() &&
                       point.y() >= minPos.y() && point.y() <= maxPos.y() &&
                       point.z() >= minPos.z() && point.z() <= maxPos.z()) {
                       // Si el punto está dentro del ROI, establecer el valor correspondiente en la imagen binaria a 1

                       maskROI.at<uchar>(i, r) = 1;
                   }
            }

        }

    }

    qInfo() << "normals: " << value_normals/n_points;



    // Mapa densidades


    // Sacar mapa de ROIs


    //FUNCIÓN DE COSTE
    double w_normals = 0.5;
    double w_distance = 0.5;
    double FC = w_normals*value_normals/n_points + w_distance*value_distances/n_points;
    qInfo() << "FC sin ROIS: " << FC;

    if(_isROI)
    {
        std::string name_raw = _path.toStdString()+"maskROI.raw";
        writeMatRaw(QString::fromStdString(name_raw),'w',maskROI);

        int n_points_ROI = 0;//cv::countNonZero(maskROI);
        int n_points_nonROI =0;// n_points - n_points_ROI;

        double value_normals_ROI=0,value_normals_nonROI=0, value_distances_ROI=0, value_distances_nonROI=0;
        for (int i = 0; i < _scan_image.size() / _resolution; i++) {
            QVector<QVector3D> scan_line = _scan_image.mid(points_per_profile*i, points_per_profile);
            for (int r = 0; r < _resolution; r++) {
                if(scan_line[r].z() > 0)
                {
                    if (maskROI.at<uchar>(i, r) == 1) {
                        value_normals_ROI += I_normal_norm.at<double>(i, r);
                        value_distances_ROI += I_scan_norm.at<double>(i, r);
                        n_points_ROI++;
                    } else {
                        value_normals_nonROI += I_normal_norm.at<double>(i, r);
                        value_distances_nonROI += I_scan_norm.at<double>(i, r);
                        n_points_nonROI++;
                    }
                }
            }
        }
        double w_ROI = n_points_ROI/(n_points_ROI+n_points_nonROI);
        double w_nonROI = 1-w_ROI;

        double FC_ROI = w_normals*value_normals_ROI/n_points_ROI + w_distance*value_distances_ROI/n_points_ROI;
        double FC_nonROI = w_normals*value_normals_nonROI/n_points_nonROI + w_distance*value_distances_nonROI/n_points_nonROI;

        FC = w_ROI*FC_ROI + (w_nonROI)*FC_nonROI;
        qInfo() << "FC con ROIS: " << FC;


    }




    return FC;

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

void TrajectoryGeneratorPlugin::setROIS(QVector3D minPosBB, QVector3D maxPosBB)
{
    _isROI = true;
    _ROIs.push_back(minPosBB);
    _ROIs.push_back(maxPosBB);
}
