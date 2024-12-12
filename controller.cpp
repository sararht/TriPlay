#include "controller.h"
#include <ctime>
#include <QDir>
#include <QXmlStreamWriter>

#include <vtkDoubleArray.h>
#include <vtkFieldData.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtktools.h>
#include <vtkIdTypeArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

#include "fitplane3d.h"

#include <QtAlgorithms>

#include <QMessageBox>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include "bspline3d.h"

inline void printQVector3D(QVector3D v)
{
    std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
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


double findMostSimilarValueQVector(std::vector<double> &v, double value, int &id)
{
    std::vector<double> distances;
    for (uint i=0; i<v.size();i++)
        distances.push_back(fabs(v[i]-value));

    id = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
    return v[id];

}

controller::controller(QObject *parent) : QObject(parent)
{

}

inline QVector3Dd ToEulerAngles(QQuaternion &q)
{
    QVector3Dd angles;

    q.normalize();
    double test = q.x()*q.y() + q.z()*q.scalar();
    double roll, pitch, yaw;

    if (test > 0.499) { // singularity at north pole
            pitch = 2 * atan2(q.x(),q.scalar());
            yaw = PI/2;
            roll = 0;
        }
    else if (test < -0.499) { // singularity at south pole
        pitch = -2 * atan2(q.x(),q.scalar());
        yaw = -PI/2;
        roll = 0;
    }

    double sqx = q.x()*q.x();
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();

    pitch = atan2(2*q.y()*q.scalar()-2*q.x()*q.z() , 1 - 2*sqy - 2*sqz);
    yaw = asin(2*test);
    roll = atan2(2*q.x()*q.scalar()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqz);




/*
    double x = q.x();
    double y=q.y();
    double z=q.z();
    double scalar = q.scalar();

   QVector3Dd angles;
   double pitch, yaw, roll;
   double sinr_cosp = 2 * (q.scalar()* q.x() + q.y() * q.z());
   double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
   roll = std::atan2(sinr_cosp, cosr_cosp);

   // pitch (y-axis rotation)
   double sinp = 2 * (q.scalar() * q.y()- q.z() * q.x());
   if (std::abs(sinp) >= 1)
       pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
   else
       pitch = std::asin(sinp);

   // yaw (z-axis rotation)
   double siny_cosp = 2 * (q.scalar() * q.z() + q.x() * q.y());
   double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
   yaw = std::atan2(siny_cosp, cosy_cosp);

*/
   angles.setX(roll);
   angles.setY(pitch);
   angles.setZ(yaw);

   angles = angles*180/PI;
       return angles;
}


//bool controller::updatePolydataModel(vtkSmartPointer<vtkSTLReader> reader)
//{
//    reader_model = reader;
//}

bool controller::updatePolydataModel(vtkSmartPointer<vtkPolyData> poly)
{
    this->polydata_model = poly;
}

//bool controller::updatePolydataModel(QString name_file)
//{
//    vtkSmartPointer<vtkSTLReader> reader;
//    reader = vtkSmartPointer<vtkSTLReader>::New();
//    reader->SetFileName(name_file.toStdString().c_str());
//    reader->Update();

//    reader_model = reader;
//}

void controller::trajectory(QVector3Dd pos_ini, QVector3Dd pos_end, QQuaternion q_ini, QQuaternion q_end, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree)
{
    clock_t tStart = clock();

    QVector<QVector<double>> distances_sensor(frames);
    QVector<QVector<QVector3Dd>> total_data_sensor(frames);

    QVector3Dd cte = (pos_end - pos_ini) / frames;
    for (int i=0; i<frames; i++)
    {
        QVector3Dd euler_i;
        QQuaternion q_i;

        QVector3Dd pos = pos_ini + cte*i;

        if (q_ini != q_end)
        {
            double t = double(i)/double(frames);

            q_i = QQuaternion::slerp(q_ini, q_end, t);
            euler_i = ToEulerAngles(q_i);
        }
        else
            euler_i = ToEulerAngles(q_ini);

        double roll = euler_i.z();
        double pitch = euler_i.x();
        double yaw = -euler_i.y();


        sensor new_sensor_model = sensor(pos,roll,pitch,yaw,w_range, w_distance,resolution,FOV*PI/180, uncertainty);
        new_sensor_model.getMeasurement(tree ,true);

        distances_sensor[i] = (new_sensor_model.distances_data);
        total_data_sensor[i] = (new_sensor_model.sensor_data);

        emit frameDone(i,frames,new_sensor_model);

    }
    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);


    //Guardar sensor_data
    std::ofstream outputFile("../resultados/sensorData_.txt");
    for (int i=0; i<total_data_sensor.size(); i++)
    {
        for (int j=0; j<total_data_sensor[0].size(); j++)
        {
            outputFile << total_data_sensor[i][j].x() << ",";
            outputFile << total_data_sensor[i][j].y() << ",";
            outputFile << total_data_sensor[i][j].z();

            if (j!= total_data_sensor[0].size()-1)
                 outputFile << ",";
        }
        outputFile << std::endl;
    }
    outputFile.close();

    std::ofstream outputFile2("../resultados/sensorDataDistances_.txt");
    for (int i=0; i<distances_sensor.size(); i++)
    {
        for (int j=0; j<distances_sensor[0].size(); j++)
        {
            outputFile2 << distances_sensor[i][j];
            if (j!= distances_sensor[0].size()-1)
                 outputFile2 << ",";
        }
        outputFile2 << std::endl;

    }
    outputFile2.close();


}

void saveData(QString path, QString file_name, QVector<QVector<QVector3Dd>> total_data_sensor )
{
    QDir dir;

    if(!dir.exists(path))
        dir.mkdir(path);

    QFile file(path + file_name);
    if (file.open(QIODevice::WriteOnly))
    {
        QTextStream stream(&file);
        for (int i=0; i<total_data_sensor.size(); i++)
        {
            for (int j=0; j<total_data_sensor[0].size(); j++)
            {
                stream << total_data_sensor[i][j].x() << "," ;
                stream << total_data_sensor[i][j].y() << ",";
                stream << total_data_sensor[i][j].z();

                if (j!= total_data_sensor[0].size()-1)
                     stream << "\n";
            }
            stream << endl;

        }
    }
    file.close();
}

void saveTraj(QString path, QString file_name, QVector<QVector3Dd> pos_data, QVector<QVector3Dd> rpy_data,
              double FPS, double vel, double FOV, int PPP, double uncertainty,
              bool complete =  true)
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

void controller::getTrajectoryNodesFromFile(QVector<QVector3Dd> pos_dataTraj,QVector<QVector3Dd> rpy_dataTraj,double fov,double resolution,double w_range, double w_distance,double uncertainty,KDNode tree, QString path)
{




    if (pos_dataTraj.size()<2)
    {
        //EMIT ui->listWidget->addItem("PUNTOS INSUFICIENTES");
    }

    else
    {
      /*
        int frames_i=0;
        QVector<int> vFrames;
        int frames_totales = 0;
        for (int j=0; j<nodes.size()-1; j++)
        {
            QVector3Dd vDist = nodes[j+1].pos() - nodes[j].pos();
            double dist = vDist.length()/1000; //m
            int frames_aux = dist/vel *frames;

            vFrames.push_back(frames_aux);
            frames_totales = frames_totales + frames_aux; //NO VA BIEEEEEN
        }

        QVector<QVector3Dd> pos_sensor;
        QVector<QVector3Dd> rpy_sensor;
*/


        clock_t tStart = clock();
        for (int j=0; j<1; j++)
        {

            cv::Mat raw(pos_dataTraj.size(), resolution, CV_64FC1);
            cv::Mat raw_error(pos_dataTraj.size(), resolution, CV_64FC1);
            cv::Mat raw_luminosidad = cv::Mat::zeros(pos_dataTraj.size(), resolution, CV_64FC1);
            cv::Mat raw_normal_scan(pos_dataTraj.size(), resolution, CV_64FC1);
            cv::Mat raw_normals(pos_dataTraj.size(), resolution, CV_64FC1);
            cv::Mat raw_normals_sensor(pos_dataTraj.size(), resolution, CV_64FC1);


            //QVector3Dd pos_ini = pos_dataTraj[0];
            //QVector3Dd pos_end = pos_dataTraj[pos_dataTraj.size()-1];
          //  QQuaternion q_ini = nodes[j].q();
          //  QQuaternion q_end = nodes[j+1].q();

            QVector<QVector<double>> distances_sensor(pos_dataTraj.size());
            QVector<QVector<QVector3Dd>> normals(pos_dataTraj.size());
            QVector<QVector<QVector3Dd>> total_data_sensor(pos_dataTraj.size());
            QVector<QVector<QVector3Dd>> total_data_sensor_error(pos_dataTraj.size());
            QVector<QVector<QVector3Dd>> total_points_real(pos_dataTraj.size());
            QVector<QVector<QVector3Dd>> total_points_real_error(pos_dataTraj.size());

            std::cout <<"HEY2"<<std::endl;


            clock_t tStart_ = clock();

            for (int i=0; i<pos_dataTraj.size(); i++)
            {
            sensor new_sensor_model = sensor(pos_dataTraj[i],rpy_dataTraj[i].x(),rpy_dataTraj[i].y(),rpy_dataTraj[i].z(),w_range, w_distance,resolution,fov*PI/180, uncertainty);
            new_sensor_model.getMeasurement(tree,true);


       //     std::cout << "Roll: " << rpy_dataTraj[i].x() << ", Pitch: " << rpy_dataTraj[i].y() << ", Yaw: " << rpy_dataTraj[i].z() << std::endl;

            distances_sensor[i] = (new_sensor_model.distances_data);
            total_data_sensor[i] = (new_sensor_model.sensor_data);
            total_points_real[i] = (new_sensor_model.sensor_data_points_real);
            total_data_sensor_error[i] = (new_sensor_model.sensor_data_error);
            total_points_real_error[i] =  (new_sensor_model.sensor_data_points_real_error);
            normals[i] = new_sensor_model.normal_data;

            QVector3D rpy_aux = rpy_dataTraj[i].toQVector3D();
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


            for(int r=0;r<resolution;r++)
            {
                raw.at<double>(i,r)=total_data_sensor[i][r].z()*100;
                raw_error.at<double>(i,r)=total_data_sensor_error[i][r].z()*100;
                raw_normals.at<double>(i,r)=normals[i][r].y();
                raw_normals_sensor.at<double>(i,r)=normal_sensor.y();
                if (normals[i][j].toQVector3D()==QVector3D(0,0,0))
                    raw_normals.at<double>(i,j) = 1;

                if (total_data_sensor_error[i][r].z() != 0)
                    raw_luminosidad.at<double>(i,r)=1;
            }

            emit frameDone(i, pos_dataTraj.size(),new_sensor_model);
            }


            //SAVE DATA
            std::stringstream name, name2, name3, name4, name_raw_error, name_raw_luminosidad, nameN, name_raw;
           // time_t now = time(0);
            //path << "../resultados/" << std::ctime(&now);
            nameN << "/normal_data0" << 0 << ".txt";
            name << "/step" << 0 << ".txt";
            name2 << "/step_00_real" << ".txt";
            name3 << "/step_error0" << 0 << ".txt";
            name4 << "/step_real_error" << 0 << ".txt";
            name_raw_error << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0') << j << "_orig.raw";
            name_raw_luminosidad << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0') << j << "_luminosidad.raw";
            name_raw << path.toStdString() << "/normal_scan.raw";


            saveData(path, QString::fromStdString(nameN.str()),normals);
            saveData(path, QString::fromStdString(name.str()),total_data_sensor);
            saveData(path, QString::fromStdString(name2.str()),total_points_real);
            saveData(path, QString::fromStdString(name3.str()),total_data_sensor_error);
            saveData(path, QString::fromStdString(name4.str()),total_points_real_error);

            writeMatRaw(QString::fromStdString(name_raw_error.str()),'w',raw_error);
            writeMatRaw(QString::fromStdString(name_raw_luminosidad.str()),'w',raw_luminosidad*20);

            raw_normal_scan = raw_normals_sensor - raw_normals;

            writeMatRaw(QString::fromStdString(name_raw.str()),'w',raw_normal_scan);

           // saveData(name.str(), total_data_sensor);
            printf("Time taken node %d: %.2fs\n", j, (double)(clock() - tStart_)/CLOCKS_PER_SEC);

        }
        printf("Total time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);


    }
    qInfo() << "---------EMIT END TRAJ------";
    emit endTrajFrom();

}

void controller::getTrajectoryNodes(QVector<trajectoryNode> nodes, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path, bool onlyEven = false)
{


     qInfo() << "Executing trajectory with: ";
     qInfo() << "Velocity: " << vel << "m/s";
     qInfo() << "Resolution: " << resolution << "points/profile";
     qInfo() << "FOV: " << FOV << "º";
     qInfo() << "FPS: " << frames << "frames/s";
     qInfo() << "Uncertainty: " << uncertainty << "μm";

    //-GUARDAR DATOS CONJUNTOS----------------------------------------------------------------
    QVector<QVector<double>> distances_sensor_all;
    QVector<QVector<QVector3Dd>> total_data_sensor_all;
    QVector<QVector<QVector3Dd>> total_data_sensor_error_all;
    QVector<QVector<QVector3Dd>> total_points_real_all;
    QVector<QVector<QVector3Dd>> total_points_real_error_all;
    QVector<QVector<QVector3Dd>> normal_data_sensor_all;
    QVector<QVector3Dd> pos_sensor_buena_all;
    QVector<QVector3Dd> rpy_sensor_buena_all;

    cv::Mat raw_complete;
    cv::Mat raw_luminosidad_complete;
    cv::Mat raw_normal_scan;
    cv::Mat raw_normals;
    cv::Mat raw_normals_sensor;
    //-----------------------------------------------------------------

    if (nodes.size()<2)
    {
        //EMIT ui->listWidget->addItem("PUNTOS INSUFICIENTES");
    }

    else
    {
        int frames_i=0;
        QVector<int> vFrames;
        int frames_totales = 0;
        QVector3Dd vDist;
        double mm_resolution, mm_leng;

        for (int j=0; j<nodes.size()-1; j++)
        {
          //  if (onlyEven && j%2!=0)
          //      continue;

            vDist = nodes[j+1].pos() - nodes[j].pos();
            double dist = vDist.length()/1000.0; //m
            mm_leng = dist*1000.0;
            int frames_aux = dist/vel *frames;

            if(frames_aux==0) frames_aux=1;
            vFrames.push_back(frames_aux);
            frames_totales = frames_totales + frames_aux; //NO VA BIEEEEEN
        }


        std::cout << "Frames: " << frames_totales << std::endl;



        double start = omp_get_wtime();

        raw_complete = cv::Mat::zeros(frames_totales, resolution, CV_64FC1);
        raw_luminosidad_complete = cv::Mat::zeros(frames_totales, resolution, CV_64FC1);
        raw_normal_scan= cv::Mat::zeros(frames_totales, resolution, CV_64FC1);
        raw_normals= cv::Mat::zeros(frames_totales, resolution, CV_64FC1);
        raw_normals_sensor= cv::Mat::zeros(frames_totales, resolution, CV_64FC1);



    double x_fov;

        //////////////////////

        QVector<QVector3Dd> pos_sensor;
        QVector<QVector3Dd> rpy_sensor;

        int frames_ant = 0;

        for (int j=0; j<nodes.size()-1; j++)
        {

            if (onlyEven && j%2!=0)
            {
                frames_i = frames_i + vFrames[j];
                continue;
            }

            cv::Mat raw(vFrames[j], resolution, CV_64FC1);
            cv::Mat raw_error(vFrames[j], resolution, CV_64FC1);
            cv::Mat raw_luminosidad = cv::Mat::zeros(vFrames[j], resolution, CV_64FC1);

            QVector3Dd pos_ini = nodes[j].pos();
            QVector3Dd pos_end = nodes[j+1].pos();
            QQuaternion q_ini = nodes[j].q();
            QQuaternion q_end = nodes[j+1].q();

            double cte_x= (pos_end.x() - pos_ini.x()) / vFrames[j] ;
            double cte_y= (pos_end.y() - pos_ini.y()) / vFrames[j] ;
            double cte_z= (pos_end.z() - pos_ini.z()) / vFrames[j] ;

            QVector<QVector<double>> distances_sensor(vFrames[j]);
            QVector<QVector<QVector3Dd>> total_data_sensor(vFrames[j]);
            QVector<QVector<QVector3Dd>> total_data_sensor_error(vFrames[j]);
            QVector<QVector<QVector3Dd>> total_points_real(vFrames[j]);
            QVector<QVector<QVector3Dd>> total_points_real_error(vFrames[j]);
            QVector<QVector<QVector3Dd>> normal_data_sensor(vFrames[j]);
            QVector<QVector3Dd> pos_sensor_buena(vFrames[j]);
            QVector<QVector3Dd> rpy_sensor_buena(vFrames[j]);



            double start_ = omp_get_wtime();

            std::cout << "Scanning... "<<vFrames[j] << std::endl;

            volatile bool flag_cancel=false;
            int NUM_THREADS = 10;
            omp_set_dynamic(0);
            omp_set_num_threads(NUM_THREADS);
            #pragma omp parallel //for num_threads(11)
            {
                int id_thread = omp_get_thread_num();
                for (int i=id_thread; i<vFrames[j]; i=i+NUM_THREADS)
                {

                    if(flag_cancel)continue;
                  //  i=i+id_thread*(vFrames[j]/NUM_THREADS);

                  //  QVector3Dd pos_i = pos_ini + i*cte;
                    QVector3Dd euler_i;
                    QQuaternion q_i;

                    double pos_x = pos_ini.x() + i*cte_x;
                    double pos_y = pos_ini.y() + i*cte_y;
                    double pos_z = pos_ini.z() + i*cte_z;

                    double roll, pitch, yaw;
                    if (q_ini != q_end)
                    {
                        double t = double(i)/double(vFrames[j]);
                        q_i = QQuaternion::slerp(q_ini, q_end, t);
                        euler_i = ToEulerAngles(q_i);
                        QVector3D euler_i_ = q_i.toEulerAngles();
                        roll = euler_i_.z();
                        pitch = euler_i_.x();
                        yaw = -euler_i_.y();
                    }
                    else
                    {
                        q_i =q_ini;
                        euler_i = ToEulerAngles(q_ini);
                        QVector3D euler_i_ = q_ini.toEulerAngles();
                        roll = euler_i_.z();
                        pitch = euler_i_.x();
                        yaw = -euler_i_.y();
                    }


                    sensor new_sensor_model = sensor(QVector3Dd(pos_x,pos_y,pos_z),q_i,w_range, w_distance,resolution,FOV*PI/180, uncertainty);

                    new_sensor_model.getMeasurement(tree,true);


                    x_fov = new_sensor_model.x_fov;

                    pos_sensor.push_back(QVector3Dd(pos_x,pos_y,pos_z));
                    rpy_sensor.push_back(QVector3Dd(roll,pitch,yaw));

                    distances_sensor[i] = (new_sensor_model.distances_data);
                    total_data_sensor[i] = (new_sensor_model.sensor_data);
                    total_points_real[i] = (new_sensor_model.sensor_data_points_real);
                    total_data_sensor_error[i] = (new_sensor_model.sensor_data_error);
                    total_points_real_error[i] =  (new_sensor_model.sensor_data_points_real_error);
                    normal_data_sensor[i] = (new_sensor_model.normal_data);
                    pos_sensor_buena[i] = QVector3Dd(pos_x, pos_y, pos_z);
                    rpy_sensor_buena[i] = QVector3Dd(roll,pitch,yaw);

                    if(i==0)
                    {
                        mm_resolution = total_data_sensor[i][total_data_sensor[i].size()-1].x() - total_data_sensor[i][0].x();
                    }

                    for(int r=0;r<resolution;r++)
                    {
                        raw.at<double>(i,r)=total_data_sensor[i][r].z()*100;
                        raw_error.at<double>(i,r)=total_data_sensor_error[i][r].z()*100;

                        if (total_data_sensor_error[i][r].z() != 0)
                            raw_luminosidad.at<double>(i,r)=1;


                    }


//                    emit frameDone(frames_i, frames_totales,new_sensor_model);
                    emit frameDoneTraj(nodes, vFrames, j, frames_i, new_sensor_model); //ESTE

                    #pragma omp atomic
                           frames_i++;

                    //Comprobar si cancelar:
                    if(this->cancelling)
                    {
                        flag_cancel = true;
                    }

                }

            }


            //SAVE DATA
            std::stringstream name, name2, name3, name4, name5, name_traj, name_traj_simple, name_raw, name_raw_error,name_raw_perlin, name_raw_luminosidad;
            name << "/step" << std::setw(2) << std::setfill('0') << j << ".txt";
            name2 << "/step_" << std::setw(2) << std::setfill('0') << j << "_real.txt";
            name3 << "/step_error" << std::setw(2) << std::setfill('0') << j << ".txt";
            name4 << "/step_real_error" << std::setw(2) << std::setfill('0') << j << ".txt";
            name5 << "/normal_data" << std::setw(2) << std::setfill('0') << j << ".txt";
            name_traj << "/traj_sensor" << std::setw(2) << std::setfill('0') << j << ".xml";
            name_traj_simple << "/step_" << std::setw(2) << std::setfill('0') << j << "_traj.txt";
            name_raw << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0')  << j << "_orig_no_error.raw";
            name_raw_error << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0') << j << "_orig.raw";
            name_raw_perlin << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0') << j << "_orig_perlin.raw";
            name_raw_luminosidad << path.toStdString() << "/step_" << std::setw(2) << std::setfill('0') << j << "_luminosidad.raw";


//            saveData(path, QString::fromStdString(name.str()),total_data_sensor);
//            saveData(path, QString::fromStdString(name2.str()),total_points_real);
//            saveData(path, QString::fromStdString(name3.str()),total_data_sensor_error);
//            saveData(path, QString::fromStdString(name4.str()),total_points_real_error);
//            saveData(path, QString::fromStdString(name5.str()),normal_data_sensor);
//            saveTraj(path, QString::fromStdString(name_traj.str()),pos_sensor_buena,rpy_sensor_buena);


            //Datos conjuntos--------------------------------------------------------------------------------
            distances_sensor_all += distances_sensor;
            total_data_sensor_all += total_data_sensor;
            total_data_sensor_error_all += total_data_sensor_error;
            total_points_real_all += total_points_real;
            total_points_real_error_all += total_points_real_error;
            normal_data_sensor_all += normal_data_sensor;
            pos_sensor_buena_all += pos_sensor_buena;
            rpy_sensor_buena_all += rpy_sensor_buena;



            //-----------------------------------------------------------------------------------------------


            //Guardar traj
//            saveTraj(path, QString::fromStdString(name_traj_simple.str()),pos_sensor_buena,rpy_sensor_buena,false);
////            writeMatRaw(QString::fromStdString(name_raw.str()),'w',raw);

//            writeMatRaw(QString::fromStdString(name_raw_error.str()),'w',raw_error);
//            writeMatRaw(QString::fromStdString(name_raw_luminosidad.str()),'w',raw_luminosidad*20);


            ///Perlin noise---------------

            double freq_x = 113/2;//mm_resolution ;//1.0/mm_resolution;
            double freq_y = mm_leng/2;//1.0/mm_length;

        //    std::cout << "freqs: " << freq_x << ", " << freq_y << ", mm_res: " << mm_resolution << ", mm_leng: " << mm_leng << std::endl;
        //    std::cout << "cols: " << raw_error.cols << ", rows: " << raw_error.rows << std::endl;

            cv::Mat perlin_noise = CreatePerlinNoiseImage(raw_error.size(), freq_y, freq_y);
            cv::Mat perlin_noise_raw;
            perlin_noise.convertTo(perlin_noise_raw, CV_64FC1);

            cv::multiply(perlin_noise_raw,raw_luminosidad,perlin_noise_raw);

       //     writeMatRaw(QString::fromStdString(name_raw_perlin.str()),'w',(raw_error+ perlin_noise_raw/256 *0.02*100));

            ///---------------------------



      //      printf("Node took %f seconds\n", omp_get_wtime() - start_);

            if(cancelling)
            {
                this->cancelling = false;
                break;
            }

        }
        printf("Total scan took %f seconds\n", omp_get_wtime() - start);
    }

    //GUARDARLO TODO JUNTO------------------------------------------------------------------------
    //SAVE DATA
    std::stringstream name, name2, name3, name4, name5, name_traj, name_traj_simple, name_raw_complete,name_raw_luminosidad_complete, name_raw, name_raw_error,name_raw_perlin, name_raw_luminosidad;
    name << "/step00.txt";
    name2 << "/step_00_real.txt";
    name3 << "/step_error00.txt";
    name4 << "/step_real_error00.txt";
    name5 << "/normal_data00" << ".txt";
    name_traj << "/traj_sensor00.xml";
    name_traj_simple << "/step_00_traj.txt";
    name_raw_complete << path.toStdString() << "/step_00_orig.raw";
    name_raw_luminosidad_complete << path.toStdString() << "/step_00_luminosidad.raw";
    name_raw << path.toStdString() << "/normal_scan.raw";

//    name_raw << path.toStdString() << "/step_" <<"_conjunto" << "_orig_no_error.raw";
//    name_raw_error << path.toStdString() << "/step_" <<"_conjunto" << "_orig.raw";
//    name_raw_perlin << path.toStdString() << "/step_" << "_conjunto" << "_orig_perlin.raw";
//    name_raw_luminosidad << path.toStdString() << "/step_" << "_conjunto" << "_luminosidad.raw";

    saveData(path, QString::fromStdString(name.str()),total_data_sensor_all);
    saveData(path, QString::fromStdString(name2.str()),total_points_real_all);
    saveData(path, QString::fromStdString(name3.str()),total_data_sensor_error_all);
    saveData(path, QString::fromStdString(name4.str()),total_points_real_error_all);
    saveData(path, QString::fromStdString(name5.str()),normal_data_sensor_all);
    saveTraj(path, QString::fromStdString(name_traj.str()),pos_sensor_buena_all,rpy_sensor_buena_all,frames, vel, FOV, resolution, uncertainty);

    saveTraj(path, QString::fromStdString(name_traj_simple.str()),pos_sensor_buena_all,rpy_sensor_buena_all,frames,  vel, FOV, resolution, uncertainty, false);


    for (int i=0; i<total_data_sensor_error_all.size();i++)
    {
        QVector3D rpy_aux = rpy_sensor_buena_all[i].toQVector3D();
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


        for(int r=0; r<resolution; r++)
        {
            raw_complete.at<double>(i,r)=total_data_sensor_error_all[i][r].z()*100;
            if (total_data_sensor_error_all[i][r].z() != 0)
                raw_luminosidad_complete.at<double>(i,r)=1;

            raw_normals.at<double>(i,r)=std::acos(normal_data_sensor_all[i][r].z())*180/M_PI;
            raw_normals_sensor.at<double>(i,r)= 180 - rpy_aux.y();//normal_sensor.z();
            if (normal_data_sensor_all[i][r].toQVector3D()==QVector3D(0,0,0))
            {
                raw_normals.at<double>(i,r) = std::acos(1)*180/M_PI;
                raw_normals_sensor.at<double>(i,r)= std::acos(1)*180/M_PI;//normal_sensor.z();
            }
        }
    }



    writeMatRaw(QString::fromStdString(name_raw_complete.str()),'w',raw_complete);
    writeMatRaw(QString::fromStdString(name_raw_luminosidad_complete.str()),'w', raw_luminosidad_complete);
    raw_normal_scan = raw_normals_sensor - raw_normals;

    writeMatRaw(QString::fromStdString(name_raw.str()),'w',raw_normal_scan);

    //            writeMatRaw(QString::fromStdString(name_raw.str()),'w',raw);

//    writeMatRaw(QString::fromStdString(name_raw_error.str()),'w',raw_error);
//    writeMatRaw(QString::fromStdString(name_raw_luminosidad.str()),'w',raw_luminosidad*20);


    std::cout << "Trajectory saved...." << std::endl;
    emit endTrajFrom();

}



void controller::insertDefectSelectionPredefined(QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata, QString type, double depth, float deg_rot, QString personalized_type, crackDef_params *params)
{
    std::cout << "PARAMS: " << params->n_intermediate_points << std::endl;

    Defect *defecto;
    defecto = Defect::make_predefinedDefect(type,depth,deg_rot,personalized_type,polydata, params);
   // defecto->setReaderModel(reader_model);
    defecto->setPolydataModel(this->polydata_model);
    defecto->constructLatticeWithSelection(polydata);
    std::cout << "Construyendo defecto..." << std::endl;
    defecto->fillLattice();
    qDebug("Construído. Ahora haciendo otras cosas");

    QString name_file_aux = file_model;
    name_file_aux.replace(".stl","");
    QString name_saving = file_saving;
    name_saving.replace(".stl","");

    int subd = defecto->subdivideVTK(name_file_aux.toStdString());

    clock_t tStart2_ = clock();

    if (subd == defecto->NO_PROBLEMATIC_AREAS)
    {
        std::cout << "NO PROBLEMATIC AREAS" << std::endl;
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else if (subd == defecto->PROBLEMATIC_AREAS)
    {
        std::cout << "PROBLEMATIC AREAS" << std::endl;
        defecto->adaptLattice();
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else    {
        std::cout << "ERROR" << std::endl;
        defecto->destructLattice();
        delete defecto;
        emit defectInserted_error(file_model);
    }

    std::cout <<"Tiempo " << (double)(clock() - tStart2_)/CLOCKS_PER_SEC << std::endl;

}

void controller::insertDefectSelection(QString file_defect, QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata)
{
    Defect *defecto;
    defecto = Defect::make_defect(file_defect);
    //defecto->setReaderModel(reader_model);
    defecto->setPolydataModel(this->polydata_model);
    defecto->constructLatticeWithSelection(polydata); //Descomentar
    std::cout << "Construyendo defecto..." << std::endl;
    defecto->fillLattice(); //Descomentar
    QString name_file_aux = file_model;
    name_file_aux.replace(".stl","");
    QString name_saving = file_saving;
    name_saving.replace(".stl","");

    int subd = defecto->subdivideVTK(name_file_aux.toStdString());

    clock_t tStart2_ = clock();

    if (subd == defecto->NO_PROBLEMATIC_AREAS)
    {
        std::cout << "NO PROBLEMATIC AREAS" << std::endl;
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else if (subd == defecto->PROBLEMATIC_AREAS)
    {
        std::cout << "PROBLEMATIC AREAS" << std::endl;
        defecto->adaptLattice();
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else
    {
        std::cout << "ERROR" << std::endl;
        defecto->destructLattice();
        delete defecto;
        emit defectInserted_error(file_model);
    }

    std::cout <<"Tiempo " << (double)(clock() - tStart2_)/CLOCKS_PER_SEC << std::endl;

}


void controller::insertDefect(QString file_defect, QString file_model, QString file_saving)
{
    Defect *defecto;
    defecto = Defect::make_defect(file_defect);
    //defecto->setReaderModel(reader_model);
    defecto->setPolydataModel(this->polydata_model);
    defecto->constructLattice(); //Descomentar
    std::cout << "Construyendo defecto..." << std::endl;
    defecto->fillLattice(); //Descomentar
    QString name_file_aux = file_model;
    name_file_aux.replace(".stl","");
    QString name_saving = file_saving;
    name_saving.replace(".stl","");
    ///

    int subd = defecto->subdivideVTK(name_file_aux.toStdString());


    clock_t tStart2_ = clock();

    if (subd == defecto->NO_PROBLEMATIC_AREAS)
    {
        std::cout << "NO PROBLEMATIC AREAS" << std::endl;
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else if (subd == defecto->PROBLEMATIC_AREAS)
    {
        std::cout << "PROBLEMATIC AREAS" << std::endl;
      //  defecto->subdivideLattice();
      //  defecto->mergeLattice(name_file_aux.toStdString(), name_saving.toStdString());

        defecto->adaptLattice();
        defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString());
        defecto->destructLattice();
        delete defecto;
        emit defectInserted(file_saving);
    }
    else
    {
        std::cout << "ERROR" << std::endl;
        defecto->destructLattice();
        delete defecto;
        emit defectInserted_error(file_model);
    }

    std::cout <<"Tiempo " << (double)(clock() - tStart2_)/CLOCKS_PER_SEC << std::endl;

    ///


   // defecto->createDefectVTK(); //Eliminar
//    if (defecto->insertDefect(name_file_aux.toStdString(), name_saving.toStdString())) //InsertDefect
//    {
////
////

//        defecto->destructLattice();
//        delete defecto;
//        emit defectInserted(file_saving);
//    }
//    else
//    {
//        defecto->destructLattice();
//        delete defecto;
//        emit defectInserted_error(file_model);

//    }

}

void findLocalMaxMin(QVector<double> data, QVector<int> &id_max, QVector<int> &id_min, QVector<int> &ids_global)
{
    for (int i=1; i<data.size()-1; i++)
    {
        if (data[i]>data[i+1] && data[i]>data[i-1] && data[i]!=-1)
        {
            id_max.push_back(i);
            ids_global.push_back(i);
        }

        else if (data[i]<data[i+1] && data[i]<data[i-1] && data[i]!=-1)
        {
            id_min.push_back(i);
            ids_global.push_back(i);
        }
    }
}

bool isIntInQVector(QVector<int> v, int data, int &id_zona)
{
    for (int i=0; i<v.size(); i++)
    {
        if (v[i]==data)
        {
            id_zona = i;
            return true;
        }
    }
    return false;
}

struct structOrientation
{
    int id_pos_ideal;
    int id_pos_real;
    double angle;
};

bool sortOnIdPosIdeal(structOrientation s1, structOrientation s2)
{
    return s1.id_pos_ideal < s2.id_pos_ideal;
}

bool isPinDeterminedVolumen(double* p_, QVector3D min, QVector3D max)
{
    QVector3D p(p_[0],p_[1],p_[2]);

    if (p.x()>min.x() && p.x()<max.x())
    {
        if (p.z()>min.z() && p.z()<max.z())
        {
            if (p.y()>min.y() && p.y()<max.y())
                return true;

        }
    }

    return false;
}



void controller::scanAllPiece(int n_steps, KDNode tree, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, QString path)
{



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



void escribirEnArchivo(const QVector<double>& datos, const QString& nombreArchivo)
{
    QFile archivo(nombreArchivo);
    if (archivo.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&archivo);
        for (int i = 0; i < datos.size(); ++i)
        {
            out << datos[i] << '\n';
        }
        archivo.close();
    }
}

// Función para calcular la matriz de base B-spline cúbica
Eigen::Matrix4d ComputeBasisMatrix(double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    Eigen::Matrix4d basisMatrix;

    basisMatrix << -1.0/6 * t3 + 0.5 * t2 - 0.5 * t + 1.0/6, 2.0/3 * t3 - t2 + 0.5, -1.0/2 * t3 + 0.5 * t2 + 0.5 * t + 1.0/2, -1.0/6 * t3,
                     0.5 * t3 - t2 + 2.0/3, -t3 + 1.0, 0.5 * t3 - 2.0 * t2 + 2.0/3, 0.0,
                     -0.5 * t3 + 0.5 * t2 + 0.5 * t + 1.0/2, 2.0/3 * t3 - 2.0 * t2 + 0.5, -t3 + t2 + t + 1.0/6, 0.0,
                     1.0/6 * t3, 0.0, 0.0, 0.0;

    return basisMatrix;
}


// Función para interpolar una trayectoria 3D con curvas B-spline
QVector<QVector3D> InterpolateBSpline(const QVector<QVector3D>& controlPoints, int numPoints) {
    int n = controlPoints.size() - 1;
    QVector<QVector3D> interpolatedPath;

    for (int i = 0; i < n - 2; ++i) {
        for (int j = 0; j < numPoints; ++j) {
            double t = static_cast<double>(j) / (numPoints - 1);
            Eigen::Matrix4d basisMatrix = ComputeBasisMatrix(t);

            QVector3D interpolatedPoint;
            for (int k = 0; k < 4; ++k) {
                double aux = basisMatrix(0, k);
                interpolatedPoint +=  controlPoints[i + k] * aux;
            }

            interpolatedPath.push_back(interpolatedPoint);
        }
    }

    return interpolatedPath;
}


void controller::trajGeneratorPlugin(TriPluginInterface *plugin/*,QVector<QVector3Dd> pos_dataTraj,QVector<QVector3Dd> rpy_dataTraj,double vel,int frames, double fov,double resolution,double w_range, double w_distance,double uncertainty,KDNode tree, QString path*/)
{
    qInfo() << "HOLA";
//    double vel = 0.3;
//    double fov = 50;
//    double resolution = 100;
//    int frames = 500;
//    double w_range = 150;
//    double w_distance = 250;
//    double uncertainty = 20;

//    int n_iteraciones = 2;
//    plugin->setCustomFlag(true);
//    //QString path ="/home/sara/Descargas/PRUEBAS_DENSIDAD/traj_100/";
//    for(int i=0; i<n_iteraciones; i++)
//    {
//        //scan_finished = false;
//        if (i>0)
//          plugin->setCustomFlag(false);

//        qInfo() << path;
//        plugin->setPath(path);
//        plugin->precalculate();
//        plugin->calculate();
//        plugin->postcalculate();

//        //Hacer el escaneo
//        QVector<QVector3D> pos_sensor_i, rpy_sensor_i;
//        plugin->getTrajectory(pos_sensor_i, rpy_sensor_i);
//        QVector<trajectoryNode> nodes_load;
//        QVector<QVector3Dd> pos_dataTraj;
//        for(int id=0; id<pos_sensor_i.size();id++)
//        {

//            QQuaternion q = QQuaternion::fromEulerAngles(rpy_sensor_i[id].y(),rpy_sensor_i[id].z(),rpy_sensor_i[id].x());
//            QVector3Dd posxyz_ = QVector3Dd(pos_sensor_i[id].x(), pos_sensor_i[id].y(), pos_sensor_i[id].z());
//            pos_dataTraj.push_back(posxyz_);

//            trajectoryNode node_aux(posxyz_,q);
//            nodes_load.push_back(node_aux);
//        }

//        path = path + "new_traj/";

//        //emit
//        getTrajectoryNodes(nodes_load, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, path, false);
//    }


}



void controller::trajectoryGeneratorRL()
{
    std::cout << "Generating Traj" << std::endl;

}


void controller::trajectoryGenerator4(GenTraj_options opt, QVector<trajectoryNode> nodes, QVector3D normal_plane_, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path)
{

        int frames_i=0;
        int frames_totales = frames*nodes.size();

       // getTrajectoryNodes(nodes,vel,frames,FOV,resolution,w_range,w_distance,uncertainty,tree,path,false);

        QVector<QVector3Dd> pos_sensor;
        QVector<QVector3Dd> rpy_sensor;
        QVector<QVector3Dd> points;
        QVector<QVector3Dd> normals;
        QVector<QVector3Dd> measurements;
        int points_per_profile = 1000;
        double working_distance = 270;
        int id_string = 0;

    // Cargamos los datos--------------------------------------------------------------------------------
       // Trayectoria del sensor
       std::string path_global = "/home/sara/Descargas/PRUEBAS_DENSIDAD/"; //portaRotu
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

               QVector3Dd posxyz_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
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

               QVector3Dd rpy_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
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
           QVector3Dd points_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
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
           QVector3Dd normals_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
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
           QVector3Dd measurements_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
           measurements.push_back(measurements_);

       }

       std::cout << "Datos cargados" << std::endl;


   // Calculamos densidades sobre la medida---------------------------------------------------------------

       std::cout << "Frames: " << points.size()/points_per_profile << std::endl;





}


void controller::trajectoryGenerator(GenTraj_options opt, QVector<trajectoryNode> nodes, QVector3D normal_plane_, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path)
{
    if (nodes.size()<2)
    {
        //EMIT ui->listWidget->addItem("PUNTOS INSUFICIENTES");
    }

    else
    {
        double working_distance = w_distance;
        switch(opt)
        {
            case optMinRange:
                working_distance = w_distance-w_range/2;
            break;
            case optWDist:
                working_distance = w_distance;
            break;
            case optMaxRange:
                working_distance = w_distance+w_range/2;
            break;
        }
        int frames_i=0;
        int frames_totales = frames*nodes.size();


        vtkSmartPointer<vtkPolyDataNormals> normalGenerator_ = vtkSmartPointer<vtkPolyDataNormals>::New();
        normalGenerator_->SetInputData(this->polydata_model);
        normalGenerator_->ComputePointNormalsOn();
        normalGenerator_->ComputeCellNormalsOff();
        normalGenerator_->Update();
        auto normals_ = normalGenerator_->GetOutput()->GetPointData()->GetNormals();


        //Número de nodos --> Diferentes trayectorias
        for (int id_node=0; id_node<nodes.size()-1; id_node=id_node+2)
        {
            QVector3Dd pos_ini;
            QVector3Dd pos_end;

            pos_ini = nodes[id_node].pos();
            pos_end = nodes[id_node+1].pos();

            double cte_x= (pos_end.x() - pos_ini.x()) / frames;
            double cte_y= (pos_end.y() - pos_ini.y()) / frames;
            double cte_z= (pos_end.z() - pos_ini.z()) / frames;

            //Encontrar dirección escaneo
            QVector3Dd scan_direction(cte_x,cte_y,cte_z);
            scan_direction = scan_direction.normalized();

            QVector<QVector3Dd> new_sensor_position;
            QVector<QVector3Dd> new_sensor_position2;
            QVector<QVector3Dd> new_sensor_orientation;

            QVector<QVector3Dd> old_orientation;
            QVector<QVector3Dd> normals;

            QVector<QVector<QVector3Dd>> total_data_sensor;
            QVector<QVector<QVector3Dd>> total_data_points_real;

            std::cout << "FRAMES: "<<frames << std::endl;
            // Frames para cada trayectoria

            for (int id_frame=0; id_frame<frames; id_frame++)
            {
                double pos_x = pos_ini.x() + id_frame*cte_x;
                double pos_y = pos_ini.y() + id_frame*cte_y;
                double pos_z = pos_ini.z() + id_frame*cte_z;

                sensor new_sensor_model = sensor(QVector3Dd(pos_x,pos_y,pos_z),0,90,90,w_range, w_distance+500,resolution,FOV*PI/180, uncertainty);
                new_sensor_model.getMeasurement(tree,false);

                total_data_sensor.push_back(new_sensor_model.sensor_data);
                total_data_points_real.push_back(new_sensor_model.sensor_data_points_real);

                //Actualizar altura para que coincida con working_distance
                int n_sum=0;
                double sum=0;
                for (int i=0; i<resolution; i++) //Rayos
                {
                    if (new_sensor_model.sensor_data[i].z() != 0) //Si no hay objeto no lo tenemos en cuenta
                    {
                        sum = sum+new_sensor_model.sensor_data[i].z();
                        n_sum++;
                    }
                }
                double mean_z=-1;
                if (sum != 0)
                    mean_z = sum/n_sum;

                double desf_wd = working_distance-mean_z;

                QVector3Dd normal_plane(normal_plane_.x(), normal_plane_.y(), normal_plane_.z());

                new_sensor_position.push_back(QVector3Dd(pos_x,pos_y,pos_z)+normal_plane*desf_wd);
                old_orientation.push_back(QVector3Dd(0,90,90));

                emit frameDone(frames_i, frames_totales,new_sensor_model);
                frames_i++;


            }
            //

            //Volver a recorrer, calculando normales y ángulos
            int n_frames_aux=6;
            QVector<double> angles;

            for(int id_frame=0; id_frame<frames;id_frame++)
            {
                //Con los puntos seleccionar area con vtk y calcular normales?
                int id_min=id_frame-n_frames_aux/2; if(id_min<0)id_min=0;
                int id_max=id_frame+n_frames_aux/2; if(id_max>frames-1)id_max=frames-1;

                double max_x=0,min_x=100000,max_y=0,min_y=100000,max_z=0,min_z=10000;
                double px = 0, py = 0, pz = 0; int total=0;

                for(int i=0;i<resolution;i++)
                {
                    if (total_data_points_real[id_frame][i].x() != 0 && total_data_points_real[id_min][i].y() != 0 && total_data_points_real[id_min][i].z() != 0)
                    {
                        px=px+total_data_points_real[id_frame][i].x();
                        py=py+total_data_points_real[id_frame][i].y();
                        pz=pz+total_data_points_real[id_frame][i].z();
                        total++;
                    }
                    if(total_data_points_real[id_min][i].x() < min_x && total_data_points_real[id_min][i].x() != 0)
                        min_x = total_data_points_real[id_min][i].x();
                    if(total_data_points_real[id_max][i].x() > max_x && total_data_points_real[id_min][i].x() != 0)
                        max_x = total_data_points_real[id_max][i].x();

                    if(total_data_points_real[id_min][i].y() < min_y && total_data_points_real[id_min][i].y() != 0)
                        min_y = total_data_points_real[id_min][i].y();
                    if(total_data_points_real[id_max][i].y() > max_y && total_data_points_real[id_min][i].y() != 0)
                        max_y = total_data_points_real[id_max][i].y();

                    if(total_data_points_real[id_min][i].z() < min_z && total_data_points_real[id_min][i].z() != 0)
                        min_z = total_data_points_real[id_min][i].z();
                    if(total_data_points_real[id_max][i].z() > max_z && total_data_points_real[id_min][i].z() != 0)
                        max_z = total_data_points_real[id_max][i].z();

                }

//                QVector3D min(min_x,min_y-100,min_z);
//                QVector3D max(max_x,max_y+100,max_z);

                if (total==0)
                {
                    total=1;
                }
                px=px/total;
                py=py/total;
                pz=pz/total;

                //ESTO ACTUALIZAR SEGUN NORMAL Y DIRECCION ESCANEO Y DEMÁS
                QVector3D min(px-15,py-1000,pz-1000);//tamaño sensor
                QVector3D max(px+15,py+1000,pz+1000);

//                std::cout << "MIN: ";
//                printQVector3D(min);
//                std::cout << "MAX: ";
//                printQVector3D(max);

                vtkNew<vtkIdTypeArray> ids;
                //for(int i=0;i<reader->GetOutput()->GetNumberOfPoints();i++)
                for(int i=0;i<this->polydata_model->GetNumberOfPoints();i++)
                {
                    //auto point = reader->GetOutput()->GetPoint(i);
                    auto point = this->polydata_model->GetPoint(i);
                    if(isPinDeterminedVolumen(point,min,max))
                    {

                        double* values;
                        values = normals_->GetTuple3(i);

                        if (values[0]==0 && values[1]==0 && values[2]==0)
                          continue;

                        if (values[0]>-0.001 && values[0]<0.001 && values[1]>-0.001 && values[1]<0.001 )
                          continue;

                   //     std::cout << "NORMALS: " << values[0] << ", " << values[1] << ", " << values[2] << std::endl;

                        //ACTUALIZAR SEGUN NORMAL------
                        if(values[1]<0)
                            continue;
                        if(values[1]>-0.02 && values[1]<0.02)
                            continue;
                        //----------------------------
                        ids->InsertNextValue(i);
                    }
                }


                //Recorremos y calculamos media de normales
                //Almacenamos en vector
                vtkSmartPointer<vtkPolyData> selected_polyData =  vtkSmartPointer<vtkPolyData>::New();
                vtkSmartPointer<vtkPolyData> notSelected_polyData =  vtkSmartPointer<vtkPolyData>::New();

                double n[3];
               // if(vtkTools::extractSelectionPolyData(reader->GetOutputPort(),ids,selected_polyData,notSelected_polyData))
                if(vtkTools::extractSelectionPolyData(this->polydata_model,ids,selected_polyData,notSelected_polyData))
                {
                    vtkTools::getNormalPolyData(selected_polyData,n);
                    normals.push_back(QVector3Dd(n[0],n[1],n[2]));
                 //   std::cout << "ENTR0: " << std::endl;
                 //    printQVector3D(QVector3D(n[0],n[1],n[2]));
                }
                else
                    normals.push_back(QVector3Dd(normal_plane_.x(),normal_plane_.y(),normal_plane_.z()));


                std::cout << "FRAME: " << id_frame << ": " ;
                printQVector3D(normals[id_frame].toQVector3D());
             //   std::cout << "POS: " << id_frame << ": " ;
             //   printQVector3D(QVector3D(px,py,pz));

                ///////////////////////////////////////////
                double angle_i = fabs(std::acos(QVector3D::dotProduct(normals[id_frame].toQVector3D(),normal_plane_)/(normals[id_frame].toQVector3D().length()*normal_plane_.length())))*180/PI;;
                angle_i=round(angle_i);
                double dl=working_distance*std::sin(angle_i*PI/180);
                double dh=working_distance*(1-std::cos(angle_i*PI/180));

                dh=-dh;

                if(id_frame>3 &&   QVector3D::dotProduct(new_sensor_position[id_frame].toQVector3D(),normal_plane_) > QVector3D::dotProduct(new_sensor_position[id_frame-3].toQVector3D(),normal_plane_)) //Subiendo
               {
                   angle_i = -angle_i;
                   dl=-dl;
                 //  dh=-dh;

               }

                new_sensor_position2.push_back(new_sensor_position[id_frame]+QVector3Dd(dl,dh,0)); //DEPENDE DE NORMAL Y DIR_ESCANEO
                new_sensor_orientation.push_back(QVector3Dd(0,90+angle_i,90));

//                std::cout << "angle: " << angle_i << std::endl;
//                std::cout << "dl: " << dl << std::endl;
//                std::cout << "dh: " << dh << std::endl;
//                std::cout << "normal: ";
//                printQVector3D(normals[id_frame].toQVector3D());
//                std::cout << "pos: ";
//                printQVector3D(new_sensor_position[id_frame].toQVector3D());
//                std::cout << "pos nueva: ";
//                printQVector3D(new_sensor_position2[id_frame].toQVector3D());
//                std::cout << std::endl;
                // lo desplazo dl y dh, con eso me tiene que quedar en el punto que quiero

            }



///Buscando zonas problemáticas
///
            QVector<int> id_problematic;
            for(int i=1;i<new_sensor_position2.size();i++)
            {
                if( !(QVector3Dd::dotProduct(new_sensor_position2[i],scan_direction)>QVector3Dd::dotProduct(new_sensor_position2[i-1],scan_direction)))
                {
                    id_problematic.push_back(i); //2 de zona problematica

                    int j=i+2;
                    while( !(QVector3Dd::dotProduct(new_sensor_position2[j],scan_direction)>QVector3Dd::dotProduct(new_sensor_position2[j-1],scan_direction)))
                    {
                      j++;
                    }
                    id_problematic.push_back(j); //1 de zona problematica

                    i=j+1;


                }
            }

            std::cout << "PROBLEMATICAS: " << id_problematic.size() << std::endl;
            for(int i=0;i<id_problematic.size();i=i+2)
            {

                  int id1 = id_problematic[i+1];
                  int id2 = id_problematic[i];

                  double x1 = QVector3Dd::dotProduct(new_sensor_position2[id1],scan_direction);
                  double x2 = QVector3Dd::dotProduct(new_sensor_position2[id2],scan_direction);

                  int id_ini, id_end;

                  for(int id_frame=0; id_frame<frames; id_frame++)
                  {
                      if(QVector3Dd::dotProduct(new_sensor_position2[id_frame],scan_direction)>x1)
                      {
                          id_ini = id_frame;
                          break; //¿Sustituir por bucle while?
                      }
                  }
                  for(int id_frame=id2;id_frame<frames;id_frame++)
                  {
                      if(QVector3Dd::dotProduct(new_sensor_position2[id_frame],scan_direction)>x2)
                      {
                          id_end = id_frame;
                          break; //¿Sustituir por bucle while?
                      }
                  }


                QVector3Dd pos_ini = new_sensor_position2[id_ini];
                QVector3Dd pos_end = new_sensor_position2[id_end];

                std::cout << "Pos ini: "; printQVector3D(pos_ini.toQVector3D());
                std::cout << "Pos end: "; printQVector3D(pos_end.toQVector3D());

                QVector3Dd ori_ini = new_sensor_orientation[id_ini];
                QVector3Dd ori_end = new_sensor_orientation[id_end];

                int step_frames = id_end-id_ini;

                QVector3Dd step_pos = (pos_end-pos_ini)/step_frames;
                QVector3Dd step_ori = (ori_end-ori_ini)/step_frames;


                for(int j=0; j<step_frames;j++)
                {
                    QVector3Dd pos_i = pos_ini + step_pos*j;
                    QVector3Dd ori_i = ori_ini + step_ori*j;

                    //Nuevo ángulo -> nuevo dl -> nuevo dh -> Actualizar pos_i
                    double new_angle = fabs(QVector3D::dotProduct(normal_plane_, ori_i.toQVector3D())-90);
                    double new_dl=working_distance*std::sin(new_angle*PI/180);
                    double new_dh=-working_distance*(1-std::cos(new_angle*PI/180));


                    //DL Y DH, RECALCULAR!!!!

                    std::cout << "Angle: " << new_angle << std::endl;
                    std::cout << "dl: " << new_dl << std::endl;
                    std::cout << "dh: " << new_dh << std::endl;

                    ///Descomentar las dos líneas siguientes:
                    new_sensor_position2[id_ini+j] = pos_i ;//+ QVector3Dd(new_dl,new_dh,0); //DEPENDE DE NORMAL Y DIR_ESCANEO
                    new_sensor_orientation[id_ini+j] = ori_i;

                    printQVector3D(pos_i.toQVector3D()); //2,1 - 2,1

                }

//                std::cout << "Problematic 2: " << id_problematic[i] << " : "; printQVector3D(pos_end.toQVector3D()); //2,1 - 2,1


            }

////////////////////////////////// fin búsqueda zonas problemáticas


            /// Una vez que tengo los ángulos y posiciones óptimas, ver que el ángulo y las posiciones no pueden variar más que cierto valor
            /// En función de las limitaciones del sistema de movimiento
            double ang_lim = 2;


            /// Si un punto se separa mucho del anterior juntarlo
            for(int i=4; i<new_sensor_position2.size();i++)
            {
//                //NO VA BIEN
//                QVector3Dd ref=new_sensor_position2[i];
//                QVector3Dd pi=new_sensor_position2[i+1];
//                if(abs(ref.x()-pi.x()) > cte_x)
//                {
//                    std::cout << "entro x" << std::endl;

//                    new_sensor_position2[i].setX(ref.x()+cte_x);
//                }
//                if(abs(ref.y()-pi.y()) > cte_x)
//                {
//                    std::cout << "entro y" << std::endl;
//                    new_sensor_position2[i].setY(ref.y());
//                }
            }

            ///

            std::stringstream name_traj;
            name_traj << "/traj_sensor" << id_node<< ".xml";
            saveTraj(path, QString::fromStdString(name_traj.str()),new_sensor_position2, new_sensor_orientation,frames,  vel, FOV, resolution, uncertainty); //new_rpy_sensor_orientation

            std::cout << "LISTO" << std::endl;

        }

    }
}
/*
void controller::trajectoryGenerator2(GenTraj_options opt, QVector<trajectoryNode> nodes, QVector3D normal_plane_, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path)
{
    if (nodes.size()<2)
    {
        //EMIT ui->listWidget->addItem("PUNTOS INSUFICIENTES");
    }

    else
    {
        double working_distance = w_distance;
        switch(opt)
        {
            case optMinRange:
                working_distance = w_distance-w_range/2;
            break;
            case optWDist:
                working_distance = w_distance;
            break;
            case optMaxRange:
                working_distance = w_distance+w_range/2;
            break;
        }


        QVector3Dd normal_plane(normal_plane_.x(), normal_plane_.y(), normal_plane_.z());
        int frames_i=0;
        QVector<int> vFrames;
        int frames_totales = 0;
        for (int j=0; j<nodes.size()-1; j=j+2)
        {
            QVector3Dd vDist = nodes[j+1].pos() - nodes[j].pos();
            double dist = vDist.length()/1000; //m
            int frames_aux = dist/vel *frames;

            vFrames.push_back(frames_aux);
            frames_totales = frames_totales + frames_aux; //NO VA BIEEEEEN
        }
        clock_t tStart = clock();

        int n_frames_change = 20; //número de frames entre actualizaciones


        //Número de nodos --> Diferentes trayectorias
        for (int j=0; j<nodes.size()-1; j=j+2)
        {

            int j_frames = j/2;
            //Definición variables
            QVector3Dd pos_ini;
            QVector3Dd pos_end;
            QQuaternion q_ini;
            QQuaternion q_end;

            pos_ini = nodes[j].pos();
            pos_end = nodes[j+1].pos();
            q_ini = nodes[j].q();
            q_end = nodes[j+1].q();


            //double angle_limit = 10;

            double cte_x= (pos_end.x() - pos_ini.x()) / vFrames[j_frames] ;
            double cte_y= (pos_end.y() - pos_ini.y()) / vFrames[j_frames] ;
            double cte_z= (pos_end.z() - pos_ini.z()) / vFrames[j_frames] ;

            //Encontrar dirección escaneo
            QVector3Dd scan_direction(cte_x,cte_y,cte_z);
            scan_direction = scan_direction.normalized();

            QVector<QVector<double>> distances_sensor(vFrames[j_frames]);
            QVector<QVector<QVector3Dd>> total_data_sensor(vFrames[j_frames]);
            QVector<QVector<QVector3Dd>> total_data_sensor_error(vFrames[j_frames]);
            QVector<QVector<QVector3Dd>> total_points_real(vFrames[j_frames]);
            QVector<QVector<QVector3Dd>> total_points_real_error(vFrames[j_frames]);
            QVector<QVector<QVector3Dd>> normal_data_sensor(vFrames[j_frames]);

            QVector<QVector3Dd> pos_sensor;
            QVector<QVector3Dd> rpy_sensor;

            clock_t tStart_ = clock();

            //--
            double pos_vertical, pos_horizontal, pos_angle, pos_angle_max;
            QVector<double> pos_sensor_vert;
            QVector<double> pos_sensor_hor;
            std::vector<double> pos_sensor_z;

            QVector<double> pos_sensor_angle;
            QVector<double> pos_sensor_angle_max; //hacia atrás

            //--
            QVector3Dd old_orientation, new_orientation;
            QVector3Dd old_position, new_position;
            QVector<QVector3Dd> new_rpy_sensor_orientation;
            QVector<QVector3Dd> old_rpy_sensor_orientation;

            QVector<double> means_sensorData;
            //--


            // Frames para cada trayectoria
            for (int i=0; i<vFrames[j_frames]; i++)
            {
                //Cálculo de una trayectoria recta entre posición inicial y final
                QVector3Dd euler_i;
                QQuaternion q_i;

                double pos_x = pos_ini.x() + i*cte_x;
                double pos_y = pos_ini.y() + i*cte_y;
                double pos_z = pos_ini.z() + i*cte_z;

                double roll, pitch, yaw;
                if (q_ini != q_end)
                {
                    double t = double(i)/double(vFrames[j_frames]);

                    q_i = QQuaternion::slerp(q_ini, q_end, t);
                    euler_i = ToEulerAngles(q_i);
                    roll = euler_i.z();
                    pitch = euler_i.x();
                    yaw = -euler_i.y();
                }
                else
                {
                    euler_i = ToEulerAngles(q_ini);
                    roll = euler_i.z();
                    pitch = euler_i.x();
                    yaw = euler_i.y();
                }

                old_orientation = QVector3Dd(roll,pitch,yaw);
                old_position = QVector3Dd(pos_x,pos_y,pos_z);

                sensor new_sensor_model = sensor(QVector3Dd(pos_x,pos_y,pos_z),roll,pitch,yaw,w_range, w_distance,resolution,FOV*PI/180, uncertainty);
                new_sensor_model.getMeasurement(tree,false);

                distances_sensor[i] = (new_sensor_model.distances_data);
                total_data_sensor[i] = (new_sensor_model.sensor_data);
                total_points_real[i] = (new_sensor_model.sensor_data_points_real);
                total_data_sensor_error[i] = (new_sensor_model.sensor_data_error);
                total_points_real_error[i] =  (new_sensor_model.sensor_data_points_real_error);
                normal_data_sensor[i] = (new_sensor_model.normal_data);


                //------------------------------------------------------------------------

                //Desfases en cada eje y en el ángulo
                double desf_width=0, desf_height=0, desf_length=0, angle_max=0;//length->dirección movimiento sensor
                double angle;


                //Actualizar posición vertical y orientación (pos_sensor, rpy_sensor) según los últimos n_frames_change frames
                QVector<QVector3D> points; //puntos para ajustar el plano

                if (i%n_frames_change == 0 && i!=0)
                {

                    QVector<double> means;
                    double sum=0;
                    QVector3Dd sum_normal(0,0,0);
                    int n_sum=0;
                    for (int n_it=n_frames_change; n_it>0 ; n_it--) //Frames
                    {
                        for (int ii=0; ii<resolution; ii++) //Rayos
                        {
                            if (int(total_data_sensor[i-n_it][ii].z()) != 0) //Si no hay objeto no lo tenemos en cuenta
                            {
                                sum = sum+total_data_sensor[i-n_it][ii].z();
                                n_sum++;
                                points.push_back(QVector3D(total_points_real[i-n_it][ii].x(),total_points_real[i-n_it][ii].y(),total_points_real[i-n_it][ii].z()));

                            }

                        }
                        if (sum != 0)
                        {
                            means.push_back(sum/n_sum);
                            means_sensorData.push_back(sum/n_sum);
                        }
                        else
                        {
                            means_sensorData.push_back(-1);
                        }
                    }

                    double mean = sum/n_sum;

                    //Ajustar un plano a points y buscar su normal
                    // guardar id de la posición en la que debería de estar para que en ese punto el ángulo sea el que busco
                    //-----------------------------------------------
                    if (points.size()>n_frames_change)
                    {
                        FitPlane3D plane(points);
                        plane.build();
                        QVector3D normal = plane.getNormal();
                        QVector3D normal_abs(fabs(normal.x()), fabs(normal.y()), fabs(normal.z()));

                       // double x_ = -atan2f(normal_abs.x() - normal_plane_.x(), normal_abs.y() - normal_plane_.y()) / PI * 180.0f + 180.0f;
                       // double y_ = asinf((normal_abs.z() - normal_plane_.z()) / (normal_plane_.length()*normal_abs.length())) * 180.0f / PI;
                       // QVector3Dd angle_3d(x_,y_,0);

                        angle = (std::acos(QVector3D::dotProduct(normal_abs,normal_plane_)/(normal_abs.length()*normal_plane_.length())))*180/PI;
                        //angle = 0;
                        //angle = angle_3d.y();

                        double desf_wd = working_distance-mean; //independientemente de si sube o baja

                        mean = working_distance;
                        double c = 2*mean*std::sin(angle/2*PI/180); //cuerda del arco
                        desf_length = mean*std::tan(angle*PI/180); //std::sin(angle*PI/180);
                        desf_height = sqrt(c*c - desf_length*desf_length);

                        if (means.size()>1 && means[0] > means[means.size()-1]) //Subiendo
                        {
                            new_orientation = old_orientation - QVector3Dd(angle*normal_plane.x(),angle*normal_plane.y(),angle*normal_plane.z());

                            angle = -angle;
                            desf_length = -desf_length;
                            desf_height = desf_wd; //----------------comentar desf_height
                            angle_max =angle; //------------------ comentar linea

                        }

                        else //Bajando
                        {
                            new_orientation = old_orientation + normal_plane*angle;

                            angle = +angle;
                            desf_length = +desf_length;
                            desf_height =  + desf_wd; //-----------------comentar desf_height
                            angle_max =angle; //------------------ comentar linea


                        }

                        pos_horizontal = desf_length;
                        pos_vertical = desf_height;
                        pos_angle = angle;
                        pos_angle_max = angle_max; //------------------- comentar linea

                        }


                    else
                        {
                            pos_horizontal = 0;
                            pos_vertical = 0;
                            new_orientation = old_orientation;
                            pos_angle = 0;
                            pos_angle_max = 0;
                        }


                    //-----------------------



                    for (int k=0; k<n_frames_change;k++)
                    {

                        pos_sensor_hor.push_back(pos_horizontal);
                        pos_sensor_vert.push_back(pos_vertical);
                        pos_sensor_angle.push_back(pos_angle);

                        new_rpy_sensor_orientation.push_back(new_orientation); //-------------------- comentar linea
                        old_rpy_sensor_orientation.push_back(old_orientation); //-------------------- comentar linea

                    }

                    if (i>0)
                        angle_max = std::atan((cte_z - pos_sensor_hor[i-1])/working_distance); //SUSTITUIR CTE_Z POR ALGO GENÉRICO

                    for (int k=0; k<n_frames_change;k++)
                    {
                        pos_sensor_angle_max.push_back(pos_angle_max);
                    }

                }

                pos_sensor.push_back(QVector3Dd(pos_x,pos_y,pos_z));
                pos_sensor_z.push_back(pos_z);

                rpy_sensor.push_back(QVector3Dd(roll,pitch,yaw));


                emit frameDone(frames_i, frames_totales,new_sensor_model);
                frames_i++;
            }





            //Completar
            int falta = pos_sensor.size()-pos_sensor_vert.size();
            for (int k=0; k<falta; k++)
            {
                pos_sensor_hor.push_back(pos_horizontal);
                pos_sensor_vert.push_back(pos_vertical);
                pos_sensor_angle.push_back(pos_angle);
                pos_sensor_angle_max.push_back(pos_angle_max);
                new_rpy_sensor_orientation.push_back(new_orientation); //------------------- comentar linea
                old_rpy_sensor_orientation.push_back(old_orientation); //-------------------- comentar linea

            }

            //----------------------------------------------------------------------
            //----------------------------------------------------------------------
            QVector<double> pos_sensor_hor_ind;
            QVector<int> id_cambio;

            for (int i=0; i<pos_sensor_hor.size(); i++)
            {
                //Convertir valores pos_sensor_hor a [0:pos_sensor.size]
                double valor = pos_sensor[i].z()+pos_sensor_hor[i];
                // A qué valor de pos_sensor se parece más? -> coger id
                int id;
                findMostSimilarValueQVector(pos_sensor_z,valor,id);
                pos_sensor_hor_ind.push_back(id);

                //if (i == 79)
                //    int a = 0;

                if (i>0 && id < pos_sensor_hor_ind[i-1])
                    id_cambio.push_back(i);
            }


            //minimo y maximo:
            //int id_min =pos_sensor_hor_ind[std::distance(pos_sensor_hor_ind.begin(), std::min_element(pos_sensor_hor_ind.begin(), pos_sensor_hor_ind.end()))];
            //int id_max =pos_sensor_hor_ind[std::distance(pos_sensor_hor_ind.begin(), std::max_element(pos_sensor_hor_ind.begin(), pos_sensor_hor_ind.end()))];

            QVector<int> id_nuevos;
            for (int i=0; i<id_cambio.size(); i++)
            {
                double last_id = pos_sensor_hor_ind[id_cambio[i]-1];
                int new_id=pos_sensor_hor_ind[id_cambio[i]];
                int j=id_cambio[i]+1;
                while(new_id<=last_id)
                {
                    new_id = pos_sensor_hor_ind[j];
                    j++;
                }
                j--;
                id_nuevos.push_back(j);
            }

            std::cout << id_cambio.size() << std::endl;

            QVector<QVector3Dd> nuevas_posiciones;
            for (int i=0; i<id_cambio.size(); i++)
            {

                std::cout << id_cambio[i] << std::endl;
                //Interpolar ángulos entre posiciones id_nuevo e id_cambio-1
                float ang_ini = new_rpy_sensor_orientation[pos_sensor_hor_ind[id_cambio[i]-1]].y();
                float ang_fin = new_rpy_sensor_orientation[pos_sensor_hor_ind[id_nuevos[i]]].y();
                int n_step = (id_nuevos[i]-(id_cambio[i]-1));
                float step = (ang_fin - ang_ini)/n_step;


                for(int j=0; j<n_step;j++)
                {
                    new_rpy_sensor_orientation[pos_sensor_hor_ind[id_cambio[i]-1] + j].setY(ang_ini + j*step);
                    pos_sensor_hor_ind[id_cambio[i] + j] = pos_sensor_hor_ind[id_cambio[i]-1];

                }
            }

            for (int i = 0; i < pos_sensor_hor.size(); i++)
            {
                nuevas_posiciones.push_back(pos_sensor[pos_sensor_hor_ind[i]]);
            }


            //int a= 0;


            //----------------------------------------------------------------------
            //----------------------------------------------------------------------


            //-----SAVE DATA-----
            std::stringstream name, name2, name3, name4, name_traj, name_traj_si;
            // time_t now = time(0);
            //path << "../resultados/" << std::ctime(&now);
            name << "/step" << j_frames << ".txt";
            name2 << "/step_real" << j_frames << ".txt";
            name3 << "/step_error" << j_frames << ".txt";
            name4 << "/step_real_error" << j_frames << ".txt";
            name_traj << "/traj_sensor" << j_frames << ".xml";

            saveData(path, QString::fromStdString(name.str()),total_data_sensor);
            saveData(path, QString::fromStdString(name2.str()),total_points_real);
            saveData(path, QString::fromStdString(name3.str()),total_data_sensor_error);
            saveData(path, QString::fromStdString(name4.str()),total_points_real_error);

            //Save custom trajectory
            QVector<QVector3Dd> pos_sensor_;
            QVector<QVector3Dd> rpy_sensor_;

   //         for (int k=0; k<pos_sensor.size(); k++) //DEPENDIENDO DEL PLANO Y EJE
   //              new_sensor_position.push_back(pos_sensor[k] + normal_plane*pos_sensor_vert[k] + scan_direction*pos_sensor_hor[k]);


            //HACER ALGO PARA CUANDO NO HAY OBJETO A INSPECCIONAR (TIPO INICIO Y FINAL)-->COMPLETAR LOS ÚLTIMOSSS

            QVector<QVector3Dd> new_sensor_position(pos_sensor.size());
            QVector<QVector3Dd> new_sensor_orientation(pos_sensor.size(), QVector3Dd(-1,-1,-1));


            //double first = true;
            //int id_pos_ideal;

            //POSICION
            //QVector<int> poses_ideales;
            //QVector<double>angles_finales(pos_sensor.size(),0);
            bool notComplete = true;


            for (int k=0; k<pos_sensor.size(); k++) //DEPENDIENDO DEL PLANO Y EJE
            {


                if (k!=0 && k%n_frames_change == 0)
                {
                    //interpolar entre pos_vertical y pos_sensor_vert[pos_sensor_vert.size()-1]
                    double pos_ini_ = pos_sensor_vert[k-n_frames_change];
                    double pos_end_ = pos_sensor_vert[k];
                    double cte_= (pos_end_ - pos_ini_) / n_frames_change;

                    QVector3Dd ang_ini_ = new_rpy_sensor_orientation[k-n_frames_change];
                    QVector3Dd ang_end_ = new_rpy_sensor_orientation[k];
                    QVector3Dd cte_ang_= (ang_end_ - ang_ini_) / n_frames_change;

                    for (int kk=0; kk<n_frames_change; kk++)
                    {
                        //Position vertical change
                        double pos_ = pos_ini_ + kk*cte_;
                        QVector3Dd ang_ = ang_ini_ + cte_ang_*kk;

                        new_sensor_position[k-n_frames_change+kk] = pos_sensor[k-n_frames_change+kk] + normal_plane*pos_ ;//+ scan_direction*pos_sensor_hor[k];
                        new_rpy_sensor_orientation[k-n_frames_change+kk] = ang_;//---------sobra


                        //if (new_sensor_position[k-n_frames_change+kk].y() > -300)
                            //int llll = 0;
                   }
                }

                //Rellenar los frames que quedan
                if ((pos_sensor.size()-k)<pos_sensor.size()%n_frames_change && notComplete)
                {
                    n_frames_change = pos_sensor.size()-k;
                    double pos_ini_ = pos_sensor_vert[k];
                    double pos_end_ = pos_sensor_vert[pos_sensor.size()-1];
                    double cte_= (pos_end_ - pos_ini_) / n_frames_change;
                    for (int kk=0; kk<n_frames_change; kk++)
                    {
                        double pos_ = pos_ini_ + kk*cte_;
                        new_sensor_position[k+kk] = pos_sensor[k+kk] + normal_plane*pos_ ;//+ scan_direction*pos_sensor_hor[k];
                    }
                    notComplete = false;
                }
            }


         //   saveTraj(path, QString::fromStdString(name_traj.str()),new_sensor_position, new_sensor_orientation);
            //saveTraj(path, QString::fromStdString(name_traj.str()),new_sensor_position, new_rpy_sensor_orientation); //new_rpy_sensor_orientation
            saveTraj(path, QString::fromStdString(name_traj.str()),new_sensor_position, new_rpy_sensor_orientation); //new_rpy_sensor_orientation


            QString name_traj_si_ = path + "/traj_sensor" + j + ".si";

            TrajectoryController trajController;
            trajController.set3DPoints(new_sensor_position);
            trajController.setRPY(old_rpy_sensor_orientation); //rpy_sensor
            trajController.SaveTrajectory(name_traj_si_);

           // saveData(name.str(), total_data_sensor);
            printf("Time taken node %d: %.2fs\n", j_frames, (double)(clock() - tStart_)/CLOCKS_PER_SEC);

        }
        printf("Total time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);


    }
}
*/
void controller::updateDragMode(bool checked, renderVTK renderer_vtk)
{
    while(checked)
    {
        if(renderer_vtk.getSignalDragActor())
        {
            double *pos, *rpy;
            renderer_vtk.getDragActor(pos,rpy);

            std::cout << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
            std::cout << "RPY: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;


            emit updateUi(pos, rpy);
            renderer_vtk.setSignalActor(false);
        }
    }
}
