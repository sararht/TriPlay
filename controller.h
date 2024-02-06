#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sensor.h"
#include "trajectorynode.h"
#include "include_defects/defect.h"
#include "trajectorycontroller.h"
#include "rendervtk.h"
#include "cv_perlin_noise/PerlinNoise.h"
#include <QtXml>

#include "plugins/TriPluginInterface/triplugininterface.h"

#include<opencv2/opencv.hpp>

#ifndef PI
#define PI 3.141592
#endif


enum GenTraj_options {optMinRange, optWDist,optMaxRange};

class controller : public QObject
{

    Q_OBJECT
public:
    explicit controller(QObject *parent = nullptr);
    void setCancelling(bool signal){cancelling=signal;}
    //bool updatePolydataModel(vtkSmartPointer<vtkSTLReader> poly);
    //bool updatePolydataModel(QString name_file);
    bool updatePolydataModel(vtkSmartPointer<vtkPolyData> poly);

   // void trajectoryGenerator4(GenTraj_options opt, QVector<trajectoryNode> nodes, QVector3D normal_plane_, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path)



private:
    bool cancelling = false;
    //vtkSmartPointer<vtkSTLReader> reader_model;
    vtkSmartPointer<vtkPolyData> polydata_model;

private slots:
    void trajectory(QVector3Dd pos_ini, QVector3Dd pos_end, QQuaternion q_ini, QQuaternion q_end, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree);
    void getTrajectoryNodes(QVector<trajectoryNode> nodes, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, KDNode tree, QString path, bool onlyEven);
    void getTrajectoryNodesFromFile(QVector<QVector3Dd>,QVector<QVector3Dd>,double,double,double, double,double,KDNode, QString);
    void insertDefect(QString file_defect, QString file_model,  QString file_saving);
    void insertDefectSelection(QString file_defect, QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata);
    void insertDefectSelectionPredefined(QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata, QString type, double depth, float deg_rot, QString personalized_type, crackDef_params* params);
    void trajectoryGenerator(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString);
    //void trajectoryGenerator2(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString);
    void trajectoryGenerator4(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString);

    void trajGeneratorPlugin(TriPluginInterface *plugin);

    void scanAllPiece(int n_steps, KDNode tree, double vel, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty, QString path);
    void updateDragMode(bool checked, renderVTK renderer_vtk);

signals:
    void frameDone(int frame, int n_frames, sensor new_sensor);
    void frameDoneTraj(QVector<trajectoryNode> nodes, QVector<int> vFrames, int id_node, int frame, sensor new_sensor);
    void defectInserted(QString file_saving);
    void defectInserted_error(QString file_saving);
    void updateUi(double* pos, double* rpy);


    void endTrajFrom();

};

#endif // CONTROLLER_H
