#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
//#include <QtDataVisualization/Q3DScatter>
//#include <QtDataVisualization/QScatterDataProxy>
#include <QWidget>
#include "render.h"
#include "sensor.h"
#include <math.h>
#include <QFileDialog>
#include "stl_reader.h"
#include <iostream>

//#include <CL/cl.hpp>
#include "qcustomplot.h"
#include "basicplot.h"
#include <time.h>

#include <fstream>
#include "KD_tree_cpp/kdnode.h"
#include "controller.h"

#include <QThread>
#include <QString>

#include <omp.h>
#include "trajectorynode.h"
#include <QJSEngine>
#include <QtXml>

#include <include_defects/defect.h>
#include <trajectorycontroller.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <rendervtk.h>
//#include <vtkSTLReader.h>
#include <vtkModifiedBSPTree.h>
#include <vtkOBBTree.h>

#include <vtktools.h>

#include "defectselection.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE





class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    myRenderer renderer;
    renderVTK renderer_vtk;
    sensor sensor_model;

    basicPlot plot;

    //enum GenTraj_options {optMinRange, optWDist,optMaxRange};


private:
    Ui::MainWindow *ui;

    void initSpinBoxes();
    void updateSpinBoxes();

    vtkSmartPointer<vtkModifiedBSPTree> obbTree;

    //STL Model
    KDNode tree;
    QVector<QVector3Dd> v0;
    QVector<QVector3Dd> v1;
    QVector<QVector3Dd> v2;
    QVector<QVector3Dd> normals_tri;

    std::vector<Triangle*> trisModel;
    QString name_file;

    //Sensor
    controller controller_;
    QVector<trajectoryNode> nodes;

    //Interface
    QMovie *movie;
    QWidget *wdg_defect;
    QWidget *wSensorParams;
    QTableWidget *wdg_guideVisualization;
    QTabWidget *wdg_helpID;
    defectSelection *wdg_defect_selection;


    QWidget *wTrajGen;
    QRadioButton *b_planeXY;
    QRadioButton *b_planeXZ;
    QRadioButton *b_planeYZ;
    QRadioButton *b_axisX;
    QRadioButton *b_axisY;
    QRadioButton *b_axisZ;
    QRadioButton *b_opt1;
    QRadioButton *b_opt2;
    QRadioButton *b_opt3;

    QWidget *wScanAll;
    QWidget *wStepsNumber;
    QWidget *wPlaneSelection;
    QComboBox *pageComboBox;
    QSpinBox *steps_sb;

    QLabel *wdg_bolloExample;
    QLabel *wdg_grietaExample;
    QLabel *wdg_picoExample;
    QLabel *wdg_personalizedExample;

  //  vtkSmartPointer<vtkSTLReader> reader;
    vtkSmartPointer<vtkPolyData> polydata_model;
    bool visibilityListPoints = false;

    bool dragModeOn = false;


signals:

    void button_traj_clicked(QVector3Dd pos_ini, QVector3Dd pos_end, QQuaternion q_ini, QQuaternion q_end, double frames, double FOV, double resolution, double w_range, double w_distance, double uncertainty,KDNode tree);
    void button_traj_node_clicked(QVector<trajectoryNode>,double,double,double,double, double,double,double,KDNode, QString, bool);
    void button_traj_generator_clicked(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString);
    void button_traj_node_from_clicked(QVector<QVector3Dd>,QVector<QVector3Dd>,double,double,double,double,double,KDNode, QString);
    void button_defect_clicked(QString file_defect, QString file_model, QString file_saving);
    void button_defect_selection_clicked(QString file_defect, QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata);
    void button_defect_selection_predefined_clicked(QString file_model, QString file_saving, vtkSmartPointer<vtkPolyData> polydata, QString type, double depth,float deg_rot, QString personalized_type, crackDef_params* params);
    void button_scan_all_clicked(int,KDNode,double,double,double,double,double,double,double,QString);
    void button_dragMode_clicked(bool, renderVTK);

private slots:

    void on_button_refresh_clicked(bool change);
    void valuesPositionChanges();
    void valuesCaracteristicsChanges();

    void updateRender(int frame,int n_frames, sensor new_sensor);
    void updateRenderAfterDefect(QString file_saving);
    void errorAfterDefect(QString file_saving);
    void frameDoneTrajUpdate(QVector<trajectoryNode> nodes, QVector<int> vFrames, int id_node, int frame, sensor new_sensor);


    void clearTrajRender();

    void on_actionOpen_triggered();
    void on_actionSave_As_triggered();
    void on_actionSensor_Parameters_triggered();
    void on_actionInsert_Defect_triggered();
    void on_actionGet_Profile_triggered();
    void on_actionGet_Trajectory_triggered();
    void on_actionSet_Point_triggered();
    void on_actionClear_Points_triggered();
    void on_actiontr_Exit_triggered();

    void wdgSensorParams();
    void wdgVisualizationGuide();
    void wdgDefectGuide();
    void wdgTrajectoryGeneratorParams();
    void wdgScanAllParams();
    void wdgDefectSelection();
    void waitForDefectInsertion();


    void on_actionDefault_triggered();
    void on_actionBlack_triggered();
    void on_actionGray_triggered();
    void on_actionWhite_triggered();
    void on_actionPointsWindow_triggered();
    void on_actionVisualization_Guide_triggered();
    void on_actionInsert_Defect_Guide_triggered();
    void on_actionGet_Trajectory_from_triggered();
    void on_actionTrajectory_Generator_triggered();

    void on_actionReset_View_triggered();
    void on_actionInsert_Defect_Selection_triggered();
    void on_actionSetViewX_triggered();
    void on_actionSetViewX_2_triggered();
    void on_actionSetViewY_triggered();
    void on_actionSetViewY_2_triggered();
    void on_actionSetViewZ_triggered();
    void on_actionSetViewZ_2_triggered();
    void on_actionSetWireframeView_triggered();
    void on_actionSetSurfaceView_triggered();
    void on_actionSetPointsView_triggered();
    void on_actionselectCells_triggered();
    void on_actionFrom_File_triggered();
    void on_actionPredefined_defect_triggered();
    void on_button_stop_clicked();
    void on_actionSetDragMode_triggered(bool checked);
    void on_actionScan_all_piece_triggered();
    void on_actionSmooth_stl_triggered();

    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);


    void updateUi_drag(double*,double*);
};
#endif // MAINWINDOW_H
