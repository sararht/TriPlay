#ifndef RENDERVTK_H
#define RENDERVTK_H

//vtk includes
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkNamedColors.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkSTLReader.h>
#include <vtkAssembly.h>

//classes includes
#include "sensor.h"
#include "trajectorynode.h"
#include "mouseinteractorstyle.h"
#include "draginteractorstyle.h"


//#include <QMainWindow>
//#include <QWidget>


class renderVTK
{
public:
    renderVTK(){};
    renderVTK(sensor sensor_model, vtkSmartPointer<vtkPolyData> poly);

    QVTKOpenGLNativeWidget *widget;

    void updateRendered(sensor sensor_model, bool isInTraj = false);
   // void changeModel(QString name_file, sensor sensor_model);
   // void changeModel(vtkSmartPointer<vtkSTLReader> reader);
    void changeModel(vtkSmartPointer<vtkPolyData> poly);


    void addNewPoint(QVector<trajectoryNode> node);
    void deletePoints();
    bool getPolySelection(vtkSmartPointer<vtkPolyData> &selection);
    bool getDragActor(double *&pos, double *&rpy);
    bool getSignalDragActor();
    void setSignalActor(bool signal);

    void drawTraj(QVector<QVector3Dd>pos);
    void deleteTraj();



    //View tools
    void resetView();
    void changeViewAxesXY();
    void changeViewAxesXZ();
    void changeViewAxesYZ();
    void changeViewAxesXY_();
    void changeViewAxesXZ_();
    void changeViewAxesYZ_();
    void setWireframeView();
    void setSurfaceView();
    void setPointsView();
    void selectCellsOn();
    void changeStyleMode(bool isDragMode) ;

    void changeBackgroundColor(double r, double g, double b);






private:
    vtkNew<vtkActor> drawLine(const QVector3D& start, const QVector3D& end);

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

    vtkSmartPointer<vtkNamedColors> colors;
    vtkSmartPointer<HighlightInteractorStyle> style;
    vtkSmartPointer<DragInteractorStyle> style_drag;
    vtkSmartPointer<vtkInteractorStyleSwitch> style_switch;

    vtkSmartPointer<vtkConeSource> laser;
    vtkSmartPointer<vtkCubeSource> sensor_case;
    vtkSmartPointer<vtkActor> actor_model;
    vtkSmartPointer<vtkActor> actor_laser;
    vtkSmartPointer<vtkActor> actor_sensor_case;
    vtkSmartPointer<vtkActor> actor_sensor_complete;

    vtkSmartPointer<vtkAssembly> sensor_laser;


    QVector<vtkSmartPointer<vtkActor>> lasers;
    QVector<vtkSmartPointer<vtkActor>> sensors;
    QVector<vtkSmartPointer<vtkActor>> lines;
    vtkSmartPointer<vtkActor> actor_traj;

    vtkSmartPointer<vtkAxesActor> actor_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> widget_axes;

   // vtkSmartPointer<vtkSTLReader> polydata_model;

    bool isDragModeOn = false;

};

#endif // RENDERVTK_H
