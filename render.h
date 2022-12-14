#ifndef MYRENDERER_H
#define MYRENDERER_H

#include <sensor.h>
#include <Qt3DRender/Qt3DRender>
#include <Qt3DExtras/Qt3DExtras>
#include <QtCore/QUrl>
#include <QWidget>
#include "trajectorynode.h"

#ifndef PI
#define PI 3.1415926535
#endif

struct Axes
{
    QVector3D origin;
    QVector<Qt3DCore::QEntity*> lines_arrows;


    Qt3DCore::QEntity *arrowX;
    Qt3DCore::QEntity *arrowY;
    Qt3DCore::QEntity *arrowZ;

    Qt3DExtras::QConeMesh *arrowX_mesh;
    Qt3DExtras::QConeMesh *arrowY_mesh;
    Qt3DExtras::QConeMesh *arrowZ_mesh;

    Qt3DCore::QTransform *arrowX_transform;
    Qt3DCore::QTransform *arrowY_transform;
    Qt3DCore::QTransform *arrowZ_transform;

    Qt3DExtras::QPhongMaterial *arrowX_Material;
    Qt3DExtras::QPhongMaterial *arrowY_Material;
    Qt3DExtras::QPhongMaterial *arrowZ_Material;

};

class myRenderer
{

public:
    myRenderer(){};
    myRenderer(sensor sensor_model, QString name_file_);
    QWidget *container_stl;
    QWidget *container_axis;

    void updateRenderer(sensor sensor_model);
    void changeModel(QString name_file, sensor sensor_model);
    void addNewPoint(QVector<trajectoryNode> node, sensor sensor_model);
    void deletePoints();
    void deleteTraj();
    void insertDefect(QVector<QVector3D> pos);
    void drawAxes(QVector3D &origin, Qt3DCore::QEntity *entity);
    void updateAxes(QVector3D &origin);
    void insertTraj(QVector<QVector3Dd> pos);

    void setRotation0(Qt3DCore::QEntity *entity);

    void changeBackgroundColor(QColor color);


private:
    //sensor sensor_model;
    Qt3DExtras::Qt3DWindow *view;
    Qt3DCore::QEntity *rootEntity;

    Qt3DExtras::Qt3DWindow *view_axis;
    Qt3DCore::QEntity *rootEntity_axis;


    Qt3DCore::QEntity *modelEntity;
    Qt3DRender::QMesh *modelMesh;// = new Qt3DRender::QMesh;

    Qt3DCore::QEntity *laserEntity;
    Qt3DExtras::QConeMesh *laserMesh;// = new Qt3DExtras::QConeMesh;
    Qt3DCore::QTransform *laserTransform;// = new Qt3DCore::QTransform;

    Qt3DCore::QEntity *sensorEntity;
    Qt3DExtras::QCuboidMesh *sensorMesh;// = new Qt3DExtras::QCuboidMesh;
    Qt3DCore::QTransform *sensorTransform;// = new Qt3DCore::QTransform;


    QVector<Qt3DCore::QEntity*> lasers;
    QVector<Qt3DCore::QEntity*> sensors;
    QVector<Qt3DCore::QEntity*> lines;

    Qt3DCore::QEntity *trajEntity;

    Qt3DRender::QCamera *camera;
    Qt3DRender::QCamera *camera_axis;

    Qt3DRender::QObjectPicker *picker;

    Axes axis;

    void picker_Clicked(Qt3DRender::QPickEvent *pick);

    void freeMemory();

};

#endif // RENDER_H
