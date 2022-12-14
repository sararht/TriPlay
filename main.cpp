#include "MainWindow.h"

#include <QApplication>

Q_DECLARE_METATYPE(KDNode)
Q_DECLARE_METATYPE(sensor)
Q_DECLARE_METATYPE(QVector<trajectoryNode>)
Q_DECLARE_METATYPE(GenTraj_options)
Q_DECLARE_METATYPE(renderVTK)
Q_DECLARE_METATYPE(vtkSmartPointer<vtkPolyData>)


int main(int argc, char *argv[])
{
    qRegisterMetaType<KDNode>("KDNode");
    qRegisterMetaType<sensor>("sensor");
    qRegisterMetaType<QVector<trajectoryNode>>("QVector<trajectoryNode>");
    qRegisterMetaType<QVector<trajectoryNode>>("QVector<QVector3Dd>");
    qRegisterMetaType<GenTraj_options>("GenTraj_options");
    qRegisterMetaType<vtkSmartPointer<vtkPolyData>>("vtkSmartPointer<vtkPolyData>");
    qRegisterMetaType<renderVTK>("renderVTK");

    QApplication a(argc, argv);
    a.setApplicationName("TriPlay");

    //Splash screen
    QPixmap pixmap(":/MyRes/images/logoTriPlayground.svg");
    QSplashScreen splash(pixmap, Qt::WindowStaysOnTopHint);
    splash.show();

    MainWindow w;
    w.setWindowTitle("TriPlay");

    QTimer::singleShot(1000, &splash,SLOT(close())); // Timer
    QTimer::singleShot(1000,&w,SLOT(show()));

   // w.show();

    return a.exec();
}


//BUSCAR TODOS LOS SARA/SARARHT DEL PROYECTO PARA QUITARLOS!!!!
