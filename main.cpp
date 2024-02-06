#include "MainWindow.h"

#include <QApplication>

Q_DECLARE_METATYPE(KDNode)
Q_DECLARE_METATYPE(sensor)
Q_DECLARE_METATYPE(QVector<trajectoryNode>)
Q_DECLARE_METATYPE(GenTraj_options)
Q_DECLARE_METATYPE(renderVTK)
Q_DECLARE_METATYPE(vtkSmartPointer<vtkPolyData>)
//Q_DECLARE_METATYPE(TriPluginInterface)


int main(int argc, char *argv[])
{
    qRegisterMetaType<KDNode>("KDNode");
    qRegisterMetaType<sensor>("sensor");
    qRegisterMetaType<QVector<trajectoryNode>>("QVector<trajectoryNode>");
    qRegisterMetaType<QVector<trajectoryNode>>("QVector<QVector3Dd>");
    qRegisterMetaType<GenTraj_options>("GenTraj_options");
    qRegisterMetaType<vtkSmartPointer<vtkPolyData>>("vtkSmartPointer<vtkPolyData>");
    qRegisterMetaType<renderVTK>("renderVTK");
  //  qRegisterMetaType<TriPluginInterface>("TriPluginInterface");

    QApplication a(argc, argv);
    a.setApplicationName("TriPlay");


    // CSS file
//    QFile styleFile(":/MyRes/images/estilo.css"); // Reemplaza con la ruta correcta a tu archivo CSS
//    styleFile.open(QFile::ReadOnly);
//    QString style = QLatin1String(styleFile.readAll());
//    a.setStyleSheet(style);


    //Splash screen
    QPixmap pixmap(":/MyRes/images/logoTriPlayground.svg");
    QSplashScreen splash(pixmap, Qt::WindowStaysOnTopHint);
    splash.show();

    MainWindow w;
    w.setWindowTitle("TriPlay");

    QTimer::singleShot(1000, &splash,SLOT(close())); // Timer
    QTimer::singleShot(1000,&w,SLOT(show()));

   // w.show();

  //  std::cout << qApp->argv() << std::endl;

    return a.exec();
}


//BUSCAR TODOS LOS SARA/SARARHT DEL PROYECTO PARA QUITARLOS!!!!
