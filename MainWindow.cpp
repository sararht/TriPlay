#include "MainWindow.h"
#include "ui_MainWindow.h"
//#include "ui_help.h"


bool USE_VTK_RENDER = true;

void printQVector3D(QVector3D v)
{
    std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
}


void MainWindow::wdgDefectGuide()
{
    wdg_helpID = new QTabWidget();

    wdg_bolloExample = new QLabel("\n\n"
                                       "   <Bollo> \n\n "
                                       "\t <posxyz>[950.0, -950.0, 200.0]</posxyz> \n\n "
                                       "\t <longitud>100</longitud> \n\n "
                                       "\t <profundidad>10</profundidad> \n\n "
                                       "\t <ancho>50</ancho> \n\n"
                                       "\t <normal>[0 1 0]</normal> \n\n"
                                       "\t <direccion_deg>0</direccion_deg10> \n\n"
                                       "   </Bollo>");
    wdg_bolloExample->setTextInteractionFlags(Qt::TextSelectableByMouse);
    wdg_bolloExample->setAlignment(Qt::AlignTop);

    wdg_grietaExample = new QLabel("\n\n"
                                        "   <Grieta> \n\n "
                                        "\t <puntos> \n"
                                            "\t\t <posxyz>[950.0, -950.0, 200.0]</posxyz> \n "
                                            "\t\t <posxyz>[730.0, -950.0, 310.0]</posxyz> \n "
                                            "\t\t <posxyz>[780.0, -950.0, 330.0]</posxyz> \n "
                                            "\t\t <posxyz>[800.0, -950.0, 315.0]</posxyz> \n "
                                            "\t\t <posxyz>[860.0, -950.0, 350.0]</posxyz> \n "
                                        "\t </puntos> \n\n"

                                        "\t <profundidad>10</profundidad> \n "
                                        "\t <ancho>50</ancho> \n"
                                        "\t <normal>[0 1 0]</normal> \n\n"
                                        "\t <columnasPrimer>30</columnasPrimer> \n"
                                        "\t <columnasUltimo>50</columnasUltimo> \n"

                                    "   </Grieta>");
    wdg_grietaExample->setTextInteractionFlags(Qt::TextSelectableByMouse);
    wdg_grietaExample->setAlignment(Qt::AlignTop);

    wdg_picoExample = new QLabel("\n\n"
                                      "   <Pico> \n\n "
                                           "\t <posxyz>[950.0, -950.0, 200.0]</posxyz> \n\n "
                                           "\t <profundidad>10</profundidad> \n\n "
                                           "\t <ancho>50</ancho> \n\n"
                                           "\t <normal>[0 1 0]</normal> \n\n"
                                           "\t <direccion_deg>0</direccion_deg10> \n\n"
                                       "   </Pico>");
    wdg_picoExample->setTextInteractionFlags(Qt::TextSelectableByMouse);
    wdg_picoExample->setAlignment(Qt::AlignTop);


    wdg_personalizedExample = new QLabel("\n\n"
                                       "   Utilizar programa de Matlab para obtener los defectos personalizados.");
    wdg_personalizedExample->setTextInteractionFlags(Qt::TextSelectableByMouse);
    wdg_personalizedExample->setAlignment(Qt::AlignTop);


    wdg_helpID->addTab(wdg_bolloExample,"Bollo");
    wdg_helpID->addTab(wdg_grietaExample,"Grieta");
    wdg_helpID->addTab(wdg_picoExample,"Pico");
    wdg_helpID->addTab(wdg_personalizedExample,"Personalizado");

    wdg_helpID->setWindowTitle("Defect Examples");
    wdg_helpID->setMinimumSize(QSize(500,400));
    wdg_helpID->resize(QSize(900,600));
}
void MainWindow::wdgVisualizationGuide()
{
    wdg_guideVisualization = new QTableWidget(9,2);

    wdg_guideVisualization->setHorizontalHeaderLabels(QStringList({"Input","Action"}));
    wdg_guideVisualization->verticalHeader()->hide();
    wdg_guideVisualization->setMouseTracking(false);
    wdg_guideVisualization->setSelectionMode(QAbstractItemView::NoSelection);

    wdg_guideVisualization->setItem(0,0,new QTableWidgetItem("Left mouse button"));
    wdg_guideVisualization->setItem(1,0,new QTableWidgetItem("Right mouse button"));
    wdg_guideVisualization->setItem(2,0,new QTableWidgetItem("Both left and right mouse button"));
    wdg_guideVisualization->setItem(3,0,new QTableWidgetItem("Mouse scroll wheel"));
    wdg_guideVisualization->setItem(4,0,new QTableWidgetItem("Arrow keys"));
    wdg_guideVisualization->setItem(5,0,new QTableWidgetItem("Page up and page down keys"));
    wdg_guideVisualization->setItem(6,0,new QTableWidgetItem("Shift key"));
    wdg_guideVisualization->setItem(7,0,new QTableWidgetItem("Alt key"));
    wdg_guideVisualization->setItem(8,0,new QTableWidgetItem("Escape"));

    wdg_guideVisualization->setItem(0,1,new QTableWidgetItem("While the left mouse button is pressed, mouse movement along x-axis moves the camera left and right and movement along y-axis moves it up and down."));
    wdg_guideVisualization->setItem(1,1,new QTableWidgetItem("While the right mouse button is pressed, mouse movement along x-axis pans the camera around the camera view center and movement along y-axis tilts it around the camera view center."));
    wdg_guideVisualization->setItem(2,1,new QTableWidgetItem("While both the left and the right mouse button are pressed, mouse movement along y-axis zooms the camera in and out without changing the view center."));
    wdg_guideVisualization->setItem(3,1,new QTableWidgetItem("Zooms the camera in and out without changing the view center."));
    wdg_guideVisualization->setItem(4,1,new QTableWidgetItem("Move the camera vertically and horizontally relative to camera viewport."));
    wdg_guideVisualization->setItem(5,1,new QTableWidgetItem("Move the camera forwards and backwards."));
    wdg_guideVisualization->setItem(6,1,new QTableWidgetItem("Changes the behavior of the up and down arrow keys to zoom the camera in and out without changing the view center. The other movement keys are disabled."));
    wdg_guideVisualization->setItem(7,1,new QTableWidgetItem("Changes the behovior of the arrow keys to pan and tilt the camera around the view center. Disables the page up and page down keys."));
    wdg_guideVisualization->setItem(8,1,new QTableWidgetItem("Moves the camera so that entire scene is visible in the camera viewport."));

    wdg_guideVisualization->resizeColumnsToContents();
    wdg_guideVisualization->setWindowTitle("Visualization Guide");
    wdg_guideVisualization->setMinimumSize(QSize(1000,310));
    wdg_guideVisualization->setMaximumSize(QSize(1510,310));

    wdg_guideVisualization->resize(QSize(1510,310));
}

void MainWindow::wdgDefectSelection()
{
    wdg_defect_selection = new defectSelection;
    wdg_defect_selection->setWindowTitle("Defect insertion");

    //Rellenar y completar widget
    //wdg_defect_selection->show(); //Que se muestre cuando lo necesite
}
void MainWindow::wdgSensorParams()
{

    wSensorParams = new QWidget();
    auto lay_general = new QHBoxLayout();
    auto lay = new QVBoxLayout();
    QHBoxLayout *slay1 = new QHBoxLayout;
    QHBoxLayout *slay2 = new QHBoxLayout;
    //QHBoxLayout *slay3 = new QHBoxLayout;
    QHBoxLayout *slay4 = new QHBoxLayout;
    QHBoxLayout *slay5 = new QHBoxLayout;
    QHBoxLayout *slay6 = new QHBoxLayout;
    QHBoxLayout *slay7 = new QHBoxLayout;
    QHBoxLayout *slay8 = new QHBoxLayout;


    slay1->addWidget(ui->label_9);
    slay1->addWidget(ui->fovSpinBox);

    slay2->addWidget(ui->label_23);
    slay2->addWidget(ui->fpsSpinBox);

  //  slay3->addWidget(ui->label_7);
  //  slay3->addWidget(ui->rangeSpinBox);

    slay7->addWidget(ui->label_11);
    slay7->addWidget(ui->workingRangeSpinBox);

    slay8->addWidget(ui->label_12);
    slay8->addWidget(ui->workingDistanceSpinBox);

    slay4->addWidget(ui->label_8);
    slay4->addWidget(ui->resolutionSpinBox);

    slay5->addWidget(ui->label_22);
    slay5->addWidget(ui->velSpinBox);

    slay6->addWidget(ui->label_10);
    slay6->addWidget(ui->uncertaintySpinBox);

  //  lay->addLayout(slay3);
    lay->addLayout(slay7);
    lay->addLayout(slay8);
    lay->addLayout(slay4);
    lay->addLayout(slay1);
    lay->addLayout(slay5);
    lay->addLayout(slay2);
    lay->addLayout(slay6);

    lay_general->addSpacing(25);
    lay_general->addLayout(lay);
    lay_general->addSpacing(25);

    wSensorParams->setLayout(lay_general);
    wSensorParams->setWindowTitle("Sensor Params");
    wSensorParams->setFixedSize(QSize(320,320));

    ui->rangeSpinBox->hide();
    ui->label_7->hide();
}
void MainWindow::wdgTrajectoryGeneratorParams()
{
    //Create widget plane selection
    wPlaneSelection = new QWidget();
    auto lay_general_ps = new QVBoxLayout();
    auto lay_planes = new QHBoxLayout();
    auto lay_axis = new QHBoxLayout();

    auto label1 = new QLabel("Select plane");
    QGroupBox *gb_planes = new QGroupBox();
    b_planeXY = new QRadioButton("XY");
    b_planeXZ = new QRadioButton("XZ");
    b_planeYZ = new QRadioButton("YZ");
    b_planeXZ->setChecked(true);
    gb_planes->setLayout(lay_planes);

    lay_planes->addWidget(b_planeXY);
    lay_planes->addWidget(b_planeXZ);
    lay_planes->addWidget(b_planeYZ);

    auto label2 = new QLabel("Select axis");
    QGroupBox *gb_axis = new QGroupBox();
    b_axisX = new QRadioButton("X");
    b_axisY = new QRadioButton("Y");
    b_axisZ = new QRadioButton("Z");
    b_axisZ->setChecked(true);
    gb_axis->setLayout(lay_axis);

    lay_axis->addWidget(b_axisX);
    lay_axis->addWidget(b_axisY);
    lay_axis->addWidget(b_axisZ);

    lay_general_ps->addWidget(label1);
    lay_general_ps->addWidget(gb_planes);
    lay_general_ps->addWidget(label2);
    lay_general_ps->addWidget(gb_axis);

    wPlaneSelection->setLayout(lay_general_ps);

    // Create widget trajGen
    wTrajGen = new QWidget();

    auto lay_general = new QVBoxLayout();
    auto lay_options = new QHBoxLayout();
    auto lay_buttons = new QHBoxLayout();

    auto label1_ = new QLabel("Select range option");
    QGroupBox *gb_options = new QGroupBox();
    b_opt1 = new QRadioButton("Min range");
    b_opt2 = new QRadioButton("Working distance");
    b_opt3 = new QRadioButton("Max Range");
    b_opt2->setChecked(true);
    gb_options->setLayout(lay_options);
    lay_options->addWidget(b_opt1);
    lay_options->addWidget(b_opt2);
    lay_options->addWidget(b_opt3);

    QPushButton *b_ok = new QPushButton("OK");
    QPushButton *b_cancel = new QPushButton("CANCEL");

    lay_buttons->addWidget(b_ok);
    lay_buttons->addWidget(b_cancel);

    lay_general->addWidget(label1_);
    lay_general->addWidget(gb_options);

    //lay_general->addWidget(wPlaneSelection);

//    lay_general->addWidget(label1);
//    lay_general->addWidget(gb_planes);
//    lay_general->addWidget(label2);
//    lay_general->addWidget(gb_axis);
    //lay_general->addLayout(lay_buttons);


    wTrajGen->setLayout(lay_general);
}
void MainWindow::wdgScanAllParams()
{
    //Widget steps
    auto lay_general = new QVBoxLayout();

    wStepsNumber = new QWidget();
    steps_sb = new QSpinBox;
    QLabel *label_sb = new QLabel;
    label_sb->setText("Steps: ");

    QHBoxLayout *layout_sb = new QHBoxLayout;
    QGroupBox *gb_sb = new QGroupBox();
    layout_sb->addWidget(label_sb);
    layout_sb->addWidget(steps_sb);
    gb_sb->setLayout(layout_sb);

    //lay_general->addWidget(wPlaneSelection);
    lay_general->addWidget(gb_sb);

    wStepsNumber->setLayout(lay_general);

    //Widget general
    wScanAll = new QWidget();
    QStackedWidget *swScanAll = new QStackedWidget;

    swScanAll->addWidget(wTrajGen);
    swScanAll->addWidget(wStepsNumber);

    pageComboBox = new QComboBox;
    pageComboBox->addItem(tr("Sensor distance"));
    pageComboBox->addItem(tr("Steps number"));

    connect(pageComboBox, SIGNAL(activated(int)),
            swScanAll, SLOT(setCurrentIndex(int)));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(pageComboBox);
    layout->addWidget(swScanAll);

    wScanAll->setLayout(layout);

}
void MainWindow::wdgTrajectoryNodesParams()
{
    // Crear check boxes para la primera pregunta
    QLabel labelPregunta1("Execute literal trajectory:");
    QCheckBox checkBoxPregunta1("Sí");

    // Crear un diseño para organizar los elementos de la primera pregunta
    QVBoxLayout layoutPregunta1;
    layoutPregunta1.addWidget(&labelPregunta1);
    layoutPregunta1.addWidget(&checkBoxPregunta1);

    // Crear el contenedor principal para la primera pregunta
    QWidget containerPregunta1;
    containerPregunta1.setLayout(&layoutPregunta1);

    // Crear check boxes para la segunda pregunta
    QLabel labelPregunta2("Split into different files:");
    QCheckBox checkBoxPregunta2("Sí");

    // Crear un diseño para organizar los elementos de la segunda pregunta
    QVBoxLayout layoutPregunta2;
    layoutPregunta2.addWidget(&labelPregunta2);
    layoutPregunta2.addWidget(&checkBoxPregunta2);

    // Crear el contenedor principal para la segunda pregunta
    QWidget containerPregunta2;
    containerPregunta2.setLayout(&layoutPregunta2);

    // Crear un diseño principal para organizar ambos contenedores
    QVBoxLayout mainLayout;
    mainLayout.addWidget(&containerPregunta1);
    mainLayout.addWidget(&containerPregunta2);

    // Configurar el cuadro de diálogo principal con el diseño
  //  dialog.setLayout(&mainLayout);

    // Agregar botón de "Aceptar"
    QPushButton btnAceptar("Aceptar");
    mainLayout.addWidget(&btnAceptar);

    wTrajectoryNodes = new QWidget();
    wTrajectoryNodes->setLayout(&mainLayout);


}


bool MainWindow::loadPlugin(const QString &desiredPluginName)
{
    QDir pluginsDir(QCoreApplication::applicationDirPath());
#if defined(Q_OS_WIN)
    if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
        pluginsDir.cdUp();
#elif defined(Q_OS_MAC)
    if (pluginsDir.dirName() == "MacOS") {
        pluginsDir.cdUp();
        pluginsDir.cdUp();
        pluginsDir.cdUp();
    }
#endif
    pluginsDir.cd("../simulador/plugins");
    const QStringList entries = pluginsDir.entryList(QDir::Files);
    for (const QString &fileName : entries) {
        QPluginLoader pluginLoader(pluginsDir.absoluteFilePath(fileName));
        QObject *plugin = pluginLoader.instance();
     //   std::cout << "Error: " << pluginLoader.errorString().toStdString() << std::endl;
        if (plugin) {
            pluginInterface = qobject_cast<TriPluginInterface *>(plugin);
            if (pluginInterface && fileName == desiredPluginName) {
                    return true;
            }
            pluginLoader.unload();
        }
    }

    return false;
}

#include<vtkVersion.h>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowIcon(QIcon(":/MyRes/images/logoTriPlayground.svg"));


    // Create Widgets
    wdgDefectSelection();
    wdgSensorParams();
    wdgVisualizationGuide();
    wdgDefectGuide();
    wdgTrajectoryGeneratorParams();
    wdgScanAllParams();
    wdgTrajectoryGeneratorParams();

    //--------------VISIBILITY------------------------

    ui->button_clean->setVisible(false);
    ui->button_setPunto->setVisible(false);
    ui->button_calcular->setVisible(false);
    ui->button_getTrajectory->setVisible(false);
    ui->button_defect->setVisible(false);
    ui->label_24->setVisible(false);
    ui->button_fileDialog->setVisible(false);
    ui->lineEdit_nameFile->setVisible(false);
    ui->button_refresh->setVisible(false);
    ui->listWidget->setVisible(false);
    ui->label_lW->setVisible(false);



    /// VTK VERSION
    std::cout << "VTK Version: " << vtkVersion::GetVTKVersion() << std::endl;

    ///LOAD PLUGINS----------------
    if (!loadPlugin("libtrajectorygeneratorplugin.so")) {std::cout << "PLUGIN NOT LOADED" << std::endl;}
    else
    {
        QStringList arguments = qApp->arguments();
        QCoreApplication *app = QCoreApplication::instance();
        int argc = app->arguments().at(0).toInt();

        char **argv = new char*[1];
        argv[0]= "/home/sara/sararht/TESIS/Codigo/simulador/QT/build-simulador-Qt_5_14_2_gcc_64-Release/simulador";
        //luginInterface->initPlugin(1,argv);
        pluginInterface->precalculate();
        pluginInterface->calculate();
        pluginInterface->postcalculate();
     //   pluginInterface->run();
        //pluginInterface->start();

    }


    ///----------------------------



    QVector3Dd A(0,0,0);

    //BUILD THE TREE---------------------------------------------------------------------------------------------
//    name_file = "/home/sara/sararht/TESIS/Codigo/modelos/PUERTA_DELANTERA_IZQ_sub.stl"; //pieza_achatada_grande2.stl"; //chapa2.stl";///pinzas/fuchosa_r.stl";//chapa2.stl" ; //PUERTA_DELANTERA_IZQ.stl
//    name_file = "../simulador/stl_examples/PUERTA_DELANTERA_IZQ_0_3.stl";
    name_file = "/home/sara/sararht/TESIS/Codigo/modelos/pieza_achatada_grande_90.stl";

    std::vector<float> coords, normals;
    std::vector<unsigned int> tris, solids;
    bool is_read = stl_reader::ReadStlFile(name_file.toStdString().c_str(),coords, normals, tris, solids);

    if(is_read)
    {
        const size_t numTris = tris.size() / 3;

        vtkNew<vtkPoints> points;
        points->SetNumberOfPoints(tris.size());
        vtkNew<vtkCellArray> polys;

        ui->lineEdit_nameFile->setText(name_file);

        for(size_t itri = 0; itri < numTris; ++itri) {
            polys->InsertNextCell(3);

            for(size_t icorner = 0; icorner < 3; ++icorner) {
                float* c = &coords[3 * tris [3 * itri + icorner]];
                QVector3Dd point(c[0],c[1],c[2]);
                if (icorner == 0)
                    v0.push_back(point);
                else if (icorner == 1)
                    v1.push_back(point);
                else if (icorner == 2)
                    v2.push_back(point);

                points->SetPoint(itri*3+icorner,c[0],c[1],c[2]);
                polys->InsertCellPoint(itri*3+icorner);

            }

            QVector3Dd A = v1[itri]-v0[itri];
            QVector3Dd B = v2[itri]-v0[itri];
            QVector3Dd N = QVector3Dd::crossProduct(A,B);
            normals_tri.push_back(N/(sqrt(N.x()*N.x()+N.y()*N.y()+N.z()*N.z())));

        }

        trisModel.reserve(v0.size());
        for(int i=0; i<v0.size();i++)
        {
           Triangle* tri_i;
           tri_i = new Triangle(v0[i],v1[i],v2[i], normals_tri[i]);
           trisModel.push_back(tri_i);


        }

        tree = *tree.build(trisModel,0);

        polydata_model = vtkSmartPointer<vtkPolyData>::New();
        polydata_model->SetPoints(points);
        polydata_model->SetPolys(polys);
    }


    //BUILD SENSOR-------------------------------------------------------
    //Parámetros sensor
    double pi=3.141592;
    double alpha = 61.5*pi/180;
    double w_distance = 400;
    double w_range = 300;
   // double alcance = w_distance+w_range/2;
    double n_rayos = 4096;

    A = QVector3Dd(v0[0].x(), v0[0].y()+200, v0[0].z()-100);

    double roll = 0;
    double pitch = 90;
    double yaw = 0;

    sensor_model = sensor(A,roll,pitch,yaw,w_range,w_distance,n_rayos,alpha,0);




    //-----------
    //Build renderer
    if(USE_VTK_RENDER)
    {
        renderer_vtk = renderVTK(sensor_model,polydata_model);
        ui->layout_graphics->addWidget(renderer_vtk.widget);
    }
    else
    {
         ui->layout_graphics->addWidget(renderer.container_stl);
    }

    controller_.updatePolydataModel(polydata_model);

    //-------------------------
    //renderer_vtk.startInteractor();
    //renderer.container_axis->show();


    initSpinBoxes();
    updateSpinBoxes();

    QThread *thread_controller = new QThread(this);
    controller_.moveToThread(thread_controller);

    //Conecting signals
    connect(ui->xSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->ySpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->zSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->rollSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->pitchSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->yawSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesPositionChanges()));
    connect(ui->rangeSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesCaracteristicsChanges()));
    connect(ui->workingRangeSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesCaracteristicsChanges()));
    connect(ui->workingDistanceSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesCaracteristicsChanges()));
    connect(ui->resolutionSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesCaracteristicsChanges()));
    connect(ui->fovSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valuesCaracteristicsChanges()));

    connect(this,SIGNAL(button_traj_clicked(QVector3Dd,QVector3Dd,QQuaternion,QQuaternion,double,double,double,double,double,double,KDNode)), &controller_, SLOT(trajectory(QVector3Dd,QVector3Dd,QQuaternion,QQuaternion,double,double,double,double,double,double,KDNode)));

    connect(&controller_,SIGNAL(frameDone(int,int,sensor)),this, SLOT(updateRender(int,int,sensor)));
    connect(&controller_,SIGNAL(frameDoneTraj(QVector<trajectoryNode>,QVector<int>,int,int,sensor)),this, SLOT(frameDoneTrajUpdate(QVector<trajectoryNode>,QVector<int>,int,int,sensor)));

    connect(&controller_,SIGNAL(endTrajFrom()),this, SLOT(clearTrajRender()));

    connect(this,SIGNAL(button_traj_node_clicked(QVector<trajectoryNode>,double,double,double,double,double,double,double,KDNode, QString, bool)), &controller_, SLOT(getTrajectoryNodes(QVector<trajectoryNode>,double,double,double,double,double,double,double,KDNode, QString, bool)));
    connect(this,SIGNAL(button_traj_generator_clicked(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString)), &controller_, SLOT(trajectoryGenerator4(GenTraj_options,QVector<trajectoryNode>,QVector3D,double,double,double,double,double,double,double,KDNode, QString))); //AQUIII

    connect(this,SIGNAL(button_traj_node_from_clicked(QVector<QVector3Dd>,QVector<QVector3Dd>,double,double,double,double,double,KDNode, QString)), &controller_, SLOT(getTrajectoryNodesFromFile(QVector<QVector3Dd>,QVector<QVector3Dd>,double,double,double,double,double,KDNode, QString)));

    connect(this,SIGNAL(button_defect_clicked(QString,QString,QString)), &controller_, SLOT(insertDefect(QString,QString,QString)));
    connect(this,SIGNAL(button_defect_selection_clicked(QString,QString,QString,vtkSmartPointer<vtkPolyData>)), &controller_, SLOT(insertDefectSelection(QString,QString,QString,vtkSmartPointer<vtkPolyData>)));
    connect(this,SIGNAL(button_defect_selection_predefined_clicked(QString,QString,vtkSmartPointer<vtkPolyData>,QString,double,float,QString,crackDef_params*)), &controller_, SLOT(insertDefectSelectionPredefined(QString,QString,vtkSmartPointer<vtkPolyData>, QString, double, float, QString, crackDef_params*)));

    connect(&controller_,SIGNAL(defectInserted(QString)),this, SLOT(updateRenderAfterDefect(QString)));
    connect(&controller_,SIGNAL(defectInserted_error(QString)),this, SLOT(errorAfterDefect(QString)));

    connect(this,SIGNAL(button_scan_all_clicked(int,KDNode,double,double,double,double,double,double,double,QString)), &controller_, SLOT(scanAllPiece(int,KDNode,double,double,double,double,double,double,double,QString)));

    connect(this, SIGNAL(button_dragMode_clicked(bool, renderVTK)), &controller_, SLOT(updateDragMode(bool, renderVTK)));
    connect(&controller_, SIGNAL(updateUi(double*,double*)), this, SLOT(updateUi_drag(double*,double*)));

    thread_controller->start();
    thread_controller->setPriority(QThread::TimeCriticalPriority);

    //Update Render
    if(USE_VTK_RENDER)
        renderer_vtk.updateRendered(sensor_model);
    else
        renderer.updateRenderer(sensor_model);

    ui->progressBar->hide();
    ui->button_stop->hide();

    //Accept drops
    this->setAcceptDrops(true);

    //controller_.trajectoryGenerator4();

}

MainWindow::~MainWindow()
{
    delete ui;
    //Connecting the signal
}

void MainWindow::clearTrajRender()
{
//    if(USE_VTK_RENDER)
//        renderer_vtk.deleteTraj();
//    else
//        renderer.deleteTraj();
}

void MainWindow::updateRender(int frame,int n_frames, sensor new_sensor)
{
    float value = float((frame+1))/float(n_frames)*100.0f;
    ui->progressBar->setValue(value);
    if (value  == 100)
    {
        ui->progressBar->hide();
        ui->button_stop->hide();
    }

    //new_sensor.roll = new_sensor.roll;//*PI/180;
    //new_sensor.pitch = new_sensor.pitch;//*PI/180;
    //new_sensor.yaw = new_sensor.yaw;//*PI/180;

    if(USE_VTK_RENDER)
        renderer_vtk.updateRendered(new_sensor);
    else
        renderer.updateRenderer(new_sensor);
}

void MainWindow::frameDoneTrajUpdate(QVector<trajectoryNode> nodes, QVector<int> vFrames, int id_node, int frame, sensor new_sensor)
{
    int n_frames = std::accumulate(vFrames.begin(), vFrames.end(), 0);
    int frames_node = vFrames[id_node];
    float value = float((frame+1))/float(n_frames)*100.0f;

    ui->progressBar->setValue(value);
    if (value  == 100)
    {
        ui->progressBar->hide();
        ui->button_stop->hide();
    }


    if(id_node>0)
    {
        for (int i=id_node; i>0; i--)
            frame= frame - vFrames[i-1];
    }

    QVector3Dd pos_ini = nodes[id_node].pos();
    QVector3Dd pos_end = nodes[id_node+1].pos();
    QQuaternion q_ini = nodes[id_node].q();
    QQuaternion q_end = nodes[id_node+1].q();
    double cte_x= (pos_end.x() - pos_ini.x()) / frames_node ;
    double cte_y= (pos_end.y() - pos_ini.y()) / frames_node ;
    double cte_z= (pos_end.z() - pos_ini.z()) / frames_node ;
    double pos_x = pos_ini.x() + frame*cte_x;
    double pos_y = pos_ini.y() + frame*cte_y;
    double pos_z = pos_ini.z() + frame*cte_z;

    QQuaternion q_i;

    if (q_ini != q_end)
    {
        double t = double(frame)/double(frames_node);
        q_i = QQuaternion::slerp(q_ini, q_end, t);
    }
    else
    {
        q_i =q_ini;
    }

    QVector3Dd origin_(pos_x,pos_y,pos_z);
    QQuaternion q_ = q_i;
    sensor sensor_tmp(origin_, q_, new_sensor.working_range, new_sensor.working_distance, new_sensor.resolution, new_sensor.FOV, new_sensor.uncertainty);

    QVector3D eulers = q_.toEulerAngles();
    if(USE_VTK_RENDER)
        renderer_vtk.updateRendered(sensor_tmp/*, true*/); //CAMBIO ESTO
    else
        renderer.updateRenderer(sensor_tmp);


}

void MainWindow::initSpinBoxes()
{
    ui->xSpinBox->setRange(sensor_model.origin.x() - 10000, sensor_model.origin.x() + 10000);
    ui->ySpinBox->setRange(sensor_model.origin.y() - 10000, sensor_model.origin.y() + 10000);
    ui->zSpinBox->setRange(sensor_model.origin.z() - 10000, sensor_model.origin.z() + 10000);
    ui->xSpinBox->setSingleStep(10);
    ui->ySpinBox->setSingleStep(10);
    ui->zSpinBox->setSingleStep(10);

    ui->rollSpinBox->setRange(-360,360);
    ui->pitchSpinBox->setRange(-360,360);
    ui->yawSpinBox->setRange(-360,360);
    ui->rangeSpinBox->setRange(0,4000);
    ui->workingRangeSpinBox->setRange(0,4000);
    ui->workingDistanceSpinBox->setRange(0,4000);
    ui->resolutionSpinBox->setRange(10,5000);
    ui->fovSpinBox->setRange(30,180);

    ui->rangeSpinBox->setSingleStep(10);
    ui->workingRangeSpinBox->setSingleStep(10);
    ui->workingDistanceSpinBox->setSingleStep(10);

    ui->fpsSpinBox->setRange(0,10000);
    ui->fpsSpinBox->setValue(500);

    ui->velSpinBox->setRange(0,100);
    ui->velSpinBox->setValue(0.1);
    ui->velSpinBox->setSingleStep(0.1);

    ui->uncertaintySpinBox->setSingleStep(1);
    ui->uncertaintySpinBox->setRange(0,10000);
    ui->uncertaintySpinBox->setValue(20);

}

void MainWindow::updateSpinBoxes()
{

    auto a = this->sensor_model.origin;
    auto r = this->sensor_model.roll, p = this->sensor_model.pitch, y=this->sensor_model.yaw;
    ui->xSpinBox->setValue(a.x());
    ui->ySpinBox->setValue(a.y());
    ui->zSpinBox->setValue(a.z());

    ui->rollSpinBox->setValue(r);
    ui->pitchSpinBox->setValue(p);
    ui->yawSpinBox->setValue(y);

    ui->rangeSpinBox->setValue(this->sensor_model.range);
    ui->workingRangeSpinBox->setValue(this->sensor_model.working_range);
    ui->workingDistanceSpinBox->setValue(this->sensor_model.working_distance);

    ui->resolutionSpinBox->setValue(this->sensor_model.resolution);
    ui->fovSpinBox->setValue(this->sensor_model.FOV*180/PI);




}


void MainWindow::valuesPositionChanges()
{
    QVector3Dd new_origin(ui->xSpinBox->value(), ui->ySpinBox->value(), ui->zSpinBox->value());
    sensor_model.updatePositionSensor(new_origin, ui->rollSpinBox->value(), ui->pitchSpinBox->value(), ui->yawSpinBox->value());
    if(USE_VTK_RENDER)
        renderer_vtk.updateRendered(sensor_model);
    else
        renderer.updateRenderer(sensor_model);


}

void MainWindow::valuesCaracteristicsChanges()
{
    sensor_model.updateCaracteristicsSensor(ui->workingRangeSpinBox->value(),ui->workingDistanceSpinBox->value(), ui->fovSpinBox->value()*PI/180, ui->resolutionSpinBox->value());
    if(USE_VTK_RENDER)
        renderer_vtk.updateRendered(sensor_model);
    else
        renderer.updateRenderer(sensor_model);

}


void MainWindow::on_button_refresh_clicked(bool change = false)
{
    //Abrir nuevo modelo

    std::vector<float> coords, normals;
    std::vector<unsigned int> tris, solids;

    QString new_name = ui->lineEdit_nameFile->text();

    if (new_name == name_file && !change)
    {
        QVector3Dd new_origin(v0[0].x(), v0[0].y()+200, v0[0].z()-100);

        double new_roll=0, new_pitch=90, new_yaw=0;
        sensor_model.updatePositionSensor(new_origin, new_roll, new_pitch, new_yaw);
        updateSpinBoxes();
        if(USE_VTK_RENDER)
            renderer_vtk.updateRendered(sensor_model);
        else
            renderer.updateRenderer(sensor_model);

    }

    else
    {
        name_file = new_name;

        stl_reader::ReadStlFile(new_name.toStdString().c_str(),coords, normals, tris, solids);

        const size_t numTris = tris.size() / 3;
        v0.clear();v1.clear();v2.clear();
        v0.shrink_to_fit();v1.shrink_to_fit();v2.shrink_to_fit();
        normals_tri.clear(); //CAMBIO
        normals_tri.shrink_to_fit();

        vtkNew<vtkPoints> points;
        points->SetNumberOfPoints(tris.size());
        vtkNew<vtkCellArray> polys;

        for(size_t itri = 0; itri < numTris; ++itri) {
            polys->InsertNextCell(3);

            for(size_t icorner = 0; icorner < 3; ++icorner) {
                float* c = &coords[3 * tris [3 * itri + icorner]];
                QVector3Dd point(c[0],c[1],c[2]);
                if (icorner == 0)
                    v0.push_back(point);
                else if (icorner == 1)
                    v1.push_back(point);
                else if (icorner == 2)
                    v2.push_back(point);

                points->SetPoint(itri*3+icorner,c[0],c[1],c[2]);
                polys->InsertCellPoint(itri*3+icorner);
            }
//            float* n = &normals[itri];
//            normals_tri.push_back(QVector3Dd(n[0],n[1],n[2]));

            QVector3Dd A = v1[itri]-v0[itri];
            QVector3Dd B = v2[itri]-v0[itri];
            QVector3Dd N = QVector3Dd::crossProduct(A,B);
            normals_tri.push_back(N/(sqrt(N.x()*N.x()+N.y()*N.y()+N.z()*N.z())));
        }


        //Liberar memoria de los punteros a triángulo
        for (auto p : trisModel)
            delete p;
        trisModel.clear();

       // std::vector<Triangle*>().swap(trisModel);
        trisModel.shrink_to_fit();
        trisModel.reserve(v0.size());

        for(int i=0; i<v0.size();i++)
        {
           Triangle* tri_i;
           tri_i = new Triangle(v0[i],v1[i],v2[i], normals_tri[i]);
           trisModel.push_back(tri_i);
        }

        //tree.delete_KDNode();
        tree = KDNode();
        tree = *tree.build(trisModel,0);

        QVector3Dd new_origin(v0[0].x(), v0[0].y()+200, v0[0].z()-100);

        double new_roll=0, new_pitch=90, new_yaw=0;
        sensor_model.updatePositionSensor(new_origin, new_roll, new_pitch, new_yaw);
        updateSpinBoxes();


        if(USE_VTK_RENDER)
        {
            qDebug("Changing model...");
            /*vtkSmartPointer<vtkPolyData>*/ polydata_model = vtkSmartPointer<vtkPolyData>::New();
            polydata_model->SetPoints(points);
            polydata_model->SetPolys(polys);
            renderer_vtk.changeModel(polydata_model);
            controller_.updatePolydataModel(polydata_model);

            renderer_vtk.updateRendered(sensor_model);
            qDebug("Changed");

        }
        else
        {
            renderer.changeModel(ui->lineEdit_nameFile->text(), sensor_model);
            renderer.updateRenderer(sensor_model);
        }

        //QVector3D axis_origin(sensor_model.origin.x()+800, sensor_model.origin.y()-500, sensor_model.origin.z()-200);
        //renderer.updateAxes(axis_origin);     

    }


}

void MainWindow::updateRenderAfterDefect(QString file_saving)
{
    //Indicar que acabó y se está actualizando el renderizado
    ui->lineEdit_nameFile->setText(file_saving);
    on_button_refresh_clicked(true);

    wdg_defect->hide();
    movie->stop();

}

void MainWindow::errorAfterDefect(QString file_saving)
{
    //Indicar que acabó y se está actualizando el renderizado
    QMessageBox::warning(this,"ERROR", "No se encuentra área donde insertar el defecto");
    ui->lineEdit_nameFile->setText(file_saving);
    //on_button_refresh_clicked();
    wdg_defect->hide();
    movie->stop();

}

// ACTION MENU BAR
void MainWindow::on_actionOpen_triggered()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open STL file", QDir::homePath(),"STL Files (*.stl)");
    if (!file_name.isEmpty() && !file_name.isNull())
    {
        ui->lineEdit_nameFile->setText(file_name);
        on_button_refresh_clicked(false);
    }
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    const QMimeData* mimeData = event->mimeData();
    if (mimeData->hasUrls())
    {
        QString path;
        QList<QUrl> urlList = mimeData->urls();

        if (urlList.size()==1)
        {

            path = urlList.at(0).toLocalFile();
            QFileInfo fi(path);
            if(fi.completeSuffix() == "stl")
            {
                 event->acceptProposedAction();
            }
        }
    }
}

void MainWindow::dropEvent(QDropEvent *event)
{
    ui->lineEdit_nameFile->setText(event->mimeData()->urls().at(0).toLocalFile());
    on_button_refresh_clicked(false);

}

void MainWindow::on_actionSave_As_triggered()
{
    QString p = QDir::homePath() +"/nuevo.stl";
    QString name_saving_ = QFileDialog::getSaveFileName(this, "Save As", p, "STL files (*.stl)");
    if (!name_saving_.isEmpty() && !name_saving_.isNull())
    {
        QString name_file_aux = name_file;
        name_file_aux.replace(".stl","");

        QString name_saving_aux = name_saving_;
        name_saving_aux.replace(".stl","");

        mimmo::MimmoGeometry * mimmo_aux = new mimmo::MimmoGeometry();
        mimmo_aux->setIOMode(IOMode::CONVERT);
        mimmo_aux->setReadDir("");
        mimmo_aux->setReadFileType(FileType::STL);
        mimmo_aux->setReadFilename(name_file_aux.toStdString());
        mimmo_aux->setWriteDir("");
        mimmo_aux->setWriteFileType(FileType::STL);
        mimmo_aux->setWriteFilename(name_saving_aux.toStdString());

        mimmo::Chain ch_aux;
        ch_aux.addObject(mimmo_aux);
        ch_aux.exec();

        delete mimmo_aux;
    }
}

void MainWindow::on_actionSensor_Parameters_triggered()
{
    wSensorParams->show();
}

void MainWindow::on_actionInsert_Defect_Selection_triggered()
{

    vtkSmartPointer<vtkPolyData> polySelection;
    if(renderer_vtk.getPolySelection(polySelection))
    {
        std::cout << "Selection has " << polySelection->GetNumberOfPoints() << " points." << std::endl;

        QString file_name = QFileDialog::getOpenFileName(this, "Insert defect", QDir::homePath(),"XML Files (*.xml)");
        //FILTRAR PARA SOLO PONER COGER XML
        if (!file_name.isEmpty() && !file_name.isNull())
        {
            QMessageBox::information(this, "INFO", "Defect selected. Save new model");
            QString p = QDir::homePath() +"/nuevo.stl";

            QString name_saving_ = QFileDialog::getSaveFileName(this, "Guardar el nuevo modelo STL", p, "STL files (*.stl)");
            if (!name_saving_.isEmpty() && !name_saving_.isNull())
            {
                emit button_defect_selection_clicked(file_name, name_file, name_saving_,polySelection);
                waitForDefectInsertion();
            }
        }

    }
    else
    {
        //Mensaje que te diga que tienes que seleccionar el área
        QMessageBox::warning(this, "Warning", "Select area to insert the defect.\nPress -R-, later click and drag with left mouse button");

    }

}

void MainWindow::on_actionInsert_Defect_triggered()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Insert defect", QDir::homePath(),"XML Files (*.xml)");
    //FILTRAR PARA SOLO PONER COGER XML
    if (!file_name.isEmpty() && !file_name.isNull())
    {
        QMessageBox::information(this, "INFO", "Defect selected. Save new model");
        QString p = QDir::homePath() +"/nuevo.stl";
        QString name_saving_ = QFileDialog::getSaveFileName(this, "Guardar el nuevo modelo STL", p, "STL files (*.stl)");
        if (!name_saving_.isEmpty() && !name_saving_.isNull())
        {

            emit button_defect_clicked(file_name, name_file, name_saving_);

            // Indicar que comienza
            wdg_defect = new QWidget();
            wdg_defect->setFixedSize(500,200);
            wdg_defect->setWindowFlags(Qt::WindowMinimizeButtonHint);
            movie = new QMovie(":/MyRes/images/loading.gif");
            movie->setScaledSize(QSize(250,250));

            QLabel *textLabel = new QLabel("INSERTANDO DEFECTO", wdg_defect);
            textLabel->setAlignment(Qt::AlignCenter);
            QLabel *gifLabel = new QLabel("", wdg_defect);
            gifLabel->setAlignment(Qt::AlignCenter);
            gifLabel->setMovie(movie);
            //gifLabel->setVisible(true);
            movie->start();

            QFrame* line = new QFrame();
            line->setFrameShape(QFrame::HLine);
            line->setFrameShadow(QFrame::Sunken);

            QVBoxLayout *vbl = new QVBoxLayout(wdg_defect);
            vbl->addWidget(textLabel);
            vbl->addWidget(line);
            vbl->addWidget(gifLabel);
            wdg_defect->show();

        }
    }

}

void MainWindow::on_actionGet_Profile_triggered()
{

    QVector3Dd origin(ui->xSpinBox->value(),ui->ySpinBox->value(), ui->zSpinBox->value());
    sensor_model.updatePositionSensor(origin, ui->rollSpinBox->value(), ui->pitchSpinBox->value(), ui->yawSpinBox->value() );
    sensor_model.updateCaracteristicsSensor(ui->workingRangeSpinBox->value(),ui->workingDistanceSpinBox->value(), ui->fovSpinBox->value()*PI/180,  ui->resolutionSpinBox->value());

    clock_t tStart2_ = clock();
    sensor_model.getMeasurement(tree, true);
    std::cout <<"Time mío " << (double)(clock() - tStart2_)/CLOCKS_PER_SEC << std::endl;

//    vtkSmartPointer<vtkOBBTree> kdtree;
//    kdtree = vtkSmartPointer<vtkOBBTree>::New();
//    kdtree->SetDataSet(this->polydata_model);
//    kdtree->BuildLocator();

//    clock_t tStart_ = clock();
//    sensor_model.getMeasurementParalell(kdtree, true);
//    std::cout <<"Time VTK " << (double)(clock() - tStart_)/CLOCKS_PER_SEC << std::endl;


    plot.show();
    plot.plot(sensor_model.sensor_data);

}

void MainWindow::on_actionGet_Trajectory_triggered()
{
    if (nodes.size()<2)
        QMessageBox::warning(this, "ERROR", "No hay suficientes puntos");

    else{
        QString path = QFileDialog::getExistingDirectory(this, "QFileDialog.getSaveDirectory", "");

        if (!path.isEmpty() && !path.isNull())
        {
            int frames = ui->fpsSpinBox->value();
            double fov = ui->fovSpinBox->value();
            double resolution = ui->resolutionSpinBox->value();
            double w_range = ui->workingRangeSpinBox->value();
            double w_distance = ui->workingDistanceSpinBox->value();
            double vel = ui->velSpinBox->value();
            double uncertainty = ui->uncertaintySpinBox->value();

            ui->progressBar->show();
            ui->button_stop->show();

            emit button_traj_node_clicked(nodes, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, path, false);
        }
    }
}

void MainWindow::on_actionSet_Point_triggered()
{
    QVector3Dd pos(ui->xSpinBox->value(), ui->ySpinBox->value(), ui->zSpinBox->value());
    QQuaternion q = sensor_model.q;

    trajectoryNode node_(pos,q);
    nodes.push_back(node_);

    QString text;
    std::stringstream text_;
    text_ << nodes.size() << " - XYZ(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")";
    text = QString::fromStdString(text_.str());
    ui->listWidget->addItem(text);

    if(USE_VTK_RENDER)
        renderer_vtk.addNewPoint(nodes);
    else
        renderer.addNewPoint(nodes,sensor_model);
}

void MainWindow::on_actionClear_Points_triggered()
{
    nodes.clear();
    ui->listWidget->clear();

    if(USE_VTK_RENDER)
        renderer_vtk.deletePoints();
    else
         renderer.deletePoints();
}

void MainWindow::on_actiontr_Exit_triggered()
{
    MainWindow::close();
}

void MainWindow::on_actionDefault_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeBackgroundColor(82,87,110);
    else
        renderer.changeBackgroundColor(QColor(82,87,110));
}

void MainWindow::on_actionBlack_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeBackgroundColor(10,10,10);
    else
        renderer.changeBackgroundColor(Qt::black);
}

void MainWindow::on_actionGray_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeBackgroundColor(127,127,127);
    else
        renderer.changeBackgroundColor(Qt::gray);
}

void MainWindow::on_actionWhite_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeBackgroundColor(255,255,255);
    else
        renderer.changeBackgroundColor(Qt::white);
}

void MainWindow::on_actionPointsWindow_triggered()
{
    visibilityListPoints = !visibilityListPoints;
    ui->listWidget->setVisible(visibilityListPoints);
    ui->label_lW->setVisible(visibilityListPoints);

}


void MainWindow::on_actionVisualization_Guide_triggered()
{
    wdg_guideVisualization->show();
}



void MainWindow::on_actionInsert_Defect_Guide_triggered()
{
    wdg_helpID->show();
}


QVector<float> fromString(QString &str)
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

void MainWindow::on_actionGet_Trajectory_from_triggered()
{

        QVector<QVector3Dd> pos_dataTraj;
        QVector<QVector3Dd> rpy_dataTraj;
        bool readFile = false;
        QFile file;
        QFileInfo file_info;
        QString path;
        QDomDocument xmlBOM;
        QString suffix;

        bool executeLiteral = false;
        bool splitFiles = false;

        QString path_loadTraj = QFileDialog::getOpenFileName(this, "Load Trajectory", QDir::homePath(),"XML Files (*.xml *.si *.txt)");

        if (!path_loadTraj.isEmpty() && !path_loadTraj.isNull())
        {
            //ARREGLAR ESTOOOO
         //   wTrajectoryNodes->show();
            //---------------------------------------------------------------------------

            QMessageBox::information(this, "INFO", "Trajectory loaded. Select save directory");
            path = QFileDialog::getExistingDirectory(this, "QFileDialog.getSaveDirectory", "");

            if (!path.isEmpty() && !path.isNull())
            {

                //Leer del archivo las posiciones, cómo guardar la trayectoria
                QFile file(path_loadTraj);
                QFileInfo file_info(path_loadTraj);
                suffix = file_info.suffix();


                if (!file.open(QIODevice::ReadOnly ))
                {
                       // Error while loading file
                    std::cout <<"ERROR" << std::endl;

                }
                else
                {

                    readFile = true;
                    xmlBOM.setContent(&file);

                }

            }
        }

        std::cout <<"probando" << std::endl;

        if (readFile && suffix=="xml")
        {
            // Extract the root markup
            QDomElement root = xmlBOM.documentElement();
            QString type = root.tagName();


            QDomElement Component=root.firstChild().toElement();
            while(!Component.isNull())
            {
                if (Component.tagName()=="POSITION")
                {
                    QDomElement Component2=Component.firstChild().toElement();
                    while (Component2.tagName()=="XYZ")
                    {
                        QString values =Component2.firstChild().toText().data();
                        QVector<float> v_f = fromString(values);

                        QVector3Dd posxyz_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
                        pos_dataTraj.push_back(posxyz_);
                        Component2 = Component2.nextSibling().toElement();

                    }

                }


                if (Component.tagName()=="RPYdata")
                {
                    QDomElement Component2=Component.firstChild().toElement();
                    while (Component2.tagName()=="RPY")
                    {
                        QString values =Component2.firstChild().toText().data();
                        QVector<float> v_f = fromString(values);

                        QVector3Dd rpy_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
                        rpy_dataTraj.push_back(rpy_);
                        Component2 = Component2.nextSibling().toElement();

                    }

                }

                Component = Component.nextSibling().toElement();

            }


             int frames = ui->fpsSpinBox->value();
             double fov = ui->fovSpinBox->value();
             double resolution = ui->resolutionSpinBox->value();
             double w_range = ui->workingRangeSpinBox->value();
             double w_distance = ui->workingDistanceSpinBox->value();
             double vel = ui->velSpinBox->value();
             double uncertainty = ui->uncertaintySpinBox->value();

             ui->progressBar->show();
             ui->button_stop->show();

             if(USE_VTK_RENDER)
                 renderer_vtk.drawTraj(pos_dataTraj);
             else
                renderer.insertTraj(pos_dataTraj);

             if(executeLiteral)
                emit button_traj_node_from_clicked(pos_dataTraj, rpy_dataTraj, fov, resolution, w_range, w_distance, uncertainty, tree, path);

            else
             {

                 QVector<trajectoryNode> nodes_load;
                 for(int id=0; id<pos_dataTraj.size();id++)
                 {

                     QQuaternion q = QQuaternion::fromEulerAngles(rpy_dataTraj[id].y(),rpy_dataTraj[id].z(),rpy_dataTraj[id].x());
                     trajectoryNode node_aux(pos_dataTraj[id],q);
                     nodes_load.push_back(node_aux);
                 }

                 emit button_traj_node_clicked(nodes_load, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, path, false);
             }
        }
        else if (readFile && suffix=="si")
        {
            //int a = 0;
            TrajectoryController trajController;
            trajController.LoadTrajectory(path_loadTraj);

            //int lhgl=0;
        }
        else if (readFile && suffix=="txt")
        {
            QVector<trajectoryNode> nodes_load;
            QQuaternion q = sensor_model.q;

            // Extract the root markup
            QDomElement root = xmlBOM.documentElement();

            QDomElement Component=root.firstChild().toElement();
            while(!Component.isNull())
            {

                if (Component.tagName()=="From" || Component.tagName()=="To")
                {
                    QString values =Component.firstChild().toText().data();
                    QVector<float> v_f = fromString(values);

                    QVector3Dd posxyz_ = QVector3Dd(v_f[0], v_f[1], v_f[2]);
                    pos_dataTraj.push_back(posxyz_);

                    trajectoryNode node_aux(posxyz_,q);
                    nodes_load.push_back(node_aux);

                }

                Component = Component.nextSibling().toElement();

            }


             int frames = ui->fpsSpinBox->value();
             double fov = ui->fovSpinBox->value();
             double resolution = ui->resolutionSpinBox->value();
             double w_range = ui->workingRangeSpinBox->value();
             double w_distance = ui->workingDistanceSpinBox->value();
             double vel = ui->velSpinBox->value();
             double uncertainty = ui->uncertaintySpinBox->value();

             ui->progressBar->show();
             ui->button_stop->show();

             on_actionClear_Points_triggered();
             if(USE_VTK_RENDER)
                 renderer_vtk.drawTraj(pos_dataTraj);

             else
                 renderer.insertTraj(pos_dataTraj);

             emit button_traj_node_clicked(nodes_load, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, path, false);


        }

    file.close();
}



void MainWindow::on_actionTrajectory_Generator_triggered()
{
    // Select plane and axis

    QMessageBox *mbTrajGen = new QMessageBox();
    mbTrajGen->layout()->addWidget(wPlaneSelection);
    mbTrajGen->layout()->addWidget(wScanAll);
    mbTrajGen->addButton(QMessageBox::Ok);
    mbTrajGen->addButton(QMessageBox::Cancel);
    auto res = mbTrajGen->exec();


    bool XY_x=false, XY_y=false, XZ_x=false, XZ_z=false, YZ_y=false, YZ_z=false;

    if (res == QMessageBox::Ok)
    {

        if (b_planeXY->isChecked())
        {
            if (b_axisX->isChecked()) XY_x = true;
            else if (b_axisY->isChecked()) XY_y = true;
            else
            {
                QMessageBox::warning(this, "ERROR", "Select axis X or Y");
                return;
            }
        }
        else if (b_planeXZ->isChecked())
        {
            if (b_axisX->isChecked()) XZ_x = true;
            else if (b_axisZ->isChecked()) XZ_z = true;
            else
            {
                QMessageBox::warning(this, "ERROR", "Select axis X or Z");
                return;
            }
        }
        else if (b_planeYZ->isChecked())
        {
            if (b_axisY->isChecked())YZ_y = true;
            else if (b_axisZ->isChecked())YZ_z = true;
            else
            {
                QMessageBox::warning(this, "ERROR", "Select axis Y or Z");
                return;
            }
        }
        else
        {
            QMessageBox::warning(this, "ERROR", "Select one plane");
            return;
        }


        // Define trajectories according to plane and axis

        QVector<trajectoryNode> nodes_g;

        auto bbox_general = tree.bbox;
        auto dim = bbox_general.maxPos - bbox_general.minPos;
        double X_FOV = sensor_model.x_fov;
        double dist_solape = X_FOV/4;
        double width_sensor = X_FOV - dist_solape;
        int n_traj;

        QVector3D normal_plane;

        if (XZ_x || XZ_z)
        {
            normal_plane = QVector3D(0,1,0);
//            double y_pos = (width_sensor/2)/tan(sensor_model.FOV/2);
            double y_pos = sensor_model.working_distance;

            if (XZ_z)
            {
                n_traj = ceil(abs(dim.x())/(width_sensor));
                std::cout << "Dims: " << dim.x() << std::endl;
                std::cout << "n_traj: " << n_traj << std::endl;
                std::cout << "width_sensor: " << width_sensor << std::endl;


                for (int i=0; i<n_traj; i++)
                {
                    double pos_x = ((bbox_general.minPos.x()+width_sensor/2)+i*width_sensor);

//                    QVector3Dd pos_ini(pos_x, bbox_general.maxPos.y()+y_pos, bbox_general.minPos.z()-10);
//                    QVector3Dd pos_end(pos_x, bbox_general.maxPos.y()+y_pos, bbox_general.maxPos.z()+10);
                    QVector3Dd pos_ini(pos_x, bbox_general.minPos.y()+y_pos, bbox_general.minPos.z()-10);
                    QVector3Dd pos_end(pos_x, bbox_general.minPos.y()+y_pos, bbox_general.maxPos.z()+10);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }
            }
            else if (XZ_x)
            {
                n_traj = ceil(abs(dim.z())/(width_sensor)); //ESTE EN EL EJE X
                for (int i=0; i<n_traj; i++)
                {
                    double pos_z = ((bbox_general.minPos.z()+width_sensor/2)+i*width_sensor);

                    QVector3Dd pos_ini(bbox_general.minPos.x()-10, bbox_general.maxPos.y()+y_pos, pos_z);
                    QVector3Dd pos_end(bbox_general.maxPos.x()+10, bbox_general.maxPos.y()+y_pos, pos_z);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }
            }
        }

        else if (XY_x || XY_y)
        {
            normal_plane = QVector3D(0,0,1);
            double z_pos = (width_sensor/2)/tan(sensor_model.FOV/2);
            if (XY_x)
            {
                n_traj = ceil(abs(dim.y())/(width_sensor)); //ESTE EN EL EJE Z
                for (int i=0; i<n_traj; i++)
                {
                    double pos_y = ((bbox_general.minPos.y()+width_sensor/2)+i*width_sensor);

                    QVector3Dd pos_ini(bbox_general.minPos.x()-10, pos_y, bbox_general.maxPos.z()+z_pos); //maxPos, minPos ¿? Todo esto puede dar problemas en negativo :(
                    QVector3Dd pos_end(bbox_general.maxPos.x()+10, pos_y, bbox_general.maxPos.z()+z_pos);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }
            }

            if (XY_y)
            {
                n_traj = ceil(abs(dim.x())/(width_sensor)); //ESTE EN EL EJE Z
                for (int i=0; i<n_traj; i++)
                {
                    double pos_x = ((bbox_general.minPos.y()+width_sensor/2)+i*width_sensor);

                    QVector3Dd pos_ini(pos_x, bbox_general.minPos.y()-10, bbox_general.maxPos.z()+z_pos);
                    QVector3Dd pos_end(pos_x, bbox_general.maxPos.y()+10, bbox_general.maxPos.z()+z_pos);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }

            }

        }

        else if (YZ_y || YZ_z)
        {
            normal_plane = QVector3D(1,0,0);
            double x_pos = (width_sensor/2)/tan(sensor_model.FOV/2);
            if (YZ_y)
            {
                n_traj = ceil(abs(dim.z())/(width_sensor)); //ESTE EN EL EJE Z
                for (int i=0; i<n_traj; i++)
                {
                    double pos_z = ((bbox_general.minPos.y()+width_sensor/2)+i*width_sensor);

                    QVector3Dd pos_ini(bbox_general.maxPos.x()+x_pos, bbox_general.minPos.y()-10, pos_z);
                    QVector3Dd pos_end(bbox_general.maxPos.x()+x_pos, bbox_general.maxPos.y()+10, pos_z);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }
            }

            if (YZ_z)
            {
                n_traj = ceil(abs(dim.y())/(width_sensor)); //ESTE EN EL EJE Z
                for (int i=0; i<n_traj; i++)
                {
                    double pos_y = ((bbox_general.minPos.y()+width_sensor/2)+i*width_sensor);

                    QVector3Dd pos_ini(bbox_general.maxPos.x()+x_pos, pos_y,  bbox_general.minPos.z()-10);
                    QVector3Dd pos_end(bbox_general.maxPos.x()+x_pos, pos_y,  bbox_general.maxPos.z()+10);

                    QQuaternion q = QQuaternion::fromEulerAngles(ui->pitchSpinBox->value(), ui->yawSpinBox->value(),ui->rollSpinBox->value());

                    trajectoryNode node_aux1(pos_ini,q);
                    trajectoryNode node_aux2(pos_end,q);

                    nodes_g.push_back(node_aux1);
                    nodes_g.push_back(node_aux2);

                }
            }
        }

       //---



        GenTraj_options opt;
        if (b_opt1->isChecked())
            opt=optMinRange;
        else if (b_opt2->isChecked())
            opt=optWDist;
        else
            opt=optMaxRange;


        QString path = QFileDialog::getExistingDirectory(this, "QFileDialog.getSaveDirectory", "");
        if (!path.isEmpty() && !path.isNull())
        {
            int frames = ui->fpsSpinBox->value();
            double fov = ui->fovSpinBox->value();
            double resolution = ui->resolutionSpinBox->value();
            double w_range = ui->workingRangeSpinBox->value();
            double w_distance = ui->workingDistanceSpinBox->value();
            double vel = ui->velSpinBox->value();
            double uncertainty = ui->uncertaintySpinBox->value();

            ui->progressBar->show();
            ui->button_stop->show();

            emit button_traj_generator_clicked(opt,nodes_g, normal_plane, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, path);
        }

    }

}


//Arreglar lectura trayectoriasssss


//Action for new toolbar
void MainWindow::on_actionReset_View_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.resetView();
}

void MainWindow::on_actionSetViewX_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesYZ();
}

void MainWindow::on_actionSetViewX_2_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesYZ_();
}

void MainWindow::on_actionSetViewY_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesXY_();
}

void MainWindow::on_actionSetViewY_2_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesXY();
}

void MainWindow::on_actionSetViewZ_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesXZ_();
}

void MainWindow::on_actionSetViewZ_2_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.changeViewAxesXZ();
}

void MainWindow::on_actionSetWireframeView_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.setWireframeView();
}

void MainWindow::on_actionSetSurfaceView_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.setSurfaceView();
}

void MainWindow::on_actionSetPointsView_triggered()
{
    if (USE_VTK_RENDER)
        renderer_vtk.setPointsView();
}

void MainWindow::on_actionselectCells_triggered()
{
    std::cout << "SELECTION MODE" << std::endl;

    if (USE_VTK_RENDER)
        renderer_vtk.selectCellsOn();
}

void MainWindow::on_actionFrom_File_triggered()
{
    on_actionInsert_Defect_Selection_triggered();
}

void MainWindow::waitForDefectInsertion()
{
    // Indicar que comienza
    wdg_defect = new QWidget();
    wdg_defect->setFixedSize(500,200);
    wdg_defect->setWindowFlags(Qt::WindowMinimizeButtonHint);
    movie = new QMovie(":/MyRes/images/loading.gif");
    movie->setScaledSize(QSize(250,250));

    QLabel *textLabel = new QLabel("INSERTANDO DEFECTO", wdg_defect);
    textLabel->setAlignment(Qt::AlignCenter);
    QLabel *gifLabel = new QLabel("", wdg_defect);
    gifLabel->setAlignment(Qt::AlignCenter);
    gifLabel->setMovie(movie);
    //gifLabel->setVisible(true);
    movie->start();

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    QVBoxLayout *vbl = new QVBoxLayout(wdg_defect);
    vbl->addWidget(textLabel);
    vbl->addWidget(line);
    vbl->addWidget(gifLabel);
    wdg_defect->show();
}


void MainWindow::on_actionPredefined_defect_triggered()
{
    vtkSmartPointer<vtkPolyData> polySelection;
    if(renderer_vtk.getPolySelection(polySelection))
    {
        wdg_defect_selection->exec();
        if (wdg_defect_selection->isAccepted)
        {
            QMessageBox::information(this, "INFO", "Defect selected. Save new model");
            QString p = QDir::homePath() +"/nuevo.stl";
            QString name_saving_ = QFileDialog::getSaveFileName(this, "Guardar el nuevo modelo STL", p, "STL files (*.stl)");
            QString type = wdg_defect_selection->currentType();
            QString personalized_type = wdg_defect_selection->currentPersonalizedType();

            float deg_rot =  wdg_defect_selection->getDegRot();
            double depth = wdg_defect_selection->getDepth();

            crackDef_params *params;

            if (type == "Crack")
            {
                std::cout << type.toStdString() << std::endl;

                params = new crackDef_params {wdg_defect_selection->getLD(), wdg_defect_selection->getWD(),
                                              wdg_defect_selection->getWidthCrack(), wdg_defect_selection->getNPoints()};


            }



            if (!name_saving_.isEmpty() && !name_saving_.isNull())
            {
                emit button_defect_selection_predefined_clicked(this->name_file, name_saving_, polySelection, type, depth, deg_rot, personalized_type, params);
                waitForDefectInsertion();
            }
        }

    }

    //

    else
    {
        //Mensaje que te diga que tienes que seleccionar el área
        QMessageBox::warning(this, "Warning", "Select area to insert the defect.\nPress -R-, later click and drag with left mouse button");

    }


    wdg_defect_selection->isAccepted = false;


}

void MainWindow::on_button_stop_clicked()
{
    controller_.setCancelling(true);
    ui->progressBar->hide();
    ui->button_stop->hide();
}

void MainWindow::on_actionSetDragMode_triggered(bool checked)
{
    if (USE_VTK_RENDER)
    {
        renderer_vtk.changeStyleMode(checked);
    }


    emit button_dragMode_clicked(checked, renderer_vtk);

}

void MainWindow::updateUi_drag(double* pos, double* rpy)
{
    QVector3Dd o(pos[0],pos[1],pos[2]);
    double r = rpy[0], p=rpy[1], y=rpy[2];

    this->sensor_model.updatePositionSensor(o, this->sensor_model.roll,  this->sensor_model.pitch,  this->sensor_model.yaw);

    updateSpinBoxes();
}


void MainWindow::on_actionScan_all_piece_triggered()
{

    /*
     Opciones:
        1. Según número de pasadas queridas
            if(h_sensor < min_distante) h_sensor = min_distance
            if(h_sensor < max_distance) n_pasadas no suficientes
        2. Según distancia del sensor:
            2.1. Working range
            2.2. Min distance
            2.3. Max distance
    */

    QMessageBox *mbTrajGen = new QMessageBox();
    mbTrajGen->layout()->addWidget(wPlaneSelection);
    mbTrajGen->layout()->addWidget(wScanAll);
    mbTrajGen->addButton(QMessageBox::Ok);
    mbTrajGen->addButton(QMessageBox::Cancel);
    auto res = mbTrajGen->exec();

    if (res==QMessageBox::Cancel)
        return;


    int frames = ui->fpsSpinBox->value();
    double fov = ui->fovSpinBox->value();
    double resolution = ui->resolutionSpinBox->value();
    double w_range = ui->workingRangeSpinBox->value();
    double w_distance = ui->workingDistanceSpinBox->value();
    double vel = ui->velSpinBox->value();
    double uncertainty = ui->uncertaintySpinBox->value();

    auto bbox = tree.bbox;
    auto dim = bbox.maxPos - bbox.minPos;

    bool XY_x=false, XY_y=false, XZ_x=false, XZ_z=false, YZ_y=false, YZ_z=false;
    if (b_planeXY->isChecked())
    {
        if (b_axisX->isChecked()) XY_x = true;
        else if (b_axisY->isChecked()) XY_y = true;
        else
        {
            QMessageBox::warning(this, "ERROR", "Select axis X or Y");
            return;
        }
    }
    else if (b_planeXZ->isChecked())
    {
        if (b_axisX->isChecked()) XZ_x = true;
        else if (b_axisZ->isChecked()) XZ_z = true;
        else
        {
            QMessageBox::warning(this, "ERROR", "Select axis X or Z");
            return;
        }
    }
    else if (b_planeYZ->isChecked())
    {
        if (b_axisY->isChecked())YZ_y = true;
        else if (b_axisZ->isChecked())YZ_z = true;
        else
        {
            QMessageBox::warning(this, "ERROR", "Select axis Y or Z");
            return;
        }
    }
    else
    {
        QMessageBox::warning(this, "ERROR", "Select one plane");
        return;
    }

    QVector3D dir_normal, dir_scan, dir_step;
    if (XY_x){dir_normal=QVector3D(0,0,1);dir_scan=QVector3D(1,0,0);dir_step=QVector3D(0,1,0);}
    if (XY_y){dir_normal=QVector3D(0,0,1);dir_scan=QVector3D(0,1,0);dir_step=QVector3D(1,0,0);}
    if (XZ_x){dir_normal=QVector3D(0,1,0);dir_scan=QVector3D(1,0,0);dir_step=QVector3D(0,0,1);}
    if (XZ_z){dir_normal=QVector3D(0,1,0);dir_scan=QVector3D(0,0,1);dir_step=QVector3D(1,0,0);}
    if (YZ_y){dir_normal=QVector3D(1,0,0);dir_scan=QVector3D(0,1,0);dir_step=QVector3D(0,0,1);}
    if (YZ_z){dir_normal=QVector3D(1,0,0);dir_scan=QVector3D(0,0,1);dir_step=QVector3D(0,1,0);}

    float width_model = QVector3D::dotProduct(dir_step,dim.toQVector3D());

    QVector<trajectoryNode> nodes;
    float w0, h0, l0, l1, h_sensor;
    int n_steps, width_step;
    if(pageComboBox->currentIndex() == 0)
    {
        if (b_opt1->isChecked()) h_sensor= w_distance - w_range/2;
        else if (b_opt2->isChecked()) h_sensor= w_distance;
        else h_sensor= w_distance + w_range/2;

        h_sensor = 125; //Arreglar esto

        width_step = round(2*h_sensor*std::tan((fov*PI/180)/2));
        n_steps = std::ceil(width_model/width_step);
    }
    else
    {
        n_steps = steps_sb->value();
        width_step = width_model/n_steps;
        h_sensor = (width_step/2)/std::tan((fov*PI/180)/2);
    }

    w0 = QVector3D::dotProduct(dir_step,bbox.minPos.toQVector3D()) + width_step/2;
    h0 = QVector3D::dotProduct(dir_normal,bbox.maxPos.toQVector3D()) + h_sensor;
    l0 = QVector3D::dotProduct(dir_scan,bbox.minPos.toQVector3D())-4;
    l1 = QVector3D::dotProduct(dir_scan,bbox.maxPos.toQVector3D())+4;

    std::cout << "w_model: " << width_model << std::endl;
    std::cout << "w_step: " << width_step << std::endl;
    std::cout << "n_steps: " << n_steps << std::endl;
    std::cout << "h_sensor: " << h_sensor << std::endl;


    QQuaternion q;
    q = sensor_model.q;

    for (int i=0; i<n_steps; i++)
    {
        QVector3D pos0_aux, pos1_aux;

        pos0_aux = dir_normal*h0 + dir_scan*l0 + dir_step*(w0+width_step*i);
        pos1_aux = dir_normal*h0 + dir_scan*l1 + dir_step*(w0+width_step*i);

        QVector3Dd pos0(pos0_aux.x(), pos0_aux.y(), pos0_aux.z());
        QVector3Dd pos1(pos1_aux.x(), pos1_aux.y(), pos1_aux.z());

        trajectoryNode node_i0(pos0,q);
        trajectoryNode node_i1(pos1,q);

        nodes.push_back(node_i0);
        nodes.push_back(node_i1);
    }


    std::cout << nodes.size() << std::endl;
    for (int i=0; i< nodes.size(); i++)
        printQVector3D(nodes[i].pos().toQVector3D());

    ui->progressBar->show();
    ui->button_stop->show();

//    QString path_folder = QDir::homePath() + "/PROBATURAS";
//    if (QDir(path_folder).exists() == false)
//        QDir(QDir::homePath()).mkdir("PROBATURAS");
    emit button_traj_node_clicked(nodes, vel, frames, fov, resolution, w_range, w_distance, uncertainty, tree, "/home/sara/Escritorio/pruebas/", true);
    //ESTE ESTÁ EN PRUEBAS

}

void MainWindow::on_actionSmooth_stl_triggered()
{
//    qDebug("Smoothing...");
//    vtkSmartPointer<vtkPolyData> poly;
//    poly = vtkSmartPointer<vtkPolyData>::New();
//    vtkTools::smoothPolyData(this->reader->GetOutput(), poly);

//    qDebug("Changing model...");
//    QString name_aux("/tmp/aux.stl");
//    vtkTools::writeSTL(poly, const_cast<char*>(name_aux.toStdString().c_str()));
//    reader->SetFileName(name_aux.toStdString().c_str());
//    reader->Update();

//    this->controller_.updatePolydataModel(reader);
//    this->renderer_vtk.changeModel(reader);
//    qDebug("Changed...");

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


void MainWindow::on_actionRemote_conexion_triggered()
{
    ///LOAD PLUGINS----------------
    if (!loadPlugin("librosplugin.so")) {std::cout << "PLUGIN NOT LOADED" << std::endl;}
    else
    {
        //Arreglar esto
        QStringList arguments = qApp->arguments();
        QCoreApplication *app = QCoreApplication::instance();
        int argc = app->arguments().at(0).toInt();
        char **argv = new char*[1];
        argv[0]= "/home/sara/sararht/TESIS/Codigo/simulador/QT/build-simulador-Qt_5_14_2_gcc_64-Release/simulador";




//----------------------------------
        //Traj sensor
        int id_string = 0;

        QVector<QVector3D> pos_sensor;
        QVector<QVector3D> rpy_sensor;
        std::string path_global = "/home/sara/Descargas/penholder/int/";
      //  std::string path_global = "/home/sara/Descargas/puerta_pequeña/";

        //std::string path = "/home/sara/Descargas/prueba/traj_sensor0" + std::to_string(id_string) +".xml";
     //   std::string path = path_global + "traj_sensor0" + std::to_string(id_string) +".xml";
        std::string path = path_global + "step_00_traj_nueva.xml";

        QFile file(path.c_str());
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
                //QVector3D rpy_ = QVector3D(0,90,0);

                rpy_sensor.push_back(rpy_);
                Component2 = Component2.nextSibling().toElement();
            }

        }
 //----------------------------------





        pluginInterface->initPlugin(1,argv, pos_sensor, rpy_sensor);
        pluginInterface->precalculate();
        pluginInterface->calculate();
      //  pluginInterface->postcalculate();
     //   pluginInterface->run();
        //pluginInterface->start();

    }
}
