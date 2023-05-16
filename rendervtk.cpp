//classes includes
#include "rendervtk.h"

//vtk includes
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderView.h>
//#include <vtkSTLReader.h>
#include <vtkLineSource.h>
#include <vtkPointSource.h>


#if VTK_VERSION_NUMBER >= 89000000000ULL
#define VTK890 1
#endif

renderVTK::renderVTK(sensor sensor_model, vtkSmartPointer<vtkPolyData> poly)
{

   //Disable Warnings
   vtkObject::GlobalWarningDisplayOff();

   // Needed to ensure appropriate OpenGL context is created for VTK rendering.
   QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
   colors = vtkSmartPointer<vtkNamedColors>::New();
   widget = new QVTKOpenGLNativeWidget();
   widget->setEnableHiDPI(true);

   renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
   #if VTK890
     widget->setRenderWindow(renderWindow);
   #else
     widget->SetRenderWindow(renderWindow);
   #endif
    widget->resize(600, 600);

    //STL MODEL
    vtkNew<vtkPolyDataMapper> mapper_model;
    mapper_model->SetInputData(poly);
    actor_model = vtkSmartPointer<vtkActor>::New();
    actor_model->SetMapper(mapper_model);
    actor_model->SetPosition(0,0,0);

    //LASER
    laser = vtkSmartPointer<vtkConeSource>::New();
    float FOV_mm = sensor_model.range*tan(sensor_model.FOV/2);
    laser->SetRadius(FOV_mm);
    laser->SetHeight(sensor_model.range);
    laser->SetDirection(0,0,-1);
    laser->SetResolution(1);
    laser->Update();
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(laser->GetOutputPort());
    actor_laser = vtkSmartPointer<vtkActor>::New();
    actor_laser->SetMapper(mapper);
    actor_laser->GetProperty()->SetDiffuseColor(colors->GetColor3d("red").GetData());
    actor_laser->GetProperty()->SetOpacity(0.6);

    //SENSOR
    vtkNew<vtkPolyDataMapper> mapper_sensor_case;
    sensor_case = vtkSmartPointer<vtkCubeSource>::New();
    sensor_case->SetXLength(130);
    sensor_case->SetYLength(20);
    sensor_case->SetZLength(75);
    sensor_case->SetCenter(0,0,-20);
    sensor_case->Update();
    mapper_sensor_case->SetInputConnection(sensor_case->GetOutputPort());
    actor_sensor_case = vtkSmartPointer<vtkActor>::New();
    actor_sensor_case->SetMapper(mapper_sensor_case);
    actor_sensor_case->GetProperty()->SetDiffuseColor(colors->GetColor3d("gray").GetData());

    sensor_laser = vtkSmartPointer<vtkAssembly>::New();
    sensor_laser->AddPart(actor_laser);
    sensor_laser->AddPart(actor_sensor_case);

    //Renderer
    renderer =vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor_model);
    renderer->AddActor(sensor_laser);
    //renderer->AddActor(actor_laser);
    //renderer->AddActor(actor_sensor_case);

    renderer->SetBackground(82.0/255,87.0/255,110.0/255);

    #if VTK890
      widget->renderWindow()->AddRenderer(renderer);
      widget->renderWindow()->SetWindowName("RenderWindowNoUIFile");
    #else
      widget->GetRenderWindow()->AddRenderer(renderer);
      widget->GetRenderWindow()->SetWindowName("RenderWindowNoUIFile");
    #endif

    renderWindowInteractor = widget->GetRenderWindow()->GetInteractor();

    //AXES
    actor_axes = vtkSmartPointer<vtkAxesActor>::New();

    widget_axes = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget_axes->SetOrientationMarker(actor_axes);
    widget_axes->SetInteractor(renderWindowInteractor);
    widget_axes->SetEnabled(1);
    widget_axes->InteractiveOff();

    vtkNew<vtkAreaPicker> areaPicker;
    renderWindowInteractor->SetPicker(areaPicker);

    style = vtkSmartPointer<HighlightInteractorStyle>::New();
    style->SetDefaultRenderer(renderer);    
    style->SetPolyData(poly);

    renderWindowInteractor->SetInteractorStyle(style);
    renderWindowInteractor->Start();

    //Click and drag actor
    style_drag = vtkSmartPointer<DragInteractorStyle>::New(); //vtkInteractorStyleTrackballActor
    style_drag->SetActor(sensor_laser);

    renderer->ResetCamera();
    widget->show();

}

void renderVTK::changeBackgroundColor(double r, double g, double b)
{
    renderer->SetBackground(r/255.0,g/255.0,b/255.0);
    renderWindow->Render();
}

void renderVTK::resetView()
{
    renderer->ResetCamera();
    renderWindow->Render();
}

void renderVTK::changeViewAxesXY()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0], fp[1], fp[2] - dist);
    renderer->GetActiveCamera()->SetViewUp(0,1,0);
    renderWindow->Render();
}

void renderVTK::changeViewAxesXY_()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0], fp[1], fp[2] + dist);
    renderer->GetActiveCamera()->SetViewUp(0,1,0);
    renderWindow->Render();
}

void renderVTK::changeViewAxesXZ()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0], fp[1] - dist, fp[2]);
    renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);
    // redraw the scene
    renderWindow->Render();
}

void renderVTK::changeViewAxesXZ_()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0], fp[1] + dist, fp[2]);
    renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);
    // redraw the scene
    renderWindow->Render();
}

void renderVTK::changeViewAxesYZ()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0] - dist, fp[1], fp[2]);
    renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);
    // redraw the scene
    renderWindow->Render();
}

void renderVTK::changeViewAxesYZ_()
{
    renderer->ResetCamera();
    double *fp = renderer->GetActiveCamera()->GetFocalPoint();
    double *p = renderer->GetActiveCamera()->GetPosition();
    double dist
        = std::sqrt((p[0] - fp[0]) * (p[0] - fp[0]) + (p[1] - fp[1]) * (p[1] - fp[1])
                    + (p[2] - fp[2]) * (p[2] - fp[2]));
    renderer->GetActiveCamera()->SetPosition(fp[0] + dist, fp[1], fp[2]);
    renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);
    // redraw the scene
    renderWindow->Render();
}

void renderVTK::updateRendered(sensor sensor_model, bool isInTraj)
{

    //LASER

    float x = sensor_model.range*tan(sensor_model.FOV/2);

    laser->SetRadius(x);
    laser->SetHeight(sensor_model.range);
    laser->SetCenter(0,0,sensor_model.range/2);
    laser->Update();
    laser->Modified();

    //Orientación

    double* angles_ant = sensor_laser->GetOrientation();

    QVector3D t_final = sensor_model.origin.toQVector3D();

    double n_roll = sensor_model.roll; while(n_roll>180)n_roll=n_roll-360; while(n_roll<-180)n_roll=n_roll+360;
    double n_pitch = sensor_model.pitch; while(n_pitch>180)n_pitch=n_pitch-360; while(n_pitch<-180)n_pitch=n_pitch+360;
    double n_yaw = sensor_model.yaw; while(n_yaw>180)n_yaw=n_yaw-360; while(n_yaw<-180)n_yaw=n_yaw+360;

    //ASSEMBLY
    sensor_laser->SetPosition(t_final.x(),t_final.y(),t_final.z());
    if (!isInTraj)
    {
        sensor_laser->SetOrientation(0,0,0);
        sensor_laser->RotateZ(90);
        sensor_laser->RotateWXYZ(n_pitch,1,0,0);
        sensor_laser->RotateWXYZ(n_yaw,0,1,0);
        sensor_laser->RotateWXYZ(n_roll,0,0,1);
    }


    sensor_laser->Modified();
    renderWindow->Render();
}

void renderVTK::changeModel(vtkSmartPointer<vtkPolyData> poly)
{
    style->SetPolyData(poly);

    vtkNew<vtkPolyDataMapper> mapper_model;
    mapper_model->SetInputData(poly);
    actor_model->SetMapper(mapper_model);
    actor_model->SetPosition(0,0,0);
    actor_model->Modified();

    renderer->ResetCamera();
    renderWindow->Render();
}

vtkNew<vtkActor> renderVTK::drawLine(const QVector3D& start, const QVector3D& end)
{
    vtkNew<vtkLineSource> line;
    line->SetPoint1(start.x(), start.y(), start.z());
    line->SetPoint2(end.x(), end.y(), end.z());

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(line->GetOutputPort());
    vtkNew<vtkActor> actor_line_i;
    actor_line_i->SetMapper(mapper);
    actor_line_i->GetProperty()->SetDiffuseColor(colors->GetColor3d("black").GetData());
    actor_line_i->GetProperty()->SetOpacity(0.3);

    return actor_line_i;
}

void renderVTK::addNewPoint(QVector<trajectoryNode> node)
{

    //PROBLEMA CON LA ORIENTACIÓNNNN

    double *pos=sensor_laser->GetPosition();
    double *ori=sensor_laser->GetOrientation();

 //   double *pos=actor_laser->GetPosition();
 //   double *ori=actor_laser->GetOrientation();
    //LASER
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(laser->GetOutputPort());
    vtkNew<vtkActor> actor_laser_i;
    actor_laser_i->SetMapper(mapper);
    actor_laser_i->GetProperty()->SetDiffuseColor(colors->GetColor3d("red").GetData());
    actor_laser_i->GetProperty()->SetOpacity(0.3);
    actor_laser_i->SetPosition(pos);
    actor_laser_i->SetOrientation(ori);

    //SENSOR
    vtkNew<vtkPolyDataMapper> mapper_sensor_case;
    mapper_sensor_case->SetInputConnection(sensor_case->GetOutputPort());
    vtkNew<vtkActor> actor_sensor_case_i;
    actor_sensor_case_i->SetMapper(mapper_sensor_case);
    actor_sensor_case_i->GetProperty()->SetDiffuseColor(colors->GetColor3d("gray").GetData());
    actor_sensor_case_i->GetProperty()->SetOpacity(0.3);
    actor_sensor_case_i->SetPosition(pos);
    actor_sensor_case_i->SetOrientation(ori);

    lasers.push_back(actor_laser_i);
    sensors.push_back(actor_sensor_case_i);

    renderer->AddActor(actor_laser_i);
    renderer->AddActor(actor_sensor_case_i);

    //LINE
    if (lasers.size()>=2)
    {
        vtkNew<vtkActor> line_i = drawLine(node[node.size()-2].pos().toQVector3D(), node[node.size()-1].pos().toQVector3D());
        lines.push_back(line_i);
        renderer->AddActor(line_i);
    }
    renderWindow->Render();

}

void renderVTK::deletePoints()
{
    for (int i=0; i<lasers.size();i++)
    {
        renderer->RemoveActor(lasers[i]);
        renderer->RemoveActor(sensors[i]);

        if (i<lasers.size()-1)
            renderer->RemoveActor(lines[i]);
    }

    renderWindow->Render();

    lasers.clear();
    sensors.clear();
    lines.clear();
}

bool renderVTK::getPolySelection(vtkSmartPointer<vtkPolyData> &selection)
{

    selection = style->GetSelection();
    if(selection->GetNumberOfPoints()==0)
        return false;

    return true;

}

bool renderVTK::getDragActor(double* &pos, double* &rpy)
{
     bool a = style_drag->GetCoords(pos,rpy);

     return a;
}

bool renderVTK::getSignalDragActor()
{
     return style_drag->getSignalUpdate();
}

void renderVTK::setSignalActor(bool signal)
{
     style_drag->setSignalActor(signal);
}


void renderVTK::setWireframeView()
{
      actor_model->GetProperty()->SetRepresentationToWireframe();
      actor_model->Modified();

      renderWindow->Render();
}

void renderVTK::setSurfaceView()
{
      actor_model->GetProperty()->SetRepresentationToSurface();
      actor_model->Modified();

      renderWindow->Render();
}

void renderVTK::setPointsView()
{
      actor_model->GetProperty()->SetRepresentationToPoints();
      actor_model->Modified();

      renderWindow->Render();
}

void renderVTK::drawTraj(QVector<QVector3Dd>pos)
{
    vtkSmartPointer<vtkPolyData> pointsSource = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    vtkIdType pid[pos.size()];
    for (int i=0; i<pos.size();i++)
    {
        pid[i]=points->InsertNextPoint(pos[i].x(),pos[i].y(),pos[i].z());
        vertices->InsertNextCell(i,pid);
    }
    pointsSource->SetPoints(points);
    pointsSource->SetVerts(vertices);
    pointsSource->Modified();

    // Create a mapper and actor
     vtkNew<vtkPolyDataMapper> mapper;
     mapper->SetInputData(pointsSource);
     mapper->Update();
     actor_traj = vtkSmartPointer<vtkActor>::New();
     actor_traj->SetMapper(mapper);
     actor_traj->GetProperty()->SetColor(colors->GetColor3d("white").GetData());

     double* colorB = renderer->GetBackground();
     if(colorB[0]==1.0 && colorB[1]==1.0 && colorB[2]==1.0)
        actor_traj->GetProperty()->SetColor(colors->GetColor3d("black").GetData()); //NO FUNCIONA

     renderer->AddActor(actor_traj);
     renderWindow->Render();
}

void renderVTK::deleteTraj()
{
    renderer->RemoveActor(actor_traj);
    renderWindow->Render();

}

void renderVTK::selectCellsOn()
{
    if (isDragModeOn)
    {
        std::cout << "Disable drag mode first." <<std::endl;

    }

    else
    {
        renderWindowInteractor->SetInteractorStyle(style);
        renderWindowInteractor->Start();

        style->changeMode();
    }


}

void renderVTK::changeStyleMode(bool isDragMode)
{
    isDragModeOn = isDragMode;

    if (isDragMode)
    {
        //style_switch->SetCurrentStyleToTrackballActor();
        renderWindowInteractor->SetInteractorStyle(style_drag);
        style_drag->changeMode(true);

    }

    else
    {
        style_drag->changeMode(false);
        renderWindowInteractor->SetInteractorStyle(style);
    }

    renderWindowInteractor->Start();

}
