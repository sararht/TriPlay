#ifndef MOUSEINTERACTORSTYLE_H
#define MOUSEINTERACTORSTYLE_H

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkDataSetMapper.h>
#include <vtkExtractSelection.h>
#include <vtkIdTypeArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkAreaPicker.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkImplicitFunction.h>
#include <vtkPlanes.h>
#include <vtkHardwareSelector.h>
#include <vtkGeometryFilter.h>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

namespace {
// Define interaction style
class HighlightInteractorStyle : public vtkInteractorStyleRubberBandPick
{
public:
  static HighlightInteractorStyle* New();
  vtkTypeMacro(HighlightInteractorStyle, vtkInteractorStyleRubberBandPick)

  HighlightInteractorStyle() : vtkInteractorStyleRubberBandPick()
  {
    this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    this->SelectedActor = vtkSmartPointer<vtkActor>::New();
    this->SelectedActor->SetMapper(SelectedMapper);
    this->PolySelection = vtkSmartPointer<vtkPolyData>::New();
  }


  virtual void changeMode()
  {
      this->CurrentMode=VTKISRBP_SELECT;
      this->GetInteractor()->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_CROSSHAIR);
  }
  virtual void OnLeftButtonUp() override
  {
    // Forward events
    vtkInteractorStyleRubberBandPick::OnLeftButtonUp();
    //Clear previous selection
    PolySelection = vtkSmartPointer<vtkPolyData>::New();

    this->GetInteractor()
        ->GetRenderWindow()
        ->GetRenderers()
        ->GetFirstRenderer()
        ->RemoveActor(SelectedActor);

    if (this->CurrentMode == VTKISRBP_SELECT)
    {
      vtkNew<vtkNamedColors> colors;

      vtkPlanes* frustum =
          static_cast<vtkAreaPicker*>(this->GetInteractor()->GetPicker())
              ->GetFrustum();


      vtkNew<vtkHardwareSelector> selector;
      selector->SetRenderer(this->Interactor->GetRenderWindow()
                                     ->GetRenderers()
                                     ->GetFirstRenderer());

     //Encontrar xmax,xmin,ymax,ymin
     int xmax = this->EndPosition[0];
     int xmin = this->StartPosition[0];
     int ymax = this->EndPosition[1];
     int ymin = this->StartPosition[1];
     if(this->StartPosition[0] > this->EndPosition[0])
     {
         xmax = this->StartPosition[0];
         xmin = this->EndPosition[0];
     }
     if(this->StartPosition[1] > this->EndPosition[1])
     {
         ymax = this->StartPosition[1];
         ymin = this->EndPosition[1];
     }
     selector->SetArea(xmin,ymin,xmax,ymax);//this->StartPosition[0],this->EndPosition[1],this->EndPosition[0],this->StartPosition[1]
     selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_CELLS);

     std::cout << this->StartPosition[0] <<", "<< this->StartPosition[1]<<", "<<this->EndPosition[0]<<", "<<this->EndPosition[1]<<std::endl;

     vtkSelection* selection = selector->Select();
     std::cout << "Selection has " << selection->GetNumberOfNodes() << " nodes." << std::endl;

     if (selection->GetNumberOfNodes()>0)
     {
         vtkNew<vtkExtractSelection> extractSelection;
         extractSelection->SetInputData(0, this->PolyData);
         extractSelection->SetInputData(1, selection);
         extractSelection->Update();
         this->SelectedMapper->SetInputConnection(extractSelection->GetOutputPort());
         this->SelectedMapper->ScalarVisibilityOff();

         this->SelectedActor->GetProperty()->SetColor(
             colors->GetColor3d("Blue").GetData());
         this->SelectedActor->GetProperty()->SetPointSize(15);
         this->SelectedActor->GetProperty()->SetRepresentationToWireframe();

         this->GetInteractor()
             ->GetRenderWindow()
             ->GetRenderers()
             ->GetFirstRenderer()
             ->AddActor(SelectedActor);
         this->GetInteractor()->GetRenderWindow()->Render();
         this->HighlightProp(NULL);

         vtkNew<vtkUnstructuredGrid> selected;
         selected->ShallowCopy(extractSelection->GetOutput());
         vtkSmartPointer<vtkGeometryFilter> geometryFilter = vtkSmartPointer<vtkGeometryFilter>::New();
         geometryFilter->SetInputData(selected);
         geometryFilter->Update();
         PolySelection = geometryFilter->GetOutput();

         std::cout << "Selection has " << PolySelection->GetNumberOfPoints() << " points." << std::endl;


     }

    }

    this->CurrentMode=VTKISRBP_ORIENT;
    this->GetInteractor()->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_ARROW);


  }

  void SetPolyData(vtkSmartPointer<vtkPolyData> polyData)
  {
    this->PolyData = polyData;
  }

  vtkSmartPointer<vtkPolyData> GetSelection()
  {
      return this->PolySelection;
  }
private:
  vtkSmartPointer<vtkPolyData> PolyData;
  vtkSmartPointer<vtkPolyData> PolySelection;
  vtkSmartPointer<vtkActor> SelectedActor;
  vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
};
vtkStandardNewMacro(HighlightInteractorStyle)

vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName);
} // namespace





















/*
namespace {

// Define interaction style
class customMouseInteractorStyle : public vtkInteractorStyleRubberBandPick
{
public:
  static customMouseInteractorStyle* New();
  vtkTypeMacro(customMouseInteractorStyle, vtkInteractorStyleRubberBandPick)

//  virtual void OnLeftButtonDown() override
//  {
//    std::cout << "Pressed left mouse button." << std::endl;
//    // Forward events
//    vtkInteractorStyleRubberBandPick::OnLeftButtonDown();
//  }

//  virtual void OnMiddleButtonDown() override
//  {
//    std::cout << "Pressed middle mouse button." << std::endl;
//    // Forward events
//    vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
//  }

//  virtual void OnRightButtonDown() override
//  {
//    std::cout << "Pressed right mouse button." << std::endl;
//    // Forward events
//    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
//  }


  virtual void OnLeftButtonUp() override
  {
      // Forward events
         vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

         if (this->CurrentMode == VTKISRBP_SELECT)
         {
           vtkNew<vtkNamedColors> colors;

           vtkPlanes* frustum =
               static_cast<vtkAreaPicker*>(this->GetInteractor()->GetPicker())
                   ->GetFrustum();

           vtkNew<vtkExtractPolyDataGeometry> extractPolyDataGeometry;
           extractPolyDataGeometry->SetInputData(this->PolyData);
           extractPolyDataGeometry->SetImplicitFunction(frustum);
           extractPolyDataGeometry->Update();

           std::cout << "Extracted "
                     << extractPolyDataGeometry->GetOutput()->GetNumberOfCells()
                     << " cells." << std::endl;
           this->SelectedMapper->SetInputData(extractPolyDataGeometry->GetOutput());
           this->SelectedMapper->ScalarVisibilityOff();

           //        vtkIdTypeArray* ids =
           //        dynamic_cast<vtkIdTypeArray*>(selected->GetPointData()->GetArray("OriginalIds"));

           this->SelectedActor->GetProperty()->SetColor(
               colors->GetColor3d("Tomato").GetData());
           this->SelectedActor->GetProperty()->SetPointSize(5);
           this->SelectedActor->GetProperty()->SetRepresentationToWireframe();

           this->GetInteractor()
               ->GetRenderWindow()
               ->GetRenderers()
               ->GetFirstRenderer()
               ->AddActor(SelectedActor);
           this->GetInteractor()->GetRenderWindow()->Render();
           this->HighlightProp(NULL);
         }
  }

  void SetPolyData(vtkSmartPointer<vtkPolyData> polyData)
   {
     this->PolyData = polyData;
   }
};

vtkStandardNewMacro(customMouseInteractorStyle)

} // namespace
*/
/*
namespace {

// Catch mouse events
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static MouseInteractorStyle* New();

  MouseInteractorStyle()
  {
    selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    selectedActor = vtkSmartPointer<vtkActor>::New();
  }

  virtual void OnLeftButtonDown() override
  {
    vtkNew<vtkNamedColors> colors;

    // Get the location of the click (in window coordinates)
    int* pos = this->GetInteractor()->GetEventPosition();

    vtkNew<vtkCellPicker> picker;
    picker->SetTolerance(0.0005);

    // Pick from this location.
    picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

    double* worldPosition = picker->GetPickPosition();
    std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

    if (picker->GetCellId() != -1)
    {

      std::cout << "Pick position is: " << worldPosition[0] << " "
                << worldPosition[1] << " " << worldPosition[2] << endl;

      vtkNew<vtkIdTypeArray> ids;
      ids->SetNumberOfComponents(1);
      ids->InsertNextValue(picker->GetCellId());

      vtkNew<vtkSelectionNode> selectionNode;
      selectionNode->SetFieldType(vtkSelectionNode::CELL);
      selectionNode->SetContentType(vtkSelectionNode::INDICES);
      selectionNode->SetSelectionList(ids);

      vtkNew<vtkSelection> selection;
      selection->AddNode(selectionNode);

      vtkNew<vtkExtractSelection> extractSelection;
      extractSelection->SetInputData(0, this->Data);
      extractSelection->SetInputData(1, selection);
      extractSelection->Update();

      // In selection
      vtkNew<vtkUnstructuredGrid> selected;
      selected->ShallowCopy(extractSelection->GetOutput());

      std::cout << "There are " << selected->GetNumberOfPoints()
                << " points in the selection." << std::endl;
      std::cout << "There are " << selected->GetNumberOfCells()
                << " cells in the selection." << std::endl;
      selectedMapper->SetInputData(selected);
      selectedActor->SetMapper(selectedMapper);
      selectedActor->GetProperty()->EdgeVisibilityOn();
      selectedActor->GetProperty()->SetColor(
          colors->GetColor3d("Pink").GetData());

      selectedActor->GetProperty()->SetLineWidth(3);

      this->Interactor->GetRenderWindow()
          ->GetRenderers()
          ->GetFirstRenderer()
          ->AddActor(selectedActor);
    }
    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  vtkSmartPointer<vtkPolyData> Data;
  vtkSmartPointer<vtkDataSetMapper> selectedMapper;
  vtkSmartPointer<vtkActor> selectedActor;
};

vtkStandardNewMacro(MouseInteractorStyle)

} // namespace
*/
#endif // MOUSEINTERACTORSTYLE_H
