#ifndef DRAGINTERACTORSTYLE_H
#define DRAGINTERACTORSTYLE_H

#include <vtkActor.h>
#include <vtkDataSetMapper.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkPropCollection.h>
#include <vtkAssembly.h>
#include <vtkInteractorStyleTrackballActor.h>


namespace {
// Define interaction style
class DragInteractorStyle : public vtkInteractorStyleTrackballActor
{
public:
  static DragInteractorStyle* New();
  vtkTypeMacro(DragInteractorStyle, vtkInteractorStyleTrackballActor)

  DragInteractorStyle() : vtkInteractorStyleTrackballActor()
  {
    this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    this->SelectedActor = vtkSmartPointer<vtkAssembly>::New();
  //  this->SelectedActor->SetMapper(SelectedMapper);
   }

  virtual void changeMode(bool signal)
  {
      if(signal)
        this->GetInteractor()->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_HAND);
      else
        this->GetInteractor()->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_ARROW);

  }
  virtual void OnLeftButtonUp() override
  {
      std::cout << "LEFT UP" << std::endl;
      vtkInteractorStyleTrackballActor::OnMiddleButtonUp();

      this->pos = SelectedActor->GetPosition();
      this->rpy = SelectedActor->GetOrientation();
      this->signal_update = true;

  }
  virtual void OnLeftButtonDown() override
  {
      this->signal_update = false;
      std::cout << "LEFT DOWN" << std::endl;

      vtkInteractorStyleTrackballActor::OnMiddleButtonDown();

      if (this->InteractionProp == this->SelectedActor)
      {
           std::cout << "Picked actor." << std::endl;
           this->pos = SelectedActor->GetPosition();
           this->rpy = SelectedActor->GetOrientation();
           vtkInteractorStyleTrackballActor::OnMiddleButtonDown();
      }

      else
          vtkInteractorStyleTrackballActor::OnMiddleButtonUp();

  }

  virtual void OnMouseMove() override
  {
      vtkInteractorStyleTrackballActor::OnMouseMove();
//      if (this->signal_update)
//      {
//          this->pos = SelectedActor->GetPosition();
//          this->rpy = SelectedActor->GetOrientation();
//      }


  }

  virtual void OnMiddleButtonUp() override
  {
      std::cout << "MIDDLE UP" << std::endl;
  }
  virtual void OnMiddleButtonDown() override
  {
      std::cout << "MIDDLE DOWN" << std::endl;
  }
  virtual void OnRightButtonUp() override
  {
      std::cout << "RIGHT UP" << std::endl;      
//      vtkInteractorStyleTrackballActor::OnLeftButtonUp();

//      this->pos = SelectedActor->GetPosition();
//      this->rpy = SelectedActor->GetOrientation();
//      this->signal_update = true;
  }
  virtual void OnRightButtonDown() override
  {
      std::cout << "RIGHT DOWN" << std::endl;
//      this->signal_update = false;

//      vtkInteractorStyleTrackballActor::OnLeftButtonDown();

//      if (this->InteractionProp == this->SelectedActor)
//      {
//           std::cout << "Picked actor." << std::endl;
//           this->pos = SelectedActor->GetPosition();
//           this->rpy = SelectedActor->GetOrientation();
//           vtkInteractorStyleTrackballActor::OnLeftButtonDown();
//      }

//      else
//          vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  void SetActor(vtkSmartPointer<vtkAssembly> actor)
  {
    this->SelectedActor = actor;
  }

  bool GetCoords(double* &pos_, double* &rpy_)
  {
        pos_ = this->pos;
        rpy_ = this->rpy;

        return true;
  }

  bool getSignalUpdate()
  {
      return this->signal_update;
  }

  void setSignalActor(bool signal)
  {
      this->signal_update = signal;
  }


private:
  vtkSmartPointer<vtkAssembly> SelectedActor;
  vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
  double* pos;
  double* rpy;
  bool signal_update = false;
};
vtkStandardNewMacro(DragInteractorStyle)

} // namespace



#endif // DRAGINTERACTORSTYLE_H
