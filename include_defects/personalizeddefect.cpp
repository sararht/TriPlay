#include "personalizeddefect.h"

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkImplicitModeller.h>
#include <vtkContourFilter.h>
#include <vtkSTLWriter.h>
#include <vtkTriangleFilter.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>

#include <vtkSTLReader.h>


PersonalizedDefect::PersonalizedDefect(){}
PersonalizedDefect::PersonalizedDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_, int cols_raw_,int rows_raw_,float scale_depth_, QVector<float> z_)
{
    rows_raw = rows_raw_;
    cols_raw = cols_raw_;
    scale_depth = scale_depth_;
    z_raw=z_;

    posxyz = posxyz_;
    length = length_;
    width = width_;
    depth = depth_;
    normal = normal_;
    dir_deg = dir_deg_;

}

inline void printQVector3D(QVector3D v)
{
    std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
}

void ReallyDeletePoint(vtkSmartPointer<vtkPoints> points, vtkIdType id)
{
  vtkSmartPointer<vtkPoints> newPoints =
    vtkSmartPointer<vtkPoints>::New();

  for(vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
    {
    if(i != id)
      {
      double p[3];
      points->GetPoint(i,p);
      newPoints->InsertNextPoint(p);
      }
    }

  points->ShallowCopy(newPoints);
}

//void PersonalizedDefect::createDefectVTK()
//{

//    //Load model in VTK
//    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
//    reader->SetFileName("/home/sara/sararht/TESIS/Codigo/modelos/chapa2.stl");
//    reader->Update();


//      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();

//      points->SetNumberOfPoints(z_raw.size());
//      cells->SetNumberOfCells(z_raw.size());

//      QVector<int> ids;
//      for(int i=0; i<z_raw.size(); i++)
//      {
//          double p[3];
//          p[0] = round(i/rows_raw) +posxyz.x()-rows_raw/2;
//          p[2] = round(i%cols_raw) +posxyz.z()-cols_raw/2;

//          int id;
//          double z_ = getProfile(QVector2D(p[0],p[2]), reader->GetOutput(), id);
//          if (z_ != -1)
//          {
//              ids.push_back(id);
//              std::cout << z_ << std::endl;
//          }
//          p[1] = scale_depth*z_raw[i]+30;
//          points->SetPoint(i,p);
//          cells->InsertNextCell(1);
//          cells->InsertCellPoint(i);

//        //  printQVector3D(QVector3D(p[0],p[1],p[2]));
//      }

////      points->SetNumberOfPoints(length*width);
////      cells->SetNumberOfCells(length*width);


//      for (int i=0; i<ids.size();i++)
//         ReallyDeletePoint(reader->GetOutput()->GetPoints(),ids[i]);


//      vtkSmartPointer<vtkPolyData> mVtkData = vtkSmartPointer<vtkPolyData>::New();
//      mVtkData->SetPoints(points);
//      mVtkData->SetVerts(cells);
//      mVtkData->BuildCells();

//      vtkSmartPointer<vtkSurfaceReconstructionFilter> surface = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
//      surface->SetSampleSpacing(1);
//      surface->AddInputData(mVtkData);
//      surface->Update();

//      vtkSmartPointer<vtkContourFilter> filter = vtkSmartPointer<vtkContourFilter>::New();
//      filter->SetInputConnection(surface->GetOutputPort());
//      filter->SetValue(0,0.0);

//      // Sometimes the contouring algorithm can create a volume whose gradient
//       // vector and ordering of polygon (using the right hand rule) are       // inconsistent. vtkReverseSense cures this problem.
//       vtkNew<vtkReverseSense> reverse;
//       reverse->SetInputConnection(filter->GetOutputPort());
//       reverse->ReverseCellsOn();
//       reverse->ReverseNormalsOn();
//       reverse->Update();

//      // Write the stl file to disk
//      vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
//      stlWriter->SetFileTypeToBinary();
//      stlWriter->SetFileName("PRUEBA_SURFACE.stl");
//      stlWriter->SetInputConnection(reverse->GetOutputPort()); //mesh //cleanFilter
//      stlWriter->Write();

//      vtkSmartPointer<vtkSTLWriter> stlWriter2 = vtkSmartPointer<vtkSTLWriter>::New();
//      stlWriter2->SetFileTypeToBinary();
//      stlWriter2->SetFileName("deletepoints.stl");
//      stlWriter2->SetInputConnection(reader->GetOutputPort()); //mesh //cleanFilter
//      stlWriter2->Write();


//}

void PersonalizedDefect::fillLattice()
{

    // Reconstruir lattice en función de las dimensiones del defecto
    iarray3E dim;
    dim[0] = 1;//1
    dim[1] = rows_raw;
    dim[2] = cols_raw;
    lattice->setDimension(dim);
    lattice->build();

    int ndof = lattice->getNNodes();

    displ = dvecarr3E(ndof, darray3E{0,0,0});
    for (int i=0; i<ndof; i++){
        int l0,l1,l2;
        int index = lattice->accessGridFromDOF(i);
        lattice->accessPointIndex(index,l0,l1,l2);

        float x,y;
        x=l2;
        y=l1/*+24*/;

        displ[i][0]=this->scale_depth*z_raw[x*cols_raw+y];

    }

    input = new mimmo::GenericInput();
    input->setReadFromFile(false);
    input->setInput(displ);

    // Set Generic output block to write the displacements defined above.
    output = new mimmo::GenericOutput();
    output->setFilename("manipulators_output_00003.csv");
    output->setCSV(true);
}
