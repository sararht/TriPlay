#include "vtktools.h"

#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSTLWriter.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkGeometryFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkInformation.h>
#include <vtkUnstructuredGrid.h>
#include <vtkTriangleFilter.h>
#include <vtkAdaptiveSubdivisionFilter.h>
#include <vtkSTLReader.h>
#include <vtkSmoothPolyDataFilter.h>


bool vtkTools::getNormalPolyData(vtkSmartPointer<vtkPolyData> polyData, double (&n)[3])
{
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(polyData);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    auto normals = normalGenerator->GetOutput()->GetPointData()->GetNormals();
    auto size = normals->GetNumberOfTuples();

    if (size == 0)
        return false;

    double normal_mean[3] = {0,0,0};

    int prueba=0;
    //OJO AQUÍ
    for (int i=0; i<normals->GetNumberOfTuples(); i++)
    {
       double* values;
       values = normals->GetTuple3(i);

       if (values[0]==0 && values[1]==0 && values[2]==0)
           continue;
       normal_mean[0] = normal_mean[0] + values[0];
       normal_mean[1] = normal_mean[1] + values[1];
       normal_mean[2] = normal_mean[2] + values[2];

     //  std::cout <<values[0] << ", " << values[1] <<", " << values[2] << std::endl;
    prueba++;
    }

    n[0] = normal_mean[0]/prueba;
    n[1] = normal_mean[1]/prueba;
    n[2] = normal_mean[2]/prueba;



    return true;
}

//vtkAlgorithmOutput* vtkTools::getOutputPortPolyData(vtkSmartPointer<vtkPolyData> polyData)
//{
//    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
//    triangleFilter->SetInputData(polyData);
//    triangleFilter->Update();

//    return (triangleFilter->GetOutputPort());
//}


bool vtkTools::writeSTL(vtkAlgorithmOutput* polyData, char *name)
{

    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileTypeToBinary();
    stlWriter->SetFileName(name);
    stlWriter->SetInputConnection(polyData);
    stlWriter->Write();

    return true;
}

bool vtkTools::writeSTL(vtkSmartPointer<vtkPolyData> polyData, char *name)
{

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polyData);
    triangleFilter->Update();

    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileTypeToBinary();
    stlWriter->SetFileName(name);
    stlWriter->SetInputConnection(triangleFilter->GetOutputPort());
    stlWriter->Write();

    return true;
}

bool vtkTools::extractSelectionPolyData(vtkSmartPointer<vtkPolyData> polyData_in, vtkIdTypeArray* ids, vtkSmartPointer<vtkPolyData> &polyData_out, vtkSmartPointer<vtkPolyData> &polyData_out_inverted )
{
    vtkNew<vtkSelectionNode> selectionNode;
    selectionNode->SetFieldType(vtkSelectionNode::POINT);
    selectionNode->SetContentType(vtkSelectionNode::INDICES);
    selectionNode->SetSelectionList(ids);
    selectionNode->GetProperties()->Set(vtkSelectionNode::CONTAINING_CELLS(), 1);

    vtkNew<vtkSelection> selection;
    selection->AddNode(selectionNode);

    vtkNew<vtkExtractSelection> extractSelection;
    extractSelection->SetInputData(0, polyData_in);
   // extractSelection->SetInputConnection(0, polyData_in);
    extractSelection->SetInputData(1, selection);
    extractSelection->Update();

    vtkNew<vtkUnstructuredGrid> selected;
    selected->ShallowCopy(extractSelection->GetOutput());

    std::cout << selected->GetNumberOfPoints() << std::endl;
    if (selected->GetNumberOfPoints() == 0)
    {
        return false;
    }
    vtkSmartPointer<vtkGeometryFilter> geometryFilter = vtkSmartPointer<vtkGeometryFilter>::New();
    geometryFilter->SetInputData(selected);
    geometryFilter->Update();

    polyData_out = geometryFilter->GetOutput();

    // Not in selection
    selectionNode->GetProperties()->Set(vtkSelectionNode::INVERSE(), 1); // invert the selection
    extractSelection->Update();

    vtkNew<vtkUnstructuredGrid> notSelected;
    notSelected->ShallowCopy(extractSelection->GetOutput());

    vtkSmartPointer<vtkGeometryFilter> geometryFilter_not = vtkSmartPointer<vtkGeometryFilter>::New();
    geometryFilter_not->SetInputData(notSelected);
    geometryFilter_not->Update();

    polyData_out_inverted = geometryFilter_not->GetOutput();

    return true;
}

bool vtkTools::subdividePolyData(vtkSmartPointer<vtkPolyData> polyData_in, float maxEdgeLength, vtkSmartPointer<vtkPolyData> &polyData_out)
{
    if(polyData_in->GetNumberOfPoints() == 0)
        return false;

    vtkSmartPointer<vtkPolyDataAlgorithm> subdivisionFilter;
    subdivisionFilter = vtkSmartPointer<vtkAdaptiveSubdivisionFilter>::New();
    subdivisionFilter->SetInputData(polyData_in);

    //dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->SetMaximumEdgeLength(maxEdgeLength/10); //AJUSTAR SEGÚN ÁREA DEFECTO
    dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->SetMaximumTriangleArea(maxEdgeLength); //AJUSTAR SEGÚN ÁREA DEFECTO

    subdivisionFilter->Update();

    vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter;
    smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoothFilter->SetInputData(subdivisionFilter->GetOutput());
    smoothFilter->Update();

    polyData_out = smoothFilter->GetOutput();

    if(polyData_out->GetNumberOfPoints() == 0)
        return false;
    return true;
}

void vtkTools::printNPointsCellsPoly_debug(vtkSmartPointer<vtkPolyData> polyData)
{
    std::cout << "There are " << polyData->GetNumberOfPoints()
              << " points in the selection." << std::endl;
    std::cout << "There are " << polyData->GetNumberOfCells()
              << " cells in the selection." << std::endl;
}

void vtkTools::printNPointsCellsPoly_debug(vtkUnstructuredGrid* polyData)
{
    std::cout << "There are " << polyData->GetNumberOfPoints()
              << " points in the selection." << std::endl;
    std::cout << "There are " << polyData->GetNumberOfCells()
              << " cells in the selection." << std::endl;
}


bool vtkTools::smoothPolyData(vtkSmartPointer<vtkPolyData> polyData_in, vtkSmartPointer<vtkPolyData> &polyData_out)
{
    vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter;
    smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoothFilter->SetInputData(polyData_in);
    smoothFilter->Update();

    polyData_out = smoothFilter->GetOutput();
}



//// KDTREE
///
bool vtkTools::getIntersectRayObbTree(vtkSmartPointer<vtkOBBTree> tree, double* pSource, double* pTarget, vtkSmartPointer<vtkPoints> &points, vtkSmartPointer<vtkIdList> &cellsIds)
{
    points = vtkSmartPointer<vtkPoints>::New();
    cellsIds = vtkSmartPointer<vtkIdList>::New();

    int code = tree->IntersectWithLine(pSource, pTarget, points, cellsIds);
    if (code==0)
        return false;

    return true;
}
