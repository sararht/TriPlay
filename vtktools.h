#ifndef VTKTOOLS_H
#define VTKTOOLS_H

#include <vtkPolyData.h>
#include <vtkAlgorithmOutput.h>
#include <vtkPoints.h>
#include <vtkUnstructuredGrid.h>

#include <vtkOBBTree.h>


namespace vtkTools {

    bool getNormalPolyData(vtkSmartPointer<vtkPolyData> polyData, double (&n)[3]);
    bool writeSTL(vtkAlgorithmOutput *polyData, char* name);
    bool writeSTL(vtkSmartPointer<vtkPolyData> polyData, char *name);
    bool extractSelectionPolyData(vtkSmartPointer<vtkPolyData> polyData_in, vtkIdTypeArray* ids, vtkSmartPointer<vtkPolyData> &polyData_out, vtkSmartPointer<vtkPolyData> &polyData_out_inverted);
    vtkAlgorithmOutput* getOutputPortPolyData(vtkSmartPointer<vtkPolyData> polyData);
    bool subdividePolyData(vtkSmartPointer<vtkPolyData> polyData_in, float maxEdgeLength, vtkSmartPointer<vtkPolyData> &polyData_out);
    void printNPointsCellsPoly_debug(vtkSmartPointer<vtkPolyData> polyData);
    void printNPointsCellsPoly_debug(vtkUnstructuredGrid* polyData);
    bool smoothPolyData(vtkSmartPointer<vtkPolyData> polyData_in, vtkSmartPointer<vtkPolyData> &polyData_out);

    bool getIntersectRayObbTree(vtkSmartPointer<vtkOBBTree> tree, double* pSource, double* pTarget, vtkSmartPointer<vtkPoints> &points, vtkSmartPointer<vtkIdList> &cellsIds);



}




#endif // VTKTOOLS_H
