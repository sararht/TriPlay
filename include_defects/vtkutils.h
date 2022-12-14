#ifndef VTKUTILS_H
#define VTKUTILS_H

#include <vtkPolyData.h>


class vtkUtils
{
public:
    vtkUtils();
    bool estimateNormalPolyData(vtkSmartPointer<vtkPolyData> polyData, double n[3]);
};

#endif // VTKUTILS_H
