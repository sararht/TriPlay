#ifndef DEFECT_H
#define DEFECT_H

#include <QString>

#include <QVector>
#include <QVector3D>
#include <QVector2D>
#include <mimmo.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkBoundingBox.h>
#include <vtkSTLReader.h>

#define PI 3.141592

struct crackDef_params
{
    QVector<float> lenghtD;
    QVector<float> widthD;
    float width_crack;
    int n_intermediate_points;
};

class CrackDefect;
class BumpDefect;
class PickDefect;

class Defect
{
public:
    Defect();
    Defect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_);

  //  void setReaderModel(vtkSmartPointer<vtkSTLReader> reader);
    void setPolydataModel(vtkSmartPointer<vtkPolyData> poly);
    void constructLattice();
    void constructLatticeWithSelection(vtkSmartPointer<vtkPolyData> polydata);
    void constructLatticeWithSelection2(vtkSmartPointer<vtkPolyData> polydata);

    virtual void fillLattice() = 0;
    bool insertDefect(std::string stl_file, std::string stl_file_new);
    void destructLattice();
    void subdivideArea();
    int subdivideVTK(std::string stl_file);

    void subdivideLattice();
    void adaptLattice();

    void mergeLattice(std::string stl_file, std::string stl_file_new);

    bool isPinVolumen(QVector3D p);
    bool isPinVolumen2(QVector3D p, bool withMargenSeg);
    bool isPinDeterminedVolumen(double* p, QVector3D min, QVector3D max);


    bool insertDefectVTK(std::string stl_file, std::string stl_file_new);
  //  virtual void createDefectVTK() = 0;
    double getProfile(QVector2D, vtkSmartPointer<vtkPolyData>, int);


    static Defect *make_defect(QString file_name);
    static Defect *make_predefinedDefect(QString type, double depth_, float deg_rot_, QString personalized_type, vtkSmartPointer<vtkPolyData> polydata, crackDef_params *params);


    void debugPrint(std::string s){std::cout << s << std::endl;}
    void debugPrint(int i){std::cout << i << std::endl;}
    void debugPrint(float f){std::cout << f << std::endl;}
    void debugPrint(double d){std::cout << d << std::endl;}
    void debugPrint(iarray3E a){std::cout << a[0] << ", " << a[1] << ", " << a[2] << std::endl;}
    void debugPrint(darray3E a){std::cout << a[0] << ", " << a[1] << ", " << a[2] << std::endl;}
    void debugPrint(double* a){std::cout << a[0] << ", " << a[1] << ", " << a[2] << std::endl;}

    int SUBDIVIDE_ERROR = 0;
    int PROBLEMATIC_AREAS = 1;
    int NO_PROBLEMATIC_AREAS = 2;

    double getDepth(){return depth;}
    float getScaleDepth(){return scale_depth;}



    
protected:
    QString type;
    QVector3D posxyz;
    float length;
    float width;
    float depth;
    float scale_depth;
    QVector3D normal;
    float dir_deg;

    iarray3E dim_world;

    QVector<QVector<double>> shape_stl;
    dmatrix33E axes;
    QVector3D normal_model;
    mimmo::FFDLattice* lattice;  
    mimmo::GenericInput* input;
    mimmo::GenericOutput * output;
    dvecarr3E displ;

    QVector<mimmo::FFDLattice*> vLattice; //For subdivide in problematic geometrie
    QVector<mimmo::GenericInput*> vInput;
    QVector<dvecarr3E> vDispl;
    QVector<QVector3D> vNormalsLattice;
    QVector<QVector3D> vOriginsLattice;
    QVector<vtkBoundingBox> vBoundings_divisions;

    QVector<QVector3D>vNormals;

//    vtkSmartPointer<vtkSTLReader> reader_model;
    vtkSmartPointer<vtkPolyData> polydata;

    mimmo::Apply* applier = new mimmo::Apply();
    mimmo::MimmoGeometry * mimmo0;
    mimmo::MimmoGeometry * mimmo1;
    mimmo::MimmoGeometry * mimmoMerge;

    bool isProblematicAreas = false;

};

#endif // DEFECT_H
