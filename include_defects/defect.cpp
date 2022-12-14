#include "defect.h"
#include "crackdefect.h"
#include "bumpdefect.h"
#include "pickdefect.h"
#include "lossdefect.h"
#include "personalizeddefect.h"
#include "fitplane3d.h"

#include <QStringList>
#include <QFile>
#include <QDomDocument>

#include <math.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPlane.h>
//#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkButterflySubdivisionFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkAdaptiveSubdivisionFilter.h>
#include<vtkProbePolyhedron.h>

//#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLStructuredGridReader.h>
#include <vtkXMLGenericDataObjectReader.h>

#include <vtkPPolyDataNormals.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkOBBTree.h>

#include <vtkSelectPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkSelectionNode.h>
#include <vtkIdTypeArray.h>
#include <vtkInformation.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkTriangleFilter.h>
#include <vtkUnstructuredGridWriter.h>

#include <vtkBoundingBox.h>

#include <QMatrix3x3>
#include <QQuaternion>

#include <VTUGridReader.hpp>
#include <vtkDataSetMapper.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkGeometryFilter.h>

#include "vtktools.h"


bool DEBUG_SUBDIVIDE_AREA = true; //MIRAR COMO PONERLO MEJOR
bool CONSIDER_PROBLEMATIC_AREAS = false;
bool ONLY_SURFACE_UP = false;

inline QVector<float> fromString2(QString &str)
{
    str.replace(";", " ");

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

inline void printQVector3D(QVector3D v)
{
    std::cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << std::endl;
}
dmatrix33E mult(dmatrix33E M, dmatrix33E N)
{
    dmatrix33E m = dmatrix33E({M[0][0]*N[0][0]+M[0][1]*N[1][0]+M[0][2]*N[2][0], M[0][0]*N[0][1]+M[0][1]*N[1][1]+M[0][2]*N[2][1], M[0][0]*N[0][2]+M[0][1]*N[1][2]+M[0][2]*N[2][2],
                               M[1][0]*N[0][0]+M[1][1]*N[1][0]+M[1][2]*N[2][0], M[1][0]*N[0][1]+M[1][1]*N[1][1]+M[1][2]*N[2][1], M[1][0]*N[0][2]+M[1][1]*N[1][2]+M[1][2]*N[2][2] ,
                               M[2][0]*N[0][0]+M[2][1]*N[1][0]+M[2][2]*N[2][0], M[2][0]*N[0][1]+M[2][1]*N[1][1]+M[2][2]*N[2][1], M[2][0]*N[0][2]+M[2][1]*N[1][2]+M[2][2]*N[2][2] });
    return m;
}
QVector3D roundNormal(QVector3D n)
{
    if (abs(n.x())>=abs(n.y()) && abs(n.x())>=abs(n.z()))
        //if (n.x < 0) return QVector3D(-1,0,0);
        return QVector3D(1,0,0);

    else if (abs(n.y())>abs(n.x()) && abs(n.y())>=abs(n.z()))
        //if (n.y < 0) return QVector3D(0,-1,0);
        return QVector3D(0,1,0);

    else if (abs(n.z())>abs(n.y()) && abs(n.z())>abs(n.x()))
        // if (n.z < 0) return QVector3D(0,0,-1);
        return QVector3D(0,0,1);
}
float area(QVector2D c1,QVector2D c2,QVector2D c3)
{
    return fabs((c1.x() * (c2.y() - c3.y()) + c2.x() * (c3.y() - c1.y()) +
                 c3.x() * (c1.y() - c2.y())) / 2.0);
}
template <typename T>
void reverseQVector(QVector<T> &v)
{
    QVector<T> aux;
    for(int i=v.size()-1; i>=0;i--)
    {
        aux.push_back(v[i]);
    }
    v.clear();
    v=aux;
}



Defect::Defect(){}
Defect::Defect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_)
    :posxyz(posxyz_),length(length_),width(width_),depth(depth_),normal(normal_),dir_deg(dir_deg_){}


Defect* Defect::make_predefinedDefect(QString type, double depth_, float deg_rot_, QString personalized_type, vtkSmartPointer<vtkPolyData> polydata, crackDef_params* params)
{
    if(type=="Bump")
    {
        return new BumpDefect(QVector3D(0,0,0), 0, 0, depth_, QVector3D(0,0,0), deg_rot_);
    }

    else if(type=="Pick")
    {
        return new PickDefect(QVector3D(0,0,0),0,depth_,QVector3D(0,0,0));
    }
    else if(type=="Personalized")
    {
        std::stringstream name_file;
        name_file << "../simulador/defects_catalog/"<<personalized_type.toStdString()<<".xml";

        Defect *d = make_defect(QString::fromStdString(name_file.str()));
        d->scale_depth = depth_/d->depth;
        d->dir_deg = deg_rot_;
        return d;
    }

    else if(type=="Crack")
    {
//        //Calculamos la normal
//        double n_new[3];
//        vtkTools::getNormalPolyData(polydata, n_new);
//        QVector3D normal = QVector3D(round(n_new[0]),round(n_new[1]), round(n_new[2]));

        //Ajustar según la normal, esto solo me vale en un caso!!, ¿o no?
        //Igual está bien ajustado porque luego se rota el lattice? Tengo que comprobar eso -> no, diría que no


        //Calculamos boundingBox
        vtkBoundingBox bounding; bounding.SetBounds(polydata->GetBounds());
        double xm,xM,ym,yM,zm,zM; bounding.GetBounds(xm,xM,ym,yM,zm,zM);
        double center[3]; bounding.GetCenter(center);
        QVector3D origin = QVector3D(xm,ym,zm);
        QVector3D end = QVector3D(xM,ym,zM);

        float width_cr = params->width_crack;
        std::cout << width_cr << std::endl;

        int n_intermidiate_points = params->n_intermediate_points;
        std::cout << n_intermidiate_points << std::endl;

        QVector<float> distributionL, distributionW;
        distributionL = params->lenghtD;
        distributionW = params->widthD;


        QVector<QVector3D> points;
        QVector3D cte = end - origin;

        for (int i=0; i<n_intermidiate_points+2;i++)
        {
            points.push_back(QVector3D(xm,ym,zm) + cte*QVector3D(distributionL[i],0,distributionW[i]));
        }
        return new CrackDefect(points,depth_,width_cr,QVector3D(0,1,0),30,30); //PARÁMETROS: N_COLUMNS_FIRST,LAST


    }

}

Defect *Defect::make_defect(QString file_name)
{
    //The QDomDocument class represents an XML document.
    QDomDocument xmlBOM;

    QFile file(file_name);
    if (!file.open(QIODevice::ReadOnly ))
    {
           // Error while loading file
    }
    else
    {
        xmlBOM.setContent(&file);
        file.close();

        QVector3D posxyz;
        float length=0;
        float depth=0;
        float width=0;
        QVector3D normal;
        float dir_deg=0;

        //grieta
        QVector<QVector3D> crack_points;
        int n_columns_first=0, n_columns_last=0;

        //personalizado
        int cols_raw =0, rows_raw = 0;
        float scale_depth = 1;
        QVector<float> z_raw(cols_raw*rows_raw);

        // Leer XML-----------------------------------------------------------------------------------------
        // Extract the root markup
        QDomElement root = xmlBOM.documentElement();
        QString type = root.tagName();


        QDomElement Component=root.firstChild().toElement();
        while(!Component.isNull())
        {

            if (Component.tagName()=="posxyz")
            {
               QString values =Component.firstChild().toText().data();
               QVector <float> v_f = fromString2(values);

               posxyz = QVector3D(v_f[0], v_f[1], v_f[2]);
            }
            if (Component.tagName()=="longitud")
               length = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="profundidad")
               depth = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="ancho")
               width = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="normal")
            {
               QString values =Component.firstChild().toText().data();
               QVector <float> v_f = fromString2(values);

               normal = QVector3D(v_f[0], v_f[1], v_f[2]);
            }

            if (Component.tagName()=="direccion_deg")
               dir_deg = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="columnasPrimer")
                n_columns_first = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="columnasUltimo")
                n_columns_last = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="escalaProfundidad")
               scale_depth = Component.firstChild().toText().data().toFloat();

            if (Component.tagName()=="puntos")
            {

               QDomElement Component2=Component.firstChild().toElement();
               while (Component2.tagName()=="posxyz")
               {
                   QString values =Component2.firstChild().toText().data();
                   QVector <float> v_f = fromString2(values);

                   QVector3D posxyz_ = QVector3D(v_f[0], v_f[1], v_f[2]);
                   crack_points.push_back(posxyz_);

                   Component2 = Component2.nextSibling().toElement();
               }
            }

            if (Component.tagName()=="defecto")
            {

                QDomElement Component2=Component.firstChild().toElement();
                while(!Component2.isNull())
                {
                    if (Component2.tagName()=="filas")
                        rows_raw = Component2.firstChild().toText().data().toFloat();

                    if (Component2.tagName()=="columnas")
                        cols_raw = Component2.firstChild().toText().data().toFloat();

                    if (Component2.tagName()=="profundidad")
                        depth = Component2.firstChild().toText().data().toFloat();

                    if (Component2.tagName()=="z")
                    {
                        QString values =Component2.firstChild().toText().data();
                        z_raw = fromString2(values);


                    }
                    Component2 = Component2.nextSibling().toElement();

                }


            }

            Component = Component.nextSibling().toElement();
        }

        if (type == "Grieta")
        {
            return new CrackDefect(crack_points,depth,width,normal,n_columns_first, n_columns_last);
        }
        if (type == "Bollo")
        {
            return new BumpDefect(posxyz, length, width, depth, normal, dir_deg);
        }
        if (type == "Pico")
        {
            return new PickDefect(posxyz,width,depth,normal);
        }
        if (type == "FaltaMaterial")
        {
            return new LossDefect(posxyz, length, width, depth, normal, dir_deg);
        }
        if (type == "Personalizado")
        {
            return new PersonalizedDefect(posxyz, length, width, depth, normal, dir_deg, cols_raw, rows_raw, scale_depth, z_raw);
        }

    }
}

void Defect::constructLattice()
{
    lattice = new mimmo::FFDLattice();
    darray3E origin = {posxyz.x(), posxyz.y(), posxyz.z()};
    float dir_rad = dir_deg* M_PI/180.0;

    //Set lattice dimensions
    darray3E span;
    //Set number of nodes of the mesh (dim) and degree of nurbs functions (deg).
    iarray3E dim, deg;

    span[0]= 50; //50
    span[1]= length;
    span[2]= width;

    dim[0] = 30;//Actualizando esto permito subdividir más el lattice a la hora de ajustar la forma
    dim[1] = 50;
    dim[2] = 50;

    deg[0] = 5;
    deg[1] = 5;
    deg[2] = 5;

    axes = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});

    lattice->setLattice(origin,span,mimmo::ShapeType::CUBE,dim, deg);

    // Change reference system to work in local cylindrical coordinates. --LO TENGO PORQUE ME FUNCIONA, PERO PODRÍA QUITARLO

    //lattice->setRefSystem(axes);
    //auto ref =lattice->getRefSystem();
    //lattice->setRefSystem(2, darray3E{0,-1,0});

//    lattice->plotCloud("/home/sara/","probandoLattice.vtu",1,0,1);
    lattice->setDisplGlobal(false); //Displacement global==true, local==false
    lattice->setPlotInExecution(true);

    // Build mesh of lattice outside the execution chain to use it during setup the displacements.
    lattice->build();

}

void Defect::constructLatticeWithSelection(vtkSmartPointer<vtkPolyData> polydata)
{
     vtkTools::writeSTL(polydata,"HAND_SELECTION_STL.stl");
     //Calculate bounding box
     vtkBoundingBox bounding; bounding.SetBounds(polydata->GetBounds());
     double xm,xM,ym,yM,zm,zM; bounding.GetBounds(xm,xM,ym,yM,zm,zM);
     double center[3]; bounding.GetCenter(center);
     this->posxyz.setX(center[0]); this->posxyz.setY(center[1]); this->posxyz.setZ(center[2]);

     //Cambiar la normal según la zona seleccionada? normal=round(normal_mean)?
     //----
     double n_new[3];
     vtkTools::getNormalPolyData(polydata, n_new);
     this->normal = QVector3D(round(n_new[0]),round(n_new[1]), round(n_new[2]));

     //----
     //Calcular orientación de la selección sobre la normal
     vtkNew<vtkOBBTree> tree;
     int maxLevel = 1;
     tree->SetDataSet(polydata);
     tree->SetMaxLevel(maxLevel);
     tree->BuildLocator();

     double corner[3] = {0.0, 0.0, 0.0};
     double max[3] = {0.0, 0.0, 0.0};
     double mid[3] = {0.0, 0.0, 0.0};
     double min[3] = {0.0, 0.0, 0.0};
     double size[3] = {0.0, 0.0, 0.0};

     tree->ComputeOBB(polydata, corner, max, mid, min, size);

//     std::cout << "Corner:\t" << corner[0] << ", " << corner[1] << ", "
//                 << corner[2] << std::endl
//                 << "Max:\t" << max[0] << ", " << max[1] << ", " << max[2]
//                 << std::endl
//                 << "Mid:\t" << mid[0] << ", " << mid[1] << ", " << mid[2]
//                 << std::endl
//                 << "Min:\t" << min[0] << ", " << min[1] << ", " << min[2]
//                 << std::endl
//                 << "Size:\t" << size[0] << ", " << size[1] << ", " << size[2]
//                 << std::endl;
  //----



     double dimensions[3] = {xM-xm, yM-ym, zM-zm};
     QVector3D dim_q(dimensions[0],dimensions[1],dimensions[2]);
     double height = QVector3D::dotProduct(this->normal, dim_q);

      if (abs(this->normal.x()) == 1){this->length = dimensions[1];width = dimensions[2];}
      if (abs(this->normal.y()) == 1){this->length = dimensions[0];width = dimensions[2];}
      if (abs(this->normal.z()) == 1){this->length = dimensions[0];width = dimensions[1];}

     //Crear lattice
     this->lattice = new mimmo::FFDLattice();
     darray3E origin = {this->posxyz.x(), this->posxyz.y(), this->posxyz.z()};
     float dir_rad = this->dir_deg* M_PI/180.0;

     //Set lattice dimensions
     darray3E span;
     //Set number of nodes of the mesh (dim) and degree of nurbs functions (deg).
     iarray3E dim, deg;

     span[0] = height; if(span[0]<10) span[0]= 50;
     span[1]= this->length; //girados!!!!!
     span[2]= this->width;


     if (CONSIDER_PROBLEMATIC_AREAS)
     {

         dim[0] = 50;//Actualizando esto permito subdividir más el lattice a la hora de ajustar la forma
         dim[1] = 50; //10,50,50
         dim[2] = 50;

         deg[0] = 20; //Para problematic areas aumentar!
         deg[1] = 20;
         deg[2] = 20;
     }
     else
     {
         dim[0] = 1;//Actualizando esto permito subdividir más el lattice a la hora de ajustar la forma
         dim[1] = 50; //1,50,50
         dim[2] = 50;

         deg[0] = 1; //Para problematic areas aumentar! Si no = 1
         deg[1] = 1; //5
         deg[2] = 1; //5

     }

     clock_t tStart2_ = clock();

     axes = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});
     lattice->setLattice(origin,span,mimmo::ShapeType::CUBE,dim, deg);
     lattice->setDisplGlobal(false); //Displacement global==true, local==false
     lattice->setPlotInExecution(true);


     // Build mesh of lattice outside the execution chain to use it during setup the displacements.
     lattice->build();

     std::cout <<"Tiempo construccion Lattice" << (double)(clock() - tStart2_)/CLOCKS_PER_SEC << std::endl;

}

//No usada
void Defect::constructLatticeWithSelection2(vtkSmartPointer<vtkPolyData> polydata)
{

    vtkTools::writeSTL(polydata,"HAND_SELECTION_STL.stl");

    //Calculate bounding box
    vtkBoundingBox bounding; bounding.SetBounds(polydata->GetBounds());
    double xm,xM,ym,yM,zm,zM; bounding.GetBounds(xm,xM,ym,yM,zm,zM);
    double center[3]; bounding.GetCenter(center);
    this->posxyz.setX(center[0]); this->posxyz.setY(center[1]); this->posxyz.setZ(center[2]);

    //Cambiar la normal según la zona seleccionada? normal=round(normal_mean)?
    //----
    double n_new[3];
    vtkTools::getNormalPolyData(polydata, n_new);
    this->normal = QVector3D(round(n_new[0]),round(n_new[1]), round(n_new[2]));
    //----

    double dimensions[3] = {xM-xm, yM-ym, zM-zm};
    QVector3D dim_q(dimensions[0],dimensions[1],dimensions[2]);
    double height = QVector3D::dotProduct(this->normal, dim_q);

     if (abs(this->normal.x()) == 1){this->length = dimensions[1];width = dimensions[2];}
     if (abs(this->normal.y()) == 1){this->length = dimensions[0];width = dimensions[2];}
     if (abs(this->normal.z()) == 1){this->length = dimensions[0];width = dimensions[1];}

    //Crear lattice
    this->lattice = new mimmo::FFDLattice();
    darray3E origin = {this->posxyz.x(), this->posxyz.y(), this->posxyz.z()};
    float dir_rad = this->dir_deg* M_PI/180.0;

    //Set lattice dimensions
    darray3E span;
    //Set number of nodes of the mesh (dim) and degree of nurbs functions (deg).
    iarray3E dim, deg;

    span[0] = height; if(span[0]<10) span[0]= 50;
    span[1]= this->length;
    span[2]= this->width;

    dim[0] = 1;//Actualizando esto permito subdividir más el lattice a la hora de ajustar la forma
    dim[1] = 50;
    dim[2] = 50;

    deg[0] = 20;
    deg[1] = 20;
    deg[2] = 20;

    axes = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});
    lattice->setLattice(origin,span,mimmo::ShapeType::CUBE,dim, deg);
    lattice->setDisplGlobal(false); //Displacement global==true, local==false
    lattice->setPlotInExecution(true);

    // Build mesh of lattice outside the execution chain to use it during setup the displacements.
    lattice->build();


}

//////Ya no las uso///////
void Defect::subdivideLattice()
{
    //Trasladar aquí el cálculo de los orígenes


    //Actualizamos los boundings para que se toquen
    for(int i=0; i<vBoundings_divisions.size()-1;i++)
    {
        double xm,xM,ym,yM,zm,zM, xm_i,xM_i,ym_i,yM_i,zm_i,zM_i;
        vBoundings_divisions[i].GetBounds(xm,xM,ym,yM,zm,zM);
        vBoundings_divisions[i+1].GetBounds(xm_i,xM_i,ym_i,yM_i,zm_i,zM_i);

       // vBoundings_divisions[i].SetBounds(xm-abs(xM_i-xm),xM,ym,yM+abs(ym_i-yM),zm,zM);
    }

    double xm,xM,ym,yM,zm,zM, xm_i,xM_i,ym_i,yM_i,zm_i,zM_i,  xm_i2,xM_i2,ym_i2,yM_i2,zm_i2,zM_i2;
    vBoundings_divisions[1].GetBounds(xm,xM,ym,yM,zm,zM);
    vBoundings_divisions[0].GetBounds(xm_i,xM_i,ym_i,yM_i,zm_i,zM_i);
    vBoundings_divisions[2].GetBounds(xm_i2,xM_i2,ym_i2,yM_i2,zm_i2,zM_i2);

  //  vBoundings_divisions[1].SetBounds(xm,xM,ym-abs(yM_i2-ym_i2)/2,yM+abs(yM_i2-ym_i2)/2,zm,zM);

    //BUENA PARA EL OTRO: vBoundings_divisions[1].SetBounds(xm-abs(xM_i2-xm),xM+abs(xm_i-xM),ym/*-abs(yM_i-ym)*/,yM/*+abs(yM-ym_i2)*/,zm,zM);

    double desf_yM, desf_ym;

    if(ym_i>ym_i2)
    {
        desf_yM = +abs(yM-ym_i);
        desf_ym = -abs(yM_i2-ym);
    }
    else
    {
        desf_yM = +abs(yM-ym_i2);
        desf_ym = -abs(yM_i-ym);
    }

    vBoundings_divisions[1].SetBounds(xm-abs(xM_i2-xm),xM+abs(xm_i-xM),ym+desf_ym,yM+desf_yM,zm,zM);

    double largo_global=0;
    int ndof_global=0;
    for(int i=0; i<vBoundings_divisions.size();i++)
    {
        double xm,xM,ym,yM,zm,zM;
        vBoundings_divisions[i].GetBounds(xm,xM,ym,yM,zm,zM);
        largo_global = largo_global + sqrt((xM-xm)*(xM-xm) +(yM-ym)*(yM-ym));
    }

    for (int i=0; i<vBoundings_divisions.size();i++)
    {
        double xm,xM,ym,yM,zm,zM;
        vBoundings_divisions[i].GetBounds(xm,xM,ym,yM,zm,zM);
        double largo = sqrt((xM-xm)*(xM-xm) +(yM-ym)*(yM-ym));
        ndof_global = ndof_global + largo*50/largo_global;
    }

    double dim_ant = 0;

    for(int i=0; i<vOriginsLattice.size();i++)
    {

        double xm,xM,ym,yM,zm,zM, xm_i=0,xM_i=0,ym_i,yM_i,zm_i,zM_i;
        vBoundings_divisions[i].GetBounds(xm,xM,ym,yM,zm,zM);
        vBoundings_divisions[i+1].GetBounds(xm_i,xM_i,ym_i,yM_i,zm_i,zM_i);

        double largo = sqrt((xM-xm)*(xM-xm) +(yM-ym)*(yM-ym))  /*+ 10 */; //ver cuánto tengo que alargar
        QVector3D dimensions(abs(xm-xM),abs(ym-yM),abs(zm-zM));
        QVector3D normal_model_aux = vNormalsLattice[i];

        if (i%2!=0) //zona problematica (una estructura con un id o algo¿?)
            normal_model_aux = roundNormal(normal_model_aux);

//        double alto = abs(ym-yM);
//        if(i+1 < vOriginsLattice.size())
//        {
//            largo=largo + abs(ym_i-yM); //mmmmmmmmmmmm algo no cuadra
//            vOriginsLattice[i].setX(vOriginsLattice[i].x()-abs(ym_i-yM)/2);
//        }


        mimmo::FFDLattice* lattice_aux = new mimmo::FFDLattice();

    //   darray3E origin = {vOriginsLattice[i].x(), vOriginsLattice[i].y(), vOriginsLattice[i].z()};
        darray3E origin = {xm+(xM-xm)/2, ym+(yM-ym)/2, zm+(zM-zm)/2};

        if(i==1)
        {
           origin[1]=origin[1]-0.1;
        }

        float dir_rad = dir_deg* M_PI/180.0;

        //Set lattice dimensions
        darray3E span;
        //Set number of nodes of the mesh (dim) and degree of nurbs functions (deg).
        iarray3E dim, deg;
        dim = lattice->getDimension();

        span[0]= QVector3D::dotProduct(dimensions,QVector3D(abs(normal_model_aux.x()),abs(normal_model_aux.y()),abs(normal_model_aux.z())));//alto+10; //50 //Cambiar la altura
//        span[0]= QVector3D::dotProduct(dimensions,QVector3D(abs(normal_model_aux.x()),abs(normal_model_aux.y()),abs(normal_model_aux.z())));//alto+10; //50 //Cambiar la altura
        span[1]= largo;//length/3; //Cambiar según subdivisiones
        span[2]= width;

        int dim_act = largo*50/largo_global;
        if (i == vOriginsLattice.size()-1 && (lattice->getDimension()[1] - dim_ant) != dim_act)
        {
            dim_act = lattice->getDimension()[1] - dim_ant;
        }
        dim[0] = dim[0];//5;
        dim[1] = dim_act; // /largo_global //50/3; //Cambiar según subdivisiones
        dim[2] = dim[2];//50;

        deg[0] = 10;
        deg[1] = 10;
        deg[2] = 10;

        axes = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});

        lattice_aux->setLattice(origin,span,mimmo::ShapeType::CUBE,dim, deg);
        lattice_aux->setDisplGlobal(false); //Displacement global==true, local==false
        lattice_aux->setPlotInExecution(true);

        // Build mesh of lattice outside the execution chain to use it during setup the displacements.
        lattice_aux->build();

        int ndof = lattice_aux->getNNodes();

        mimmo::GenericInput* input_aux = new mimmo::GenericInput();
        //Dividir el relleno del lattice en función de las subdivisiones
        dvecarr3E displ_aux(ndof, darray3E{0,0,0});


        for (int j=0; j<ndof; j++)
        {
            int l0,l1,l2;
            int index = lattice_aux->accessGridFromDOF(j);
            lattice_aux->accessPointIndex(index,l0,l1,l2);
            int id = lattice->accessPointIndex(l0,l1+dim_ant,l2);
         //   int id=j+dim_ant;

//            if (i==1)
//            {
//                if(l1==0||l1==(dim[i]-1))
//                    continue;
//                //interpolar las primeras entradas
//                //Última columna del anterior

//            }
            displ_aux[j][0]= displ[id][0];


        }

        dim_ant = dim_ant + dim[1];

        input_aux->setReadFromFile(false);
        input_aux->setInput(displ_aux);

        vInput.push_back(input_aux);
        vLattice.push_back(lattice_aux);
        vDispl.push_back(displ_aux);
    }

}
void Defect::mergeLattice(std::string stl_file, std::string stl_file_new)
{
    //¿Modificar el lattice original?


//    QVector<QVector3D> normals_lattice;
//    normals_lattice.push_back(QVector3D(0.364348, 0.41228, -0.0206974));
//    normals_lattice.push_back(QVector3D(0.364348, 0.41228, -0.0206974));

    debugPrint("HOLA");

    for (int i=0; i< vLattice.size(); i++)
    {
        // Orientación según normal defecto
        if (abs(normal.y()))
            vLattice[i]->setRefSystem(1, darray3E({-1*normal.y(),0,0}));

        else
            vLattice[i]->setRefSystem(0, darray3E({normal.x(),normal.y(),normal.z()})); //No se por qué falla con la y


        vLattice[i]->build();


        //Subdivide Area and get normal rotation

        auto ref1= vLattice[i]->getRefSystem();
        normal_model = vNormalsLattice[i]; //normals_lattice[i]



        vLattice[i]->setRefSystem(1, darray3E({normal_model.x(),normal_model.y(),normal_model.z()})); //No se por qué falla con la y
        vLattice[i]->build();

        auto ref2= vLattice[i]->getRefSystem();
        auto axes11 = mult(ref1,ref2);
        vLattice[i]->setRefSystem(axes11);
        std::stringstream name;
        name << "aux" << i;
        vLattice[i]->setName(name.str());
        vLattice[i]->setId(0);
        vLattice[i]->build();

        std::stringstream name_0, name_f;
        mimmo::MimmoGeometry* mimmo_0 = new mimmo::MimmoGeometry();
        mimmo_0->setIOMode(IOMode::CONVERT);
        mimmo_0->setReadFileType(FileType::STL);

        mimmo::MimmoGeometry* mimmo_a = new mimmo::MimmoGeometry();
        mimmo_a->setIOMode(IOMode::WRITE);
        mimmo_a->setWriteFileType(FileType::STL);


        if (i==0)
        {
            name_0 << "tmpMerge";
        }
        else
        {
            name_0 << "probando" <<i-1;
        }

        mimmo_a->setWriteDir("./");

        if (i == vLattice.size()-1)
        {
            mimmo_a->setWriteDir("");
            name_f << stl_file_new;
        }
        else
        {
            name_f << "probando" <<i;
        }


        mimmo_0->setReadDir("./");
        mimmo_0->setReadFilename(name_0.str());
        mimmo_0->setWriteFileType(FileType::STL);

        mimmo_a->setWriteFilename(name_f.str());

        //Setup pin connections.
        mimmo::pin::addPin(mimmo_0, vLattice[i], M_GEOM, M_GEOM);
        mimmo::pin::addPin(vInput[i], vLattice[i], M_DISPLS, M_DISPLS);
        mimmo::pin::addPin(vInput[i], output, M_DISPLS, M_DISPLS);
        mimmo::pin::addPin(mimmo_0, applier, M_GEOM, M_GEOM);
        mimmo::pin::addPin(vLattice[i], applier, M_GDISPLS, M_GDISPLS);
        mimmo::pin::addPin(applier, mimmo_a, M_GEOM, M_GEOM);

        //Setup execution chain.
        mimmo::Chain ch0;
        ch0.addObject(mimmo_0);
        ch0.addObject(vInput[i]);
        ch0.addObject(output);
        ch0.addObject(vLattice[i]);
        ch0.addObject(applier);
        ch0.addObject(mimmo_a);

        //Execution of chain.
        //Use debug flag false (default) to avoid to to print out the execution steps.
        std::cout << " " << std::endl;
        std::cout << " --- execution start --- " << std::endl;
        ch0.exec();
        std::cout << " --- execution done --- " << std::endl;
        std::cout << " " << std::endl;

    }



/*

    int ndof = lattice->getNNodes();
    displ = dvecarr3E(ndof, darray3E{0,0,0});
    QVector<double> deg_rad;
    deg_rad.push_back(0);
    deg_rad.push_back(-45*PI/180);

    debugPrint("ndof_general:");
    debugPrint(ndof);
    for (int i=0; i< vLattice.size(); i++)
    {
        int n_dof_ant = 0;

        darray3E span =  vLattice[i]->getSpan();
        iarray3E dim =  vLattice[i]->getDimension();

        if(i>0) n_dof_ant = vLattice[i-1]->getNNodes();

        debugPrint("ndof_ant:");
        debugPrint(n_dof_ant);

        debugPrint("ndof_i:");
        debugPrint(vLattice[i]->getNNodes());

        for (int j=0; j<vLattice[i]->getNNodes(); j++)
        {
            int l0,l1,l2;
            int index = vLattice[i]->accessGridFromDOF(j);
            vLattice[i]->accessPointIndex(index,l0,l1,l2);
            float x, cte_x;
            cte_x = 50/25; //50=lenght defecto(span); 25= dim defecto
            x = - 50/2 + cte_x*l1;

            double z_aux = (x+span[0]/2)*std::tan(deg_rad[i]); //Arreglar esa parte. Mirar donde acabó el anterior y solaparlos
            dvecarr3E displ_aux = vDispl[i];

            debugPrint(z_aux);

            int id_aux = lattice->accessPointIndex(l0,l1+25*i,l2);
            int id_aux2 = lattice->accessDOFFromGrid(id_aux);

            displ[id_aux2][0]= displ_aux[j][0] + z_aux; //Luego habría que sumar aquí la rotacion
        }

    }

    input->setInput(displ);
*/
}
/////////////////////////

//Solo si problematic areas
void Defect::adaptLattice()
{
    iarray3E dim = lattice->getDimension();
    dvecarr3E displ_nuevo =dvecarr3E(lattice->getNNodes(), darray3E{0,0,0});

    printQVector3D(normal);
    std::cout << "N_NODES: " << lattice->getNNodes() << std::endl;


    for(int i=0;i<lattice->getNNodes();i++)
    {
        int l0,l1,l2;
        int index = lattice->accessGridFromDOF(i);
        lattice->accessPointIndex(index,l0,l1,l2);

        int l0_, l1_, l2_;
        if (abs(normal.y()))
        {
            l0_ =dim[1]-l1;
            l1_=l0;
            l2_=l2;
        }


        QVector3D n_aux = vNormals[l2_+l1_*dim_world[2]+l0_*dim_world[1]*dim_world[2]];



      //  QVector3D n_aux = vNormals[/*dim[1]-1-*/l1+l0*dim[1]/*+l2*(dim[0]+dim[1])*/];

//        printQVector3D(n_aux);
        if (n_aux.x()==-1 && n_aux.y()==-1 && n_aux.z()==-1)
            continue;
//        if (abs(n_aux.x())> 1 || abs(n_aux.y()) > 1 || abs(n_aux.z()) >1)
//            continue;

        if (fabs(n_aux.x())> 1) n_aux.setX(0);
        if (fabs(n_aux.y())> 1) n_aux.setY(0);
        if (fabs(n_aux.z())> 1) n_aux.setZ(0);

      //  std::cout << "N_AUX: "; printQVector3D(n_aux);

//        if(l0<dim[0]-2)
//            l0=l0+1;

//        if(abs(n_aux.x())>0.3 &&l1<dim[1]-2)
//            l1=l1+1;
        int ii = lattice->accessPointIndex(l0,l1,l2);


    //    std::cout << "i: "<<i <<", ii: " << ii << std::endl;
        //-------------------------------------------------------------------
        //-------------------------------------------------------------------
        /*
        darray3E span = lattice->getSpan();

        float cte_x;
        float cte_y;
        float sigmax, sigmay; //0.06,0.01
        float xm_ = 0;
        float ym_ = 0;

        //Actualización del cálculo de sigmax y sigmay
        sigmax = span[2]/6;
        sigmay = span[1]/6;

        cte_x = span[2]/dim[2];
        float x,y;
        x = - span[2]/2 + cte_x*l2;
        y = - span[1]/2 + cte_y*l1;
        displ[i][0] = - depth*exp(-((x-xm_)*(x-xm_)/(2*sigmax*sigmax)) - ((y-ym_)*(y-ym_)/(2*sigmay*sigmay)));
*/
//-------------------------------------------------------------------
        //-------------------------------------------------------------------
        if (normal.y()==1)
        {
            displ_nuevo[i][0] = displ[i][0]/*5*/*n_aux.y();
            displ_nuevo[i][1] = -displ[i][0]/*5*/*n_aux.x();
            displ_nuevo[i][2] = displ[i][0]/*5*/*n_aux.z();
        }
        else if (normal.y()==-1)
        {
            displ_nuevo[i][0] = -displ[i][0]*n_aux.y();
            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
        }
        else if (normal.x()==1)
        {
            displ_nuevo[i][0] = displ[i][0]*n_aux.x();
            displ_nuevo[i][1] = displ[i][0]*n_aux.y();
            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
        }
        else if (normal.x()==-1)
        {
            displ_nuevo[i][0] = -displ[i][0]*n_aux.x();
            displ_nuevo[i][1] = displ[i][0]*n_aux.y();
            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
        }
        else if (normal.z()==1)
        {
            displ_nuevo[i][0] = displ[i][0]*n_aux.z();
            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
            displ_nuevo[i][2] = displ[i][0]*n_aux.y();
        }
        else if (normal.z()==-1)
        {
            displ_nuevo[i][0] = -displ[i][0]*n_aux.z();
            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
            displ_nuevo[i][2] = displ[i][0]*n_aux.y();
        }

//        displ_nuevo[i][0] = -displ[i][0]*n_aux.y();
//        displ_nuevo[i][1] = displ[i][0]*n_aux.x();
//        displ_nuevo[i][2] = displ[i][0]*n_aux.z();

//        if(fabs(displ_nuevo[i][0]) > fabs(displ[i][0])+fabs(depth)) displ_nuevo[i][0] = 0;
//        if(fabs(displ_nuevo[i][1]) > fabs(displ[i][1])+fabs(depth)) displ_nuevo[i][0] = 0;
//        if(fabs(displ_nuevo[i][2]) > fabs(displ[i][2])+fabs(depth)) displ_nuevo[i][0] = 0;

    }



    printQVector3D(normal);
    debugPrint("---------------");

   input->setInput(displ_nuevo);
}
//void Defect::adaptLattice()
//{
//    iarray3E dim = lattice->getDimension();
//    dvecarr3E displ_nuevo =dvecarr3E(lattice->getNNodes(), darray3E{0,0,0});

//    printQVector3D(normal);
//    std::cout << "N_NODES: " << lattice->getNNodes() << std::endl;


//    for(int i=0;i<lattice->getNNodes();i++)
//    {
//        int l0,l1,l2;
//        int index = lattice->accessGridFromDOF(i);
//        lattice->accessPointIndex(index,l0,l1,l2);

//        int l0_, l1_, l2_;
//        if (abs(normal.y()))
//        {
//            l0_ =l1;
//            l1_=l0;
//            l2_=l2;
//        }


//        QVector3D n_aux = vNormals[l2_+l1_*dim_world[2]+l0_*dim_world[1]*dim_world[2]];
//      //  QVector3D n_aux = vNormals[/*dim[1]-1-*/l1+l0*dim[1]/*+l2*(dim[0]+dim[1])*/];

////        printQVector3D(n_aux);
//        if (n_aux.x()==-1 && n_aux.y()==-1 && n_aux.z()==-1)
//            continue;

////        if(l0<dim[0]-2)
////            l0=l0+1;

////        if(abs(n_aux.x())>0.3 &&l1<dim[1]-2)
////            l1=l1+1;
//        int ii = lattice->accessPointIndex(l0,l1,l2);


//    //    std::cout << "i: "<<i <<", ii: " << ii << std::endl;


//        if (normal.y()==1)
//        {
//            displ_nuevo[ii][0] = displ[i][0]/*5*/*n_aux.y();
//            displ_nuevo[ii][1] = -displ[i][0]/*5*/*n_aux.x();
//            displ_nuevo[ii][2] = displ[i][0]/*5*/*n_aux.z();
//        }
//        else if (normal.y()==-1)
//        {
//            displ_nuevo[i][0] = -displ[i][0]*n_aux.y();
//            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
//            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
//        }
//        else if (normal.x()==1)
//        {
//            displ_nuevo[i][0] = displ[i][0]*n_aux.x();
//            displ_nuevo[i][1] = displ[i][0]*n_aux.y();
//            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
//        }
//        else if (normal.x()==-1)
//        {
//            displ_nuevo[i][0] = -displ[i][0]*n_aux.x();
//            displ_nuevo[i][1] = displ[i][0]*n_aux.y();
//            displ_nuevo[i][2] = displ[i][0]*n_aux.z();
//        }
//        else if (normal.z()==1)
//        {
//            displ_nuevo[i][0] = displ[i][0]*n_aux.z();
//            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
//            displ_nuevo[i][2] = displ[i][0]*n_aux.y();
//        }
//        else if (normal.z()==-1)
//        {
//            displ_nuevo[i][0] = -displ[i][0]*n_aux.z();
//            displ_nuevo[i][1] = displ[i][0]*n_aux.x();
//            displ_nuevo[i][2] = displ[i][0]*n_aux.y();
//        }


//    }


//    printQVector3D(normal);
//    debugPrint("---------------");

//   input->setInput(displ_nuevo);
//}
//

bool Defect::insertDefect(std::string stl_file, std::string stl_file_new)
{
    mimmo0 = new mimmo::MimmoGeometry();
    mimmo0->setIOMode(IOMode::CONVERT);
    mimmo0->setReadFileType(FileType::STL);
    mimmo0->setReadDir("");
    mimmo0->setReadFilename(stl_file);
    mimmo0->setWriteFileType(FileType::STL);

    mimmo1 = new mimmo::MimmoGeometry();
    mimmo1->setIOMode(IOMode::WRITE);
    mimmo1->setWriteDir("");
    mimmo1->setWriteFileType(FileType::STL);
    mimmo1->setWriteFilename(stl_file_new);

    std::cout<<"NORMAL: ";printQVector3D(normal);
    dmatrix33E axes_dir, axes;

    //Rotación según dir_deg
    float dir_rad = dir_deg* M_PI/180.0;
    axes_dir = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});

    //Rotación según dirección normal prioritaria
    if(abs(normal.x()) == 1) axes = dmatrix33E({normal.x(),0,0, 0,normal.x(),0, 0,0,1});
    else if(abs(normal.y()) == 1) axes = dmatrix33E({0,normal.y(),0, -normal.y(),0,0, 0,0,1});
    else if(abs(normal.z()) == 1) axes = dmatrix33E({0,0,normal.z(), 0,1,0, -normal.z(),0,0});

    axes = mult(axes_dir,axes);

    //Rotar 90º si normal en Z (no tengo claro por qué)
    if(abs(normal.z()) == 1)
    {
        //Rotar 90º
        dmatrix33E aux = dmatrix33E({1,0,0, 0,cos(M_PI/2),sin(M_PI/2), 0,-sin(M_PI/2),cos(M_PI/2)});
        axes = mult(aux,axes);
    }


    //Rotación según dirección normal modelo
    QVector3D normal_n = normal/normal.length();
    QVector3D normal_model_n = normal_model/normal_model.length();

    QVector3D axisRot = QVector3D::crossProduct(normal_n,normal_model_n);
    double b=QVector3D::dotProduct(normal_model_n, normal_n);
    double theta = atan2(sqrt(1-b*b),b);
    QQuaternion qRot = QQuaternion::fromAxisAndAngle(axisRot,-theta*180/M_PI);
    QMatrix3x3 M = qRot.toRotationMatrix();

    dmatrix33E M_({M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2)});
    dmatrix33E axes_normal = mult(axes,M_);

    lattice->setRefSystem(axes_normal);



/* Cambio 11 noviembre

    // Orientación según normal defecto
//    if (abs(normal.y()))
//        lattice->setRefSystem(1, darray3E({-1*normal.y(),0,0}));

//    else
//        lattice->setRefSystem(0, darray3E({normal.x(),normal.y(),normal.z()})); //No se por qué falla con la y

//    lattice->build();
//    auto ref1= lattice->getRefSystem();
//    std::cout << normal_model.x() << ", " << normal_model.y() << ", " << normal_model.z() << std::endl;
//    std::cout << "EEEEEEEEEEEEEEEEY" << std::endl;


//    // Orientación segun dir_deg
    float dir_rad = dir_deg* M_PI/180.0;
    axes = dmatrix33E({1,0,0, 0,cos(dir_rad),sin(dir_rad), 0,-sin(dir_rad),cos(dir_rad)});
  //  auto axes11 = mult(axes,ref1);

    std::cout << "Axes dir_deg: "<< axes << std::endl;

//    // Orientación según normal_model
    std::cout << "Normal model: "; printQVector3D(normal_model);

    //lattice->setRefSystem(0, darray3E({normal_model.x(),normal_model.y(),normal_model.z()})); //No se por qué falla con la y
//    if (abs(normal.y()))
//    {
//        lattice->setRefSystem(1, darray3E({-1*normal_model.y(),0,0}));
//        std::cout << "1 " << std::endl;
//    }

//    else
//    {
       lattice->setRefSystem(0, darray3E({normal_model.x(),normal_model.y(),normal_model.z()})); //No se por qué falla con la y
       std::cout << "2 " << std::endl;
//    }
    std::cout << "Normal model: "; printQVector3D(normal_model);

 //   lattice->setRefSystem(1, darray3E({-1*normal.y(),0,0}));

    lattice->build();
    auto ref2= lattice->getRefSystem();

    std::cout << "Axes normal_model: "<< ref2 << std::endl;

    auto axes_fin = mult(axes,ref2);
    std::cout << "Axes fin: " << axes_fin << std::endl;

    lattice->setRefSystem(axes_fin);

    debugPrint("HEY");

    if(abs(normal.z()) == 1)
    {
        //Rotar 90º
        dmatrix33E aux = dmatrix33E({1,0,0, 0,cos(M_PI/2),sin(M_PI/2), 0,-sin(M_PI/2),cos(M_PI/2)});
        auto axes_fin2 = mult(aux,axes_fin);

        lattice->setRefSystem(axes_fin2);
        std::cout << "Axes fin 2: " << axes_fin << std::endl;

    }


*/

    lattice->setName("aux");
    lattice->setId(0);
    lattice->build();
    //-----------------------

    //mimmo0 ---> mimmoMerge con subdivide
    mimmoMerge = new mimmo::MimmoGeometry();
    mimmoMerge->setIOMode(IOMode::CONVERT);
    mimmoMerge->setReadFileType(FileType::STL);
    mimmoMerge->setReadDir("./");
    mimmoMerge->setReadFilename("tmpMerge");
    mimmoMerge->read();

    //Setup pin connections.
    mimmo::pin::addPin(mimmoMerge, lattice, M_GEOM, M_GEOM);
    mimmo::pin::addPin(input, lattice, M_DISPLS, M_DISPLS);
    mimmo::pin::addPin(input, output, M_DISPLS, M_DISPLS);
    mimmo::pin::addPin(mimmoMerge, applier, M_GEOM, M_GEOM);
    mimmo::pin::addPin(lattice, applier, M_GDISPLS, M_GDISPLS);
    mimmo::pin::addPin(applier, mimmo1, M_GEOM, M_GEOM);

    //Setup execution chain.
    mimmo::Chain ch0;
    ch0.addObject(mimmoMerge);
    ch0.addObject(input);
    ch0.addObject(output);
    ch0.addObject(lattice);
    ch0.addObject(applier);
    ch0.addObject(mimmo1);

    //Execution of chain.
    //Use debug flag false (default) to avoid to to print out the execution steps.
    std::cout << " " << std::endl;
    std::cout << " --- execution start --- " << std::endl;
    ch0.exec();
    std::cout << " --- execution done --- " << std::endl;
    std::cout << " " << std::endl;


    //Free memory
    // Clean up & exit;
    ch0.deleteObject(5);
    ch0.deleteObject(4);
    ch0.deleteObject(3);
    ch0.deleteObject(2);
    ch0.deleteObject(1);
    ch0.deleteObject(0);
    ch0.clear();

//    delete applier;
//    delete lattice;
//    delete input;
//    delete output;
//    delete mimmo0;
//    delete mimmo1;
//    lattice = NULL;
//    applier = NULL;
//    input   = NULL;
//    output   = NULL;
//    mimmo0  = NULL;
//    mimmo1  = NULL;

    destructLattice();

}

bool Defect::isPinDeterminedVolumen(double* p_, QVector3D min, QVector3D max)
{

    float dir_rad;
    QVector3D p(p_[0],p_[1],p_[2]);

    if (p.x()>min.x() && p.x()<max.x())
    {
        if (p.z()>min.z() && p.z()<max.z())
        {
            if (p.y()>min.y() && p.y()<max.y())
                return true;

        }
    }

    return false;
}

bool Defect::isPinVolumen2(QVector3D p, bool withMargenSeg)
{
    darray3E dimSpan = lattice->getSpan();
    float margen_seg = 0;
    if (withMargenSeg)
        margen_seg = std::max(this->width,this->length)/3;

    float margen_seg_h = std::max(this->width,this->length)/3;

    float dir_rad;
    QVector2D o;

    //Actualizar dim en función de la normal
    QVector2D dim, c;
    dim = QVector2D(dimSpan[1]+margen_seg, dimSpan[2]+margen_seg);
    float h, h_max, h_min;

    if (abs(this->normal.y()) == 1)
    {
        c = QVector2D(p.x(),p.z());
        h = p.y();
        o = QVector2D(this->posxyz.x(), this->posxyz.z());
        h_min = this->posxyz.y() - dimSpan[0]/2 -margen_seg_h; //REVISAR ESTO, ANTES NO LO TENÍA. qUIZÁ INCLUIR EL MS DENTRO DEL /2
        h_max = this->posxyz.y() + dimSpan[0]/2 +margen_seg_h;
        dir_rad = -this->dir_deg*PI/180;

        if(this->normal.y() == -1)
            dir_rad = -dir_rad;

    }
    else if (abs(normal.x()) == 1)
    {
        c = QVector2D(p.y(),p.z());
        h = p.x();
        o = QVector2D(posxyz.y(), posxyz.z());
        h_min = posxyz.x() - dimSpan[0]/2 -margen_seg_h;
        h_max = posxyz.x() + dimSpan[0]/2 +margen_seg_h;
        dir_rad = (-dir_deg)*PI/180;

        if(this->normal.x() == 1)
            dir_rad = -dir_rad;
    }
    else if (abs(normal.z()) == 1)
    {
        c = QVector2D(p.x(),p.y());
        h = p.z();
        o = QVector2D(posxyz.x(), posxyz.y());
        h_min = posxyz.z() - dimSpan[0]/2 -margen_seg_h;
        h_max = posxyz.z() + dimSpan[0]/2 +margen_seg_h;
        dir_rad = (-dir_deg)*PI/180;

        if(this->normal.z() == 1)
            dir_rad = -dir_rad;
    }

    //Esquinas sin rotación
    QVector2D c1,c2,c3,c4;
    c1 = o+dim/2;
    c2 = QVector2D(o.x()-dim.x()/2, o.y()+dim.y()/2);
    c3 = o-dim/2;
    c4 = QVector2D(o.x()+dim.x()/2, o.y()-dim.y()/2);

    //Esquinas rotadas
    QVector2D c1_,c2_,c3_,c4_;
    c1_.setX((c1.x()-o.x())*cos(dir_rad)-(c1.y()-o.y())*sin(dir_rad) + o.x());
    c1_.setY((c1.x()-o.x())*sin(dir_rad)+(c1.y()-o.y())*cos(dir_rad) + o.y());

    c2_.setX((c2.x()-o.x())*cos(dir_rad)-(c2.y()-o.y())*sin(dir_rad) + o.x());
    c2_.setY((c2.x()-o.x())*sin(dir_rad)+(c2.y()-o.y())*cos(dir_rad) + o.y());

    c3_.setX((c3.x()-o.x())*cos(dir_rad)-(c3.y()-o.y())*sin(dir_rad) + o.x());
    c3_.setY((c3.x()-o.x())*sin(dir_rad)+(c3.y()-o.y())*cos(dir_rad) + o.y());

    c4_.setX((c4.x()-o.x())*cos(dir_rad)-(c4.y()-o.y())*sin(dir_rad) + o.x());
    c4_.setY((c4.x()-o.x())*sin(dir_rad)+(c4.y()-o.y())*cos(dir_rad) + o.y());


    //Comprobar si el punto está en el rectángulo
    float A,A1,A2,A3,A4;
    A= area(c1_,c2_,c3_) + area(c1_,c4_,c3_);
    A1=area(c,c1_,c2_);
    A2=area(c,c2_,c3_);
    A3=area(c,c3_,c4_);
    A4=area(c,c1_,c4_);

    if (round(A) == round(A1+A2+A3+A4) && (h>h_min && h<h_max)) return true;
    else return false;
}

//////////////Ya no se usa////////////////////
bool Defect::isPinVolumen(QVector3D p)
{

    darray3E dimSpan = lattice->getSpan();
    float margen_seg = 70;

    if(dir_deg!= 0)
        margen_seg = 160;


    QVector3D dim;
    if (abs(normal.x())==1)
        dim = QVector3D(dimSpan[0], dimSpan[1], dimSpan[2]) + QVector3D(margen_seg,margen_seg,margen_seg);

    else if(abs(normal.y())==1)
        dim = QVector3D(dimSpan[1], dimSpan[0], dimSpan[2]) + QVector3D(margen_seg,margen_seg,margen_seg);

    else if(abs(normal.z())==1)
        dim = QVector3D(dimSpan[1], dimSpan[2], dimSpan[0]) + QVector3D(margen_seg,margen_seg,margen_seg);

    QVector3D min = posxyz-dim/2;
    QVector3D max = posxyz+dim/2;

    if (p.x()>min.x() && p.x()<max.x())
    {
        if (p.z()>min.z() && p.z()<max.z())
        {
            if (p.y()>min.y() && p.y()<max.y())
                return true;

        }
    }

    return false;
}
//////////////////////////////////////////////

//void Defect::setReaderModel(vtkSmartPointer<vtkSTLReader> reader)
//{
//    this->reader_model = reader;
//}

void Defect::setPolydataModel(vtkSmartPointer<vtkPolyData> poly)
{
    this->polydata = poly;
}

int Defect::subdivideVTK(std::string stl_file)
{
    std::stringstream name_file;
    name_file << stl_file << ".stl";
    std::string name_aux_tmp = name_file.str();

    qDebug("Load model in vtk");
    //Load model in VTK
   // vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  //  reader->SetFileName(name_aux_tmp.c_str());
  //  reader->Update();


    std::cout << this->polydata->GetNumberOfPoints() << std::endl;

    qDebug("Generating triangles");

    //¿Para qué sirve el triangleFilter? ¿Por qué no directamente originalMesh = reader->GetOutput()?
    vtkSmartPointer<vtkPolyData> originalMesh;
    vtkSmartPointer<vtkTriangleFilter> triangles = vtkSmartPointer<vtkTriangleFilter>::New();
   // triangles->SetInputConnection(this->reader_model->GetOutputPort());
    triangles->SetInputData(this->polydata);
    triangles->Update();
    originalMesh = triangles->GetOutput();

    //Select surface to subdivide
    //Get normal for the subdivide area

    qDebug("Normal generator");

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator2 = vtkSmartPointer<vtkPolyDataNormals>::New();
    //normalGenerator2->SetInputData(this->reader_model->GetOutput());
    normalGenerator2->SetInputData(this->polydata);
    normalGenerator2->ComputePointNormalsOn();
    normalGenerator2->ComputeCellNormalsOff();
    normalGenerator2->Update();

    auto normals_m = normalGenerator2->GetOutput()->GetPointData()->GetNormals();

    vtkNew<vtkIdTypeArray> ids;
    vtkNew<vtkIdTypeArray> ids_withoutMargin;
    vtkNew<vtkIdTypeArray> ids_problematic_points;

    QVector<int> id_points;
    QVector<int> id_points_withoutMargin;

    qDebug("selecting polydata");

    QVector<QVector3D> points;
    QVector<QVector3D>normals_points;
    for(int i=0; i<originalMesh->GetNumberOfPoints();i++)
    {
        auto p = originalMesh->GetPoint(i);
        double *n;

        n= normals_m->GetTuple3(i);
        if(isPinVolumen2(QVector3D(p[0],p[1],p[2]),true))
        {

            //Comenté esto para que me coja las dos partes del modelo!!
            if (ONLY_SURFACE_UP)
            {
                if (QVector3D::dotProduct(normal,QVector3D(n[0],n[1],n[2]))<0) //DESCOMENTAR
                    continue; //Algo aquí está mal eh. No sabría decir qué
            }

            ids->InsertNextValue(i);
            id_points.push_back(i);
            points.push_back(QVector3D(p[0],p[1],p[2]));
            normals_points.push_back(QVector3D(n[0],n[1],n[2]));
        }

        if(isPinVolumen2(QVector3D(p[0],p[1],p[2]),false))
        {
            if (QVector3D::dotProduct(normal,QVector3D(n[0],n[1],n[2]))<0) //DESCOMENTAR
            {
                continue; //Algo aquí está mal eh. No sabría decir qué
            }

            ids_withoutMargin->InsertNextValue(i);
        }
    }    

    //Tratar los puntos seleccionados para encontrar zonas problemáticas
    // if(abs(1-(abs(QVector3D::dotProduct(normals_points[i],normal)))) <0.2 ) //0.01

    for(int i=0; i<points.size(); i++)
    {
         if(abs(1-(abs(normals_points[i][0])))<0.2 ) //0.01
         {
             if(isPinVolumen2(points[i],false))
                ids_problematic_points->InsertNextValue(id_points[i]);
         }
    }

    std::cout << "PROBLEMATIC POINTSSSS: " << ids_problematic_points->GetNumberOfValues() << std::endl;


    vtkSmartPointer<vtkPolyData> selected_polyData =  vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> notSelected_polyData =  vtkSmartPointer<vtkPolyData>::New();

    qDebug("extracting selection");

    //vtkTools::extractSelectionPolyData(this->reader_model->GetOutputPort(),ids, selected_polyData, notSelected_polyData);
    vtkTools::extractSelectionPolyData(this->polydata,ids, selected_polyData, notSelected_polyData);

    vtkSmartPointer<vtkTriangleFilter> selectionFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    selectionFilter->SetInputData(selected_polyData);
    selectionFilter->Update();
    vtkSmartPointer<vtkTriangleFilter> notSelectionFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    notSelectionFilter->SetInputData(notSelected_polyData);
    notSelectionFilter->Update();

    if(selected_polyData->GetNumberOfPoints() == 0)
        return SUBDIVIDE_ERROR;

    ///
    vtkSmartPointer<vtkPolyData> selected_polyData_wM =  vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> notSelected_polyData_wM =  vtkSmartPointer<vtkPolyData>::New();

   // vtkTools::extractSelectionPolyData(this->reader_model->GetOutputPort(),ids_withoutMargin, selected_polyData_wM, notSelected_polyData_wM);
    vtkTools::extractSelectionPolyData(this->polydata,ids_withoutMargin, selected_polyData_wM, notSelected_polyData_wM);

    vtkSmartPointer<vtkTriangleFilter> selectionFilter_wM = vtkSmartPointer<vtkTriangleFilter>::New();
    selectionFilter_wM->SetInputData(selected_polyData_wM);
    selectionFilter_wM->Update();
    ////

    if(selected_polyData_wM->GetNumberOfPoints() == 0)
            return SUBDIVIDE_ERROR;

    if (DEBUG_SUBDIVIDE_AREA)
    {
        vtkTools::printNPointsCellsPoly_debug(selected_polyData);
        vtkTools::printNPointsCellsPoly_debug(notSelected_polyData);

        vtkTools::writeSTL(selected_polyData_wM, "NO_MARGIN_SELECTION_VTK_TOOLS.stl");
        vtkTools::writeSTL(selected_polyData, "SELECTION_VTK_TOOLS.stl");
        vtkTools::writeSTL(notSelected_polyData, "NOT_SELECTION_VTK_TOOLS.stl");

    }

    //Selection for problematic areas----------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------
    vtkSmartPointer<vtkPolyData> selected_problematic_polyData;
    vtkSmartPointer<vtkPolyData> notselected_problematic_polyData;

    //isProblematicAreas = vtkTools::extractSelectionPolyData(this->reader_model->GetOutputPort(),ids_problematic_points,selected_problematic_polyData,notselected_problematic_polyData);
    isProblematicAreas = vtkTools::extractSelectionPolyData(this->polydata,ids_problematic_points,selected_problematic_polyData,notselected_problematic_polyData);

    if (!CONSIDER_PROBLEMATIC_AREAS)
        isProblematicAreas = false;

   // vtkTools::printNPointsCellsPoly_debug(selected_problematic_polyData);
   // vtkTools::writeSTL(selected_problematic_polyData,"problematic.stl");
/*
    if (false)//isProblematicAreas)
    {

        vtkSmartPointer<vtkTriangleFilter> selectionProblematicFilter = vtkSmartPointer<vtkTriangleFilter>::New();
        selectionProblematicFilter->SetInputData(selected_problematic_polyData);
        selectionProblematicFilter->Update();

        if (DEBUG_SUBDIVIDE_AREA)
        {
            std::cout << "There are " << selected_problematic_polyData->GetNumberOfPoints()
                      << " points in the selection." << std::endl;
            std::cout << "There are " << selected_problematic_polyData->GetNumberOfCells()
                      << " cells in the NOT selection." << std::endl;

            vtkTools::writeSTL(selectionProblematicFilter->GetOutputPort(), "VTKproblematicAreas.stl");

        }


         // Get number of problematic regions
         vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
         connectivityFilter->SetInputConnection(selectionProblematicFilter->GetOutputPort());

         connectivityFilter->Update();
         int n_problematic_regions = connectivityFilter->GetNumberOfExtractedRegions();
         connectivityFilter->SetExtractionModeToSpecifiedRegions();

         std::cout <<"Problematic regions: " << n_problematic_regions << std::endl;

         //Buscar el lugar de las regiones, si están cerca me vale con un nuevo lattice para ellas.
         //Guardar su posición y crear uno nuevo con el trozo de defecto que le tocaría en ese punto
         //Todo esto valdrá? Igual es todo una tontería

         //Calcular normales de esas regiones problemáticas
         QVector<QVector3D> vNormals_problematic_regions;
         QVector<vtkBoundingBox> vBoundings_problematic_regions;
         QVector<vtkSmartPointer<vtkPlane>> vPlanes_problematic_regions;

         //Separa la seleccion por regiones
         for(int i=0; i<n_problematic_regions; i++)
         {
             //Si tiene pocos puntos no le hago caso
             connectivityFilter->AddSpecifiedRegion(i); //select the region to extract here
             connectivityFilter->Update();    

             if (DEBUG_SUBDIVIDE_AREA)
             {
                 std::stringstream aux;
                 aux << "problematic_" <<i << ".stl";
                 vtkTools::writeSTL(connectivityFilter->GetOutputPort(),const_cast<char*>(aux.str().c_str()));
             }

             //if points<Xvalor
             // continue;

             //Calculat bounding box
             vtkBoundingBox bounding;
             bounding.SetBounds(connectivityFilter->GetOutput()->GetBounds());
             vBoundings_problematic_regions.push_back(bounding);

             //Calcular normales
             double normal_mean_aux[3];// = {0,0,0};
             vtkTools::getNormalPolyData(connectivityFilter->GetOutput(), normal_mean_aux);
             vNormals_problematic_regions.push_back(QVector3D(normal_mean_aux[0], normal_mean_aux[1], normal_mean_aux[2]));

        //     printQVector3D(QVector3D(normal_mean_aux[0], normal_mean_aux[1], normal_mean_aux[2]));
             //Calcular plano! y utilizarlo luego para divir el polydata
             vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
             plane->SetNormal(normal_mean_aux);

             QVector3D normal_mean_aux_aux = roundNormal(QVector3D(normal_mean_aux[0], normal_mean_aux[1], normal_mean_aux[2]));
             int dir=1;
             double dot = QVector3D::dotProduct(normal_mean_aux_aux,QVector3D(normal_mean_aux[0], normal_mean_aux[1], normal_mean_aux[2]));
             if(dot<0)
                 dir=-1;

             double origin[3];
             bounding.GetCenter(origin);

             double xm,xM,ym,yM,zm,zM;
             bounding.GetBounds(xm,xM,ym,yM,zm,zM);
             origin[0] = origin[0]+dir*abs(xM-origin[0])/4; //Segun el boundingbox y la direccion que me interese

             debugPrint("plane origins");
             debugPrint(xm);
             debugPrint(xM);

             plane->SetOrigin(origin);
             vPlanes_problematic_regions.push_back(plane);

             std::cout << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;

             //Clip
             vtkSmartPointer<vtkClipPolyData> clip = vtkSmartPointer<vtkClipPolyData>::New();
             clip->SetValue(0);
             clip->GenerateClippedOutputOn();
             clip->SetInputConnection(selectionFilter_wM->GetOutputPort());
             clip->SetClipFunction(plane);
             clip->Update();

             vtkSmartPointer<vtkGeometryFilter> geometryFilterClip = vtkSmartPointer<vtkGeometryFilter>::New();
             geometryFilterClip->SetInputData(clip->GetOutput());
             geometryFilterClip->Update();

             std::cout << clip->GetOutput()->GetNumberOfPoints() << std::endl;

             if (DEBUG_SUBDIVIDE_AREA)
             {
                 std::stringstream aux;
                 aux << "regions_" <<i << ".stl";
                 vtkTools::writeSTL(geometryFilterClip->GetOutputPort(),const_cast<char*>(aux.str().c_str()));
             }

             //Calcular normal Clip Region
             double normal_mean_clip[3];// = {0,0,0};
             vtkTools::getNormalPolyData(geometryFilterClip->GetOutput(), normal_mean_clip);
             vNormalsLattice.push_back(QVector3D(normal_mean_clip[0], normal_mean_clip[1], normal_mean_clip[2]));
             vNormalsLattice.push_back(QVector3D(normal_mean_aux[0], normal_mean_aux[1], normal_mean_aux[2]));

             vBoundings_divisions.push_back(geometryFilterClip->GetOutput()->GetBounds());
             vBoundings_divisions.push_back(bounding);


             double origin_clip[3];
             geometryFilterClip->GetOutput()->GetCenter(origin_clip);
             vOriginsLattice.push_back(QVector3D(origin_clip[0], origin_clip[1], origin_clip[2]));

             bounding.GetCenter(origin);
             vOriginsLattice.push_back(QVector3D(origin[0], origin[1],origin[2]));

             double normal_inverse[3] = {-normal_mean_aux[0], -normal_mean_aux[1], -normal_mean_aux[2]};
             origin[0] = origin[0]-dir*abs(xm-origin[0])/4;

             plane->SetNormal(normal_inverse);
             plane->SetOrigin(origin);
             clip->SetClipFunction(plane);
             clip->Update();

             geometryFilterClip->SetInputData(clip->GetOutput());
             geometryFilterClip->Update();

             //Calcular normal Clip Region
             double normal_mean_clip_inversed[3];// = {0,0,0};
             vtkTools::getNormalPolyData(geometryFilterClip->GetOutput(), normal_mean_clip_inversed);

             vNormalsLattice.push_back(QVector3D(normal_mean_clip_inversed[0], normal_mean_clip_inversed[1], normal_mean_clip_inversed[2]));
             double origin_clip_inversed[3];
             geometryFilterClip->GetOutput()->GetCenter(origin_clip_inversed);
             vOriginsLattice.push_back(QVector3D(origin_clip_inversed[0], origin_clip_inversed[1], origin_clip_inversed[2]));
             vBoundings_divisions.push_back(geometryFilterClip->GetOutput()->GetBounds());

             if (DEBUG_SUBDIVIDE_AREA)
             {
                 std::stringstream aux;
                 aux << "regions_reves" <<i << ".stl";
                 vtkTools::writeSTL(geometryFilterClip->GetOutputPort(),const_cast<char*>(aux.str().c_str()));
             }


             connectivityFilter->DeleteSpecifiedRegion(i);
             connectivityFilter->Update();

             if(dir==-1)
             {
                 //ESTO CUANDO TENGA MÁS DE UNA REGIÓN VA A SER PROBLEMÁTICO
                 reverseQVector(vNormalsLattice);
                 reverseQVector(vOriginsLattice);
                 reverseQVector(vBoundings_divisions);
                 //Girar todos los vectores!
             }
         }


//         debugPrint("NORMALSS");
//         printQVector3D(vNormalsLattice[0]);
//         printQVector3D(vNormalsLattice[1]);
//         printQVector3D(vNormalsLattice[2]);

    }
*/


   //----
    //Get normal for the subdivide area
    double normal_mean[3] = {0,0,0};
    vtkTools::getNormalPolyData(selectionFilter_wM->GetOutput(), normal_mean);
    normal_model = QVector3D(normal_mean[0],normal_mean[1],normal_mean[2]);

    cout << "NORMAL_MODEL_1" <<std::endl;
    printQVector3D(normal_model);

    //

    //Subdivide area
//    vtkSmartPointer<vtkPolyDataAlgorithm> subdivisionFilter;
//    subdivisionFilter = vtkSmartPointer<vtkAdaptiveSubdivisionFilter>::New();
//    subdivisionFilter->SetInputData(selected_polyData);

    float defect_area=width*length;
    float defect_tr=defect_area/100000;
    float aux = sqrt(16/3*defect_area*defect_tr);
    float side = sqrt(aux);
//    dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->SetMaximumEdgeLength(side/10); //AJUSTAR SEGÚN ÁREA DEFECTO
//    subdivisionFilter->Update();

    std::cout << "DEFECT_AREA: " << defect_area << std::endl;
    std::cout << "DEFECT_TR: " << defect_tr << std::endl;
    std::cout << "AUX: " << aux << std::endl;
    std::cout << "SIDE: " << side << std::endl;


    qDebug("SUBDIVIDIENDO...");
    vtkSmartPointer<vtkPolyData> subdivideSelected_polyData;
    vtkTools::subdividePolyData(selected_polyData,defect_tr*10,subdivideSelected_polyData);

    // Write the stl file to disk (ONLY FOR DEBUGGING PURPOSES)
    if (DEBUG_SUBDIVIDE_AREA)
    {
        vtkTools::writeSTL(subdivideSelected_polyData, "VTKsubdividide.stl");

        //std::cout << "PUNTOS TOTALES ANTES SUBDIVIDE: " << selected_polyData->GetNumberOfPoints() + notSelected_polyData->GetNumberOfPoints()<< std::endl;
        //std::cout << "PUNTOS TROZO ANTES SUBDIVIDE: " << selected_polyData->GetNumberOfPoints()<< std::endl;
        //std::cout << "PUNTOS TROZO DESPUÉS SUBDIVIDE: " << subdivideSelected_polyData->GetNumberOfPoints()<<  std::endl;
        //std::cout << "PUNTOS TROZO DESPUÉS SUBDIVIDE FUNCIÓN: " << subdivideSelected_polyData->GetNumberOfPoints()<<  std::endl;
        //std::cout << "PUNTOS TOTALES DESPUÉS SUBDIVIDE: " << subdivideSelected_polyData->GetNumberOfPoints()+notSelected_polyData->GetNumberOfPoints() << std::endl;
        //std::cout << "PUNTOS NOT SELECTED POLY DATA: " << notSelected_polyData->GetNumberOfPoints() << std::endl;
        //std::cout << "PUNTOS SELECTED POLY DATA: " << selected_polyData->GetNumberOfPoints() << std::endl;
    }

    qDebug("APPENDING...");
    //Append the two meshes
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInputData(notSelected_polyData); //notSelected_polyData
    appendFilter->AddInputData(subdivideSelected_polyData);
    appendFilter->Update();

    qDebug("CLEANING...");
//    vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
//    cleanFilter->AddInputData(appendFilter->GetOutput());
//    cleanFilter->Update();


    if (DEBUG_SUBDIVIDE_AREA)
    {
        //std::cout << "PUNTOS TROZO DESPUÉS APPEND: " << appendFilter->GetOutput()->GetNumberOfPoints()<< std::endl;
        //std::cout << "PUNTOS TROZO DESPUÉS CLEAN: " << cleanFilter->GetOutput()->GetNumberOfPoints()<< std::endl;
    }

    //-------------------------------------------------------------------------
    //MIRAR A VER LO QUE SOBRA Y LO QUE PUEDO LIMPIAR

       //Get normals points según lattice

    qDebug("HACIENDO OTRAS COSAS");

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator_sub = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator_sub->SetInputData(subdivideSelected_polyData);
    normalGenerator_sub->ComputePointNormalsOn();
    normalGenerator_sub->ComputeCellNormalsOff();
    normalGenerator_sub->Update();
    auto normals_m_sub = normalGenerator_sub->GetOutput()->GetPointData()->GetNormals();

    vtkNew<vtkIdTypeArray> ids_wM_s;
    for (int i=0; i<subdivideSelected_polyData->GetNumberOfPoints(); i++)
    {
        double *n;
        n= normals_m_sub->GetTuple3(i);

        auto p =subdivideSelected_polyData->GetPoint(i);
        if(isPinVolumen2(QVector3D(p[0],p[1],p[2]),false))
        {
            if (QVector3D::dotProduct(normal,QVector3D(n[0],n[1],n[2]))<0) //ARREGLAR ESTO PARA QUE SOLO ME COJA LA SUPERFICIE SUPERIOR!!!
                continue; //Algo aquí está mal eh

            ids_wM_s->InsertNextValue(i);
        }
    }

    vtkSmartPointer<vtkPolyData> selected_polyData_wM_sub =  vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> notSelected_polyData_wM_sub =  vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkCleanPolyData> clean1 = vtkSmartPointer<vtkCleanPolyData>::New();
    clean1->SetInputData(subdivideSelected_polyData);

    //vtkTools::extractSelectionPolyData(clean1->GetOutputPort(),ids_wM_s,selected_polyData_wM_sub,notSelected_polyData_wM_sub);
    vtkTools::extractSelectionPolyData(clean1->GetOutput(),ids_wM_s,selected_polyData_wM_sub,notSelected_polyData_wM_sub);

    vtkSmartPointer<vtkTriangleFilter> selectionFilter_wM_sub = vtkSmartPointer<vtkTriangleFilter>::New();
    selectionFilter_wM_sub->SetInputData(selected_polyData_wM_sub);
    selectionFilter_wM_sub->Update();

    if(DEBUG_SUBDIVIDE_AREA)
        vtkTools::writeSTL(selected_polyData_wM_sub, "SUB_NO_MARGIN_SELECTION_VTK_TOOLS.stl");

    vtkBoundingBox bb_subdivide;
    bb_subdivide.SetBounds(selectionFilter_wM_sub->GetOutput()->GetBounds());
    double new_origin[3];
    bb_subdivide.GetCenter(new_origin);
//    darray3E new_origin_ = {new_origin[0],new_origin[1],new_origin[2]};

    double xm,xM,ym,yM,zm,zM;
    bb_subdivide.GetBounds(xm,xM,ym,yM,zm,zM);

    //----------------------------------------------

    QVector3D new_span(abs(yM-ym),abs(xM-xm),abs(zM-zm));
    darray3E old_span = lattice->getSpan();

    if (abs(yM-ym) < old_span[0])
    {
        new_span.setX(old_span[0]);
    }

  //  lattice->setSpan(new_span.x(),new_span.y(),new_span.z()); //ESTO DEPENDE DE LAS NORMALESSS
  //  lattice->setOrigin(new_origin_);  //ESTO DEPENDE DE LAS NORMALESSS
  //  lattice->build();
    //----------------------------------------------

    iarray3E dim = lattice->getDimension();
    darray3E spacing = lattice->getSpacing(); //Tamaño de cada celda
    darray3E span = lattice->getSpan();

    posxyz = QVector3D(lattice->getOrigin()[0],lattice->getOrigin()[1],lattice->getOrigin()[2]);

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator_ = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator_->SetInputData(subdivideSelected_polyData);
    normalGenerator_->ComputePointNormalsOn();
    normalGenerator_->ComputeCellNormalsOff();
    normalGenerator_->Update();
    auto normals = normalGenerator_->GetOutput()->GetPointData()->GetNormals();

    QVector3D min(xm,ym,zm), max;
    spacing = lattice->getSpacing();

    this->dim_world = dim;
    if (abs(normal.y()))
    {
        xm = posxyz.x()-span[1]/2;
        ym = posxyz.y()-span[0]/2;
        zm = posxyz.z()-span[2]/2;

        xM = posxyz.x()+span[1]/2;
        yM = posxyz.y()+span[0]/2;
        zM = posxyz.z()+span[2]/2;
        spacing = darray3E{spacing[1],spacing[0],spacing[2]};
        dim_world = iarray3E{dim[1],dim[0],dim[2]};

    }
    else if(abs(normal.x()))
    {
        xm = posxyz.x()-span[0]/2;
        ym = posxyz.y()-span[1]/2;
        zm = posxyz.z()-span[2]/2;

        xM = posxyz.x()+span[0]/2;
        yM = posxyz.y()+span[1]/2;
        zM = posxyz.z()+span[2]/2;
        spacing = darray3E{spacing[0],spacing[1],spacing[2]};
        dim_world = iarray3E{dim[0],dim[1],dim[2]};

    }

    else if(abs(normal.z()))
    {
        xm = posxyz.x()-span[1]/2;
        ym = posxyz.y()-span[2]/2;
        zm = posxyz.z()-span[0]/2;

        xM = posxyz.x()+span[1]/2;
        yM = posxyz.y()+span[2]/2;
        zM = posxyz.z()+span[0]/2;
        spacing = darray3E{spacing[1],spacing[2],spacing[0]};
        dim_world = iarray3E{dim[1],dim[2],dim[0]};


    }

debugPrint(spacing);
printQVector3D(QVector3D(xm,ym,zm));
printQVector3D(QVector3D(xM,yM,zM));

std::cout << "dim0: " << dim[0] <<std::endl;
std::cout << "dim1: " << dim[1] <<std::endl;



//----------------------
//---------------------

if(isProblematicAreas)
{

    for (int l0=0;l0<dim_world[0];l0++)//dim[0]
    {
        for(int l1=0; l1<dim_world[1]; l1++)
        {
             for(int l2=0; l2<dim_world[2];l2++)
             {

                 min.setX(xm+spacing[0]*(l0-3));
                 min.setY(ym+spacing[1]*(l1-3));
                 min.setZ(zm+spacing[2]*(l2-3));

                 max.setX(xm+spacing[0]*(l0+3));
                 max.setY(ym+spacing[1]*(l1+3));
                 max.setZ(zm+spacing[2]*(l2+3));

                 QVector<double*>v_normals_aux;

              //   debugPrint("MIN_MAX:");
              //   printQVector3D(min);
              //   printQVector3D(max);


              //  std::cout << "N_POINTS: " << subdivideSelected_polyData->GetNumberOfPoints()<<std::endl;
                for(int p=0; p<subdivideSelected_polyData->GetNumberOfPoints(); p++)
                {
                    auto point = subdivideSelected_polyData->GetPoint(p);
                    //std::cout <<point[0] << ", " << point[1] <<", " << point[2] << std::endl;

                    if(isPinDeterminedVolumen(point,min,max)) //Actualizar esta función
                    {
                        double* values;
                        values = normals->GetTuple3(p);
                        // std::cout <<point[0] << ", " << point[1] <<", " << point[2] << std::endl;
                        // debugPrint("normal:");
                        // std::cout <<values[0] << ", " << values[1] <<", " << values[2] << std::endl;

                        if (values[0]==0 && values[1]==0 && values[2]==0)
                          continue;

                        v_normals_aux.push_back(values);
                  }
              }

              //Calcular media
              double v_normals_aux_mean[3] = {0,0,0};
              for(int n=0; n<v_normals_aux.size(); n++)
              {
                  v_normals_aux_mean[0] = v_normals_aux_mean[0] + v_normals_aux[n][0];
                  v_normals_aux_mean[1] = v_normals_aux_mean[1] + v_normals_aux[n][1];
                  v_normals_aux_mean[2] = v_normals_aux_mean[2] + v_normals_aux[n][2];

              }

              if (v_normals_aux.size()>0)
              {
                //  debugPrint(v_normals_aux_mean);
                //  debugPrint(v_normals_aux.size());

                  v_normals_aux_mean[0] = v_normals_aux_mean[0]/v_normals_aux.size();
                  v_normals_aux_mean[1] = v_normals_aux_mean[1]/v_normals_aux.size();
                  v_normals_aux_mean[2] = v_normals_aux_mean[2]/v_normals_aux.size();

                  vNormals.push_back(QVector3D(v_normals_aux_mean[0], v_normals_aux_mean[1], v_normals_aux_mean[2]));

               //   debugPrint("NORMAL:");
               //   printQVector3D(QVector3D(v_normals_aux_mean[0], v_normals_aux_mean[1], v_normals_aux_mean[2]));
              }
              else
              {
               //   debugPrint("-------NO HAY PUNTOS EN LA CELDA-------");
                  vNormals.push_back(QVector3D(-1,-1,-1)/*normal*/);

                //  printQVector3D(normal);

              }
          }

         }
    }


}




//---------------------
//---------------------

//      for (int l0=0;l0<dim[2];l0++)//dim[0]
//      {
//        for(int i=0; i<dim[1]; i++)
//        {
//    //        for(int l2=0; l2<dim[2];l2++)
//    //        {
//                QVector<double*>v_normals_aux;

//                if(abs(normal.y())==1)
//                {
//                    min.setX(xM - spacing[0]*(i+1));
//                    min.setY(ym/*ym + spacing[1]*(l0-1)*/);
//                    min.setZ(/*zm*/zm + spacing[2]*(l0-5));

//                    max.setX(xM - spacing[0]*(i-1));
//                    max.setY(yM/*ym + spacing[1]*(l0+1)*/);
//                    max.setZ(/*zM*/ zm + spacing[2]*(l0+5));
//                }

//                else if(abs(normal.x())==1)
//                {
//                    min.setX(xM - spacing[0]*(l0+1));
//                    min.setY(ym + spacing[1]*(i-1));
//                    min.setZ(zm/*zM + spacing[2]*(l2-5)*/);

//                    max.setX(xM - spacing[0]*(l0-1));
//                    max.setY(ym + spacing[1]*(i+1));
//                    max.setZ(zM/* + spacing[2]*(l2+5)*/);
//                }

//                else if(abs(normal.z())==1)
//                {
//                    min.setX(xM - spacing[1]*(i-1));
//                    min.setY(ym);
//                    min.setZ(zM + spacing[0]*(l0+1)/*zM + spacing[2]*(l2-5)*/);

//                    max.setX(xM + spacing[1]*(i+1));
//                    max.setY(yM);
//                    max.setZ(zM - spacing[0]*(l0-1)/* + spacing[2]*(l2+5)*/);
//                }
//                //COMPLETAR LA Z


//    //            if(l0==0)
//    //                min.setY(ym  + spacing[0]*(l0));

//    //            if(l0 == dim[0]-1)
//    //                max.setY(ym + spacing[0]*(l0));


//                debugPrint("MIN_MAX:");
//                printQVector3D(min);
//                printQVector3D(max);


//                std::cout << "N_POINTS: " << subdivideSelected_polyData->GetNumberOfPoints()<<std::endl;
//                for(int p=0; p<subdivideSelected_polyData->GetNumberOfPoints(); p++)
//                {
//                    auto point = subdivideSelected_polyData->GetPoint(p);
//                    //std::cout <<point[0] << ", " << point[1] <<", " << point[2] << std::endl;

//                    if(isPinDeterminedVolumen(point,min,max)) //Actualizar esta función
//                    {
//                        double* values;
//                        values = normals->GetTuple3(p);
//                         // std::cout <<point[0] << ", " << point[1] <<", " << point[2] << std::endl;
//                         // debugPrint("normal:");
//                         // std::cout <<values[0] << ", " << values[1] <<", " << values[2] << std::endl;

//                          if (values[0]==0 && values[1]==0 && values[2]==0)
//                            continue;

//                        v_normals_aux.push_back(values);
//                    }
//                }

//                //Calcular media
//                double v_normals_aux_mean[3] = {0,0,0};
//                for(int n=0; n<v_normals_aux.size(); n++)
//                {
//                    v_normals_aux_mean[0] = v_normals_aux_mean[0] + v_normals_aux[n][0];
//                    v_normals_aux_mean[1] = v_normals_aux_mean[1] + v_normals_aux[n][1];
//                    v_normals_aux_mean[2] = v_normals_aux_mean[2] + v_normals_aux[n][2];

//                }
//                // debugPrint("HOLA");



//                if (v_normals_aux.size()>0)
//                {
//                  //  debugPrint(v_normals_aux_mean);
//                  //  debugPrint(v_normals_aux.size());

//                    v_normals_aux_mean[0] = v_normals_aux_mean[0]/v_normals_aux.size();
//                    v_normals_aux_mean[1] = v_normals_aux_mean[1]/v_normals_aux.size();
//                    v_normals_aux_mean[2] = v_normals_aux_mean[2]/v_normals_aux.size();

//                    vNormals.push_back(QVector3D(v_normals_aux_mean[0], v_normals_aux_mean[1], v_normals_aux_mean[2]));

//                 //   debugPrint("NORMAL:");
//                    printQVector3D(QVector3D(v_normals_aux_mean[0], v_normals_aux_mean[1], v_normals_aux_mean[2]));
//                }
//                else
//                {
//                 //   debugPrint("-------NO HAY PUNTOS EN LA CELDA-------");
//                    vNormals.push_back(QVector3D(-1,-1,-1)/*normal*/);

//                  //  printQVector3D(normal);

//                }
//            }

//   //     }
//    }

 //   isProblematicAreas = false;


  //  debugPrint(vNormals.size());
  //  debugPrint(dim[1]*dim[2]);





    //-------------------------------------------------------------------------


    // Write the stl file to disk
    if (vtkTools::writeSTL(appendFilter->GetOutputPort(), "tmpMerge.stl"))
    {
        if (isProblematicAreas)
        {
            normal_model = normal;
            return PROBLEMATIC_AREAS;
        }
        return NO_PROBLEMATIC_AREAS;
    }
    else
        return SUBDIVIDE_ERROR;

}

///////////Ya no se usa//////////////////
void Defect::subdivideArea()
{

    //SELECT PART OF STL MODEL AND SUBDIVIDE!
    mimmo::MimmoGeometry *mimmoS = new mimmo::MimmoGeometry();
    mimmoS->setIOMode(IOMode::WRITE);
    mimmoS->setWriteDir("./");
    mimmoS->setWriteFileType(FileType::STL);
    mimmoS->setWriteFilename("tmpBeforeSubdivide");

    //Instantiation of a Selection By Box block.
    //Setup of span and origin of cube.
    darray3E origin = {posxyz.x(), posxyz.y(), posxyz.z()};
    mimmo::SelectionByBox * boxSel = new mimmo::SelectionByBox();
    boxSel->setOrigin(origin);

    //darray3E span_l = lattice->getSpan();
    darray3E span_bs;

    span_bs= darray3E{lattice->getSpan()[0]+160,lattice->getSpan()[1]+160,lattice->getSpan()[2]+160};

    boxSel->setSpan(span_bs);
    boxSel->setRefSystem(axes);
    boxSel->setPlotInExecution(true);
    boxSel->setOutputPlot("");

    //BOX SEL 2 ------------------------------------------------------------

    mimmo::SelectionByBox * boxSel2 = new mimmo::SelectionByBox();
    boxSel2->setOrigin(origin);
    boxSel2->setSpan(span_bs);
    boxSel2->setRefSystem(axes);
    boxSel2->setPlotInExecution(true);
    boxSel2->setOutputPlot("");
    boxSel2->setDual(true);

    mimmo::MimmoGeometry *mimmoS2 = new mimmo::MimmoGeometry();
    mimmoS2->setIOMode(IOMode::WRITE);
    mimmoS2->setWriteDir("./");
    mimmoS2->setWriteFileType(FileType::STL);
    mimmoS2->setWriteFilename("tmpBeforeSubdivide2");

    //Instantiation of a Refine Geometry block.
    //Setup refining.
    mimmo::RefineGeometry * refine2 = new mimmo::RefineGeometry();
    refine2->setRefineType(mimmo::RefineType::TERNARY);
    refine2->setRefineSteps(1);
    refine2->setSmoothingSteps(2);

    mimmo::pin::addPin(mimmo0, refine2, M_GEOM, M_GEOM);
    mimmo::pin::addPin(mimmo0, boxSel2, M_GEOM, M_GEOM);
    mimmo::pin::addPin(boxSel2, mimmoS2, M_GEOM, M_GEOM);


    mimmo::Chain ch12;
    ch12.addObject(mimmo0);
    ch12.addObject(refine2);
    ch12.addObject(boxSel2);
    ch12.addObject(mimmoS2);
    ch12.exec(true);

    //Free memory
    delete(mimmoS2);
    delete(refine2);
    delete(boxSel2);


    //----------------------------------------------------------------------

    //Instantiation of a Refine Geometry block.
    //Setup refining.
    mimmo::RefineGeometry * refine = new mimmo::RefineGeometry();
    refine->setRefineType(mimmo::RefineType::TERNARY);
    refine->setRefineSteps(1);
    refine->setSmoothingSteps(2);

    mimmo::pin::addPin(mimmo0, refine, M_GEOM, M_GEOM);
    mimmo::pin::addPin(mimmo0, boxSel, M_GEOM, M_GEOM);
    mimmo::pin::addPin(boxSel, mimmoS, M_GEOM, M_GEOM);

    mimmo::Chain ch1;
    ch1.addObject(mimmo0);
    ch1.addObject(refine);
    ch1.addObject(boxSel);
    ch1.addObject(mimmoS);
    ch1.exec(true);

    //Free memory
    delete(mimmoS);
    delete(refine);
    delete(boxSel);



    //auto vS = mimmoS->getGeometry()->getNVertices();

    //SUBDIVIDE
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName("tmpBeforeSubdivide.stl");
    reader->Update();

    //SEGUIR AQUÍ----
    vtkSmartPointer<vtkPolyDataAlgorithm> subdivisionFilter;
    subdivisionFilter = vtkSmartPointer<vtkAdaptiveSubdivisionFilter>::New();
//    auto a = dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->GetMaximumTriangleArea();
//    std::cout << a << std::endl;

//    dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->SetMaximumTriangleArea(500000);

//    a = dynamic_cast<vtkAdaptiveSubdivisionFilter*>(subdivisionFilter.GetPointer())->GetMaximumTriangleArea();
//    std::cout << a << std::endl;

    //dynamic_cast<vtkAdaptiveSubdivisionFilter *> (subdivisionFilter.GetPointer());

    //dynamic_cast<vtkSubdivisionFilter *> (subdivisionFilter.GetPointer())->SetNumberOfSubdivisions();
    vtkSmartPointer<vtkPolyData> originalMesh; //TRANSFORM STL MIMMO TO VTK OBJECT DE ALGUNA MANERA
    vtkSmartPointer<vtkTriangleFilter> triangles = vtkSmartPointer<vtkTriangleFilter>::New();
    triangles->SetInputConnection(reader->GetOutputPort());
    triangles->Update();
    originalMesh = triangles->GetOutput();

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(originalMesh);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOff();
    normalGenerator->Update();


    originalMesh = normalGenerator->GetOutput();
    auto normals = originalMesh->GetPointData()->GetNormals();

 //   auto points = originalMesh->GetPointData();



    //--- GET NORMAL MODEL
    auto size = normals->GetNumberOfTuples();
    double* values;
    double normal_mean[3] = {0,0,0};
    int signo_x=1,signo_y=1,signo_z=1;
    int n_x=0,n_y=0,n_z=0;

    //QVector<QVector3D> points;
    for (int i=0; i<normals->GetNumberOfTuples(); i++)
    {
       values = normals->GetTuple3(i);
       normal_mean[0] = normal_mean[0] + fabs(values[0]);
       normal_mean[1] = normal_mean[1] + fabs(values[1]);
       normal_mean[2] = normal_mean[2] + fabs(values[2]);

       if(values[0]<0)n_x++;
       if(values[1]<0)n_y++;
       if(values[2]<0)n_z++;

     //  points.push_back(QVector3D(values[0], values[1], values[2]));
    }

    if(n_x > size/2)signo_x=-1;
    if(n_y > size/2)signo_y=-1;
    if(n_z > size/2)signo_z=-1;

    normal_mean[0] = signo_x*normal_mean[0]/size;
    normal_mean[1] = signo_y*normal_mean[1]/size;
    normal_mean[2] = signo_z*normal_mean[2]/size;

    normal_model = QVector3D(normal_mean[0],normal_mean[1],normal_mean[2]);


    //--
//    FitPlane3D plane(points);
//    plane.build();
//    auto n = plane.getNormal();
//    std::cout << n[0] <<". "<< n[1] << ". " << n[2] << std::endl;

    //--

    //---


    subdivisionFilter->SetInputData(originalMesh);
    subdivisionFilter->Update();


    //---------

    vtkSmartPointer<vtkSTLReader> reader2 = vtkSmartPointer<vtkSTLReader>::New();
    reader2->SetFileName("tmpBeforeSubdivide2.stl");
    reader2->Update();

//    vtkSmartPointer<vtkPolyData> input1 = vtkSmartPointer<vtkPolyData>::New();
//    vtkSmartPointer<vtkPolyData> input2 = vtkSmartPointer<vtkPolyData>::New();

//    input1->ShallowCopy(subdivisionFilter->GetOutput());
//    input2->ShallowCopy(reader2->GetOutput());

    //Append the two meshes
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInputData(subdivisionFilter->GetOutput());
    appendFilter->AddInputData(reader2->GetOutput());

     // Remove any duplicate points.
//    vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
//    cleanFilter->SetInputConnection(appendFilter->GetOutputPort());
  //  cleanFilter->Update();


    //---------

    // Write the stl file to disk
    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileTypeToBinary();
    //stlWriter->SetFileName("tmpAfterSubdivide.stl");
    stlWriter->SetFileName("tmpMerge.stl");
    stlWriter->SetInputConnection(appendFilter->GetOutputPort()); //mesh //cleanFilter
    stlWriter->Write();

    mimmoMerge = new mimmo::MimmoGeometry();
    mimmoMerge->setIOMode(IOMode::CONVERT);
    mimmoMerge->setReadFileType(FileType::STL);
    mimmoMerge->setReadDir("./");
    mimmoMerge->setReadFilename("tmpMerge");


    //------------------------


}
/////////////////////////////////////////

void Defect::destructLattice()
{
    // Clean up & exit;
    delete applier;
    delete lattice;
    delete input;
    delete output;
    delete mimmo0;
    delete mimmo1;
  //  delete mimmoMerge; //CAMBIO
    lattice = NULL;
    applier = NULL;
    input   = NULL;
    output   = NULL;
    mimmo0  = NULL;
    mimmo1  = NULL;
  //  mimmoMerge = NULL; //CAMBIO

    vLattice.clear();
    vInput.clear();
    vDispl.clear();
    vNormalsLattice.clear();
    vOriginsLattice.clear();
    vBoundings_divisions.clear();
    vNormals.clear();



}

//////////No se usa//////////////////////
double Defect::getProfile(QVector2D p, vtkSmartPointer<vtkPolyData> mesh, int id_mesh)
{
    QVector<double> profiles;
    QVector<int> id_m;
    for(int i=0; i<mesh->GetNumberOfPoints();i++)
    {
        auto p_m = mesh->GetPoint(i);
        if(abs (p.x()-p_m[0]) < 1 && abs(p.y()-p_m[2])<1)
        {
            profiles.push_back(p_m[1]);
            id_m.push_back(i);
        }
    }

    int id=-1;
    float min = abs(profiles[0]-posxyz.y());
    for (int i=0; i< profiles.size();i++)
    {
        if(abs(profiles[i]-posxyz.y()))
        {
            id = i;
            min = abs(profiles[0]-posxyz.y());
        }
    }

    if (id != -1)
    {
        double aux = profiles[id];
       // debugPrint("Eliminando");
       // mesh->DeletePoint(id_m[id]);
        id_mesh = id_m[id];
        return aux;
    }
    else return -1;
}
/////////////////////////////////////////

//////////No se usa//////////////////////
bool Defect::insertDefectVTK(std::string stl_file, std::string stl_file_new)
{

    std::stringstream name_file;
    name_file << stl_file << ".stl";
    std::string name_aux_tmp = name_file.str();

    //Load model in VTK
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(name_aux_tmp.c_str());
    reader->Update();

    //Load DEFECT in VTK
    vtkSmartPointer<vtkSTLReader> reader_defect = vtkSmartPointer<vtkSTLReader>::New();
    reader_defect->SetFileName("PRUEBA_SURFACE.stl");
    reader_defect->Update();

    vtkSmartPointer<vtkPolyData> originalMesh;
    vtkSmartPointer<vtkTriangleFilter> triangles = vtkSmartPointer<vtkTriangleFilter>::New();
    triangles->SetInputConnection(reader->GetOutputPort());
    triangles->Update();
    originalMesh = triangles->GetOutput();

    //Select surface
    vtkNew<vtkIdTypeArray> ids;
    for(int i=0; i<originalMesh->GetNumberOfPoints();i++)
    {
        auto p = originalMesh->GetPoint(i);
        if(isPinVolumen2(QVector3D(p[0],p[1],p[2]),true))
          ids->InsertNextValue(i);
    }

    debugPrint("Seleccionado");


    vtkNew<vtkSelectionNode> selectionNode;
    selectionNode->SetFieldType(vtkSelectionNode::POINT);
    selectionNode->SetContentType(vtkSelectionNode::INDICES);
    selectionNode->SetSelectionList(ids);
    selectionNode->GetProperties()->Set(vtkSelectionNode::CONTAINING_CELLS(), 1);

    vtkNew<vtkSelection> selection;
    selection->AddNode(selectionNode);

    vtkNew<vtkExtractSelection> extractSelection;

    extractSelection->SetInputConnection(0, reader->GetOutputPort());
    extractSelection->SetInputData(1, selection);
    extractSelection->Update();

    // In selection
    vtkNew<vtkUnstructuredGrid> selected;
    selected->ShallowCopy(extractSelection->GetOutput());

    if (DEBUG_SUBDIVIDE_AREA)
    {
        std::cout << "AYAYAYAYAYAYAYAYA" << std::endl;
       vtkTools::printNPointsCellsPoly_debug(static_cast<vtkUnstructuredGrid*>(selected));
    }


    if (selected->GetNumberOfPoints() == 0)
        return false;

    vtkSmartPointer<vtkGeometryFilter> geometryFilter = vtkSmartPointer<vtkGeometryFilter>::New();
    geometryFilter->SetInputData(selected);
    geometryFilter->Update();

    vtkSmartPointer<vtkPolyData> polydata = geometryFilter->GetOutput();
    vtkSmartPointer<vtkTriangleFilter> triangles_ = vtkSmartPointer<vtkTriangleFilter>::New();
    triangles_->SetInputData(polydata);
    triangles_->Update();


    // Get points that are NOT in the selection
    selectionNode->GetProperties()->Set(vtkSelectionNode::INVERSE(), 1); // invert the selection
    extractSelection->Update();

    vtkNew<vtkUnstructuredGrid> notSelected;
    notSelected->ShallowCopy(extractSelection->GetOutput());

    if (DEBUG_SUBDIVIDE_AREA)
    {
        vtkTools::printNPointsCellsPoly_debug(static_cast<vtkUnstructuredGrid*>(notSelected));
        std::cout << "AYAYAYAYAYAYAYAYA" << std::endl;

    }

    vtkSmartPointer<vtkGeometryFilter> geometryFilter_not = vtkSmartPointer<vtkGeometryFilter>::New();
    geometryFilter_not->SetInputData(notSelected);
    geometryFilter_not->Update();

    vtkSmartPointer<vtkPolyData> polydata_not = geometryFilter_not->GetOutput();
    vtkSmartPointer<vtkTriangleFilter> triangles_not = vtkSmartPointer<vtkTriangleFilter>::New();
    triangles_not->SetInputData(polydata_not);
    triangles_not->Update();


    // Write the stl file to disk (ONLY FOR DEBUGGING PURPOSES)
    if (DEBUG_SUBDIVIDE_AREA)
    {
        vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
        stlWriter->SetFileTypeToBinary();
        stlWriter->SetFileName("DEFECT_AREA_selection.stl");
        stlWriter->SetInputConnection(geometryFilter->GetOutputPort()); //mesh //cleanFilter
        stlWriter->Write();
    }


    //Append the two meshes
    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInputData(reader->GetOutput());
    appendFilter->AddInputData(reader_defect->GetOutput());
    appendFilter->Update();

    // Write the stl file to disk
    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileTypeToBinary();
    stlWriter->SetFileName("tmpMerge.stl");
    stlWriter->SetInputConnection(appendFilter->GetOutputPort()); //mesh //cleanFilter
    stlWriter->Write();

    return true;

}
/////////////////////////////////////////

