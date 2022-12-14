#include "bumpdefect.h"

BumpDefect::BumpDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_)
    :Defect(posxyz_, length_, width_, depth_, normal_, dir_deg_)
{

}

void BumpDefect::createDefectVTK()
{}
void BumpDefect::fillLattice()
{

//    dvector1D knots;
//    ivector1D map;
//    lattice->returnKnotsStructure(2,knots,map);
//    std::cout << "KNOTS DIM: " << lattice->getKnotsDimension() << std::endl;
//    std::cout << "KNOTS: " << knots << std::endl;
//    std::cout << "MAP: " << map << std::endl;


    int ndof = lattice->getNNodes();
    displ =dvecarr3E(ndof, darray3E{0,0,0});

    darray3E span = lattice->getSpan();
    iarray3E dim = lattice->getDimension();

    float cte_x;
    float cte_y;
    float sigmax, sigmay; //0.06,0.01

    //sigmax = span[2]/6;
    //sigmay = span[1]/6;

    //Actualización del cálculo de sigmax y sigmay
    sigmax = sqrt(-(span[2]/2)*(span[2]/2)/(2*log(1e-6/fabs(depth))));
    sigmay = sqrt(-(span[1]/2)*(span[1]/2)/(2*log(1e-6/fabs(depth))));

    cte_x = span[2]/dim[2];
    cte_y = span[1]/dim[1];

    float xm_ = 0;
    float ym_ = 0;


    for (int i=0; i<ndof; i++){
        int l0,l1,l2;
        int index = lattice->accessGridFromDOF(i);
        lattice->accessPointIndex(index,l0,l1,l2);

        float x,y;
        x = - span[2]/2 + cte_x*l2;
        y = - span[1]/2 + cte_y*l1;

        float z= - depth*exp(-((x-xm_)*(x-xm_)/(2*sigmax*sigmax)) - ((y-ym_)*(y-ym_)/(2*sigmay*sigmay)));

       // if (abs(z)>abs(3*depth/4)) z=std::numeric_limits<double>::quiet_NaN();
       // float z=20;
       // if(y==0) z=std::numeric_limits<double>::quiet_NaN();
       // if(x==0) z=std::numeric_limits<double>::quiet_NaN();

       // float z = std::numeric_limits<double>::quiet_NaN();

        displ[i][0]= /*normal.x()**/ z;


    }
    debugPrint(displ[0][0]);




    // Set Generic input block with the displacements defined above.
    input = new mimmo::GenericInput();
    input->setReadFromFile(false);
    input->setInput(displ);

    // Set Generic output block to write the displacements defined above.
    output = new mimmo::GenericOutput();
    output->setFilename("manipulators_output_00003.csv");
    output->setCSV(true);
}
