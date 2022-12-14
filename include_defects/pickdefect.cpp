#include "pickdefect.h"

PickDefect::PickDefect(QVector3D posxyz_,float width_, float depth_, QVector3D normal_)
    :Defect(posxyz_, width_, width_, depth_, normal_, 0)
{

}

void PickDefect::fillLattice()
{
    int ndof = lattice->getNNodes();
    displ = dvecarr3E(ndof, darray3E{0,0,0});

    darray3E span = lattice->getSpan();
    iarray3E dim = lattice->getDimension();

    float cte_x;
    float cte_y;
    //float sigmax, sigmay; //0.06,0.01
    //sigmax = span[2]/6;
    //sigmay = span[1]/6;

    cte_x = span[2]/dim[2];
    cte_y = span[1]/dim[1];

    float rx = span[2]/2;
    float ry = span[1]/2;

    std::cout << "Rx: " << rx << " Ry: " << ry << std::endl;


    float sigma;

    if(rx>=ry) sigma=ry/7;
    else sigma=rx/7;

    for (int i=0; i<ndof; i++){
        int l0,l1,l2;
        int index = lattice->accessGridFromDOF(i);
        lattice->accessPointIndex(index,l0,l1,l2);

        float x,y;

        x = - span[2]/2 + cte_x*l2;
        y = - span[1]/2 + cte_y*l1;

        if(x==0)x=0.01;

        //float alpha = fmod(std::atan2(x,y), 2*PI);
        float alpha = fmod(std::atan2(x,y) + 2*PI, 2*PI) ;
        //(alpha > 0 ? alpha : (2*PI + alpha)) * 360 / (2*PI);

        float r = x/std::sin(alpha);
        float z=depth/std::exp(r/(sigma));

       // std::cout << r << std::endl;

        //float z = depth*(1- std::sqrt( (x/(rx-rx/8) * x/(rx-rx/8)) + (y/(ry-ry/8) * y/(ry-ry/8)) ));
       // if (z<0) z = 0;
        displ[i][0]= z;

    }

    // Set Generic input block with the displacements defined above.
    input = new mimmo::GenericInput();
    input->setReadFromFile(false);
    input->setInput(displ);
    // Set Generic output block to write the displacements defined above.
    output = new mimmo::GenericOutput();
    output->setFilename("manipulators_output_00003.csv");
    output->setCSV(true);
}
void PickDefect::createDefectVTK()
{}
