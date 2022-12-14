#include "lossdefect.h"

LossDefect::LossDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_)
:Defect(posxyz_, length_, width_, depth_, normal_, dir_deg_)
{

}

void LossDefect::fillLattice()
{

    int ndof = lattice->getNNodes();
    dvecarr3E displ(ndof, darray3E{0,0,0});

    darray3E span = lattice->getSpan();
    iarray3E dim = lattice->getDimension();

    float cte_x;
    float cte_y;

    cte_x = span[2]/dim[2];
    cte_y = span[1]/dim[1];

    float xm_ = 0;
    float ym_ = 0;

    QVector<darray3E> points;
    dvecarr3E points_;
    dvecarr3E displ1;

    for (int i=0; i<ndof; i++){
        int l0,l1,l2;
        int index = lattice->accessGridFromDOF(i);
        lattice->accessPointIndex(index,l0,l1,l2);

        float x,y;
        x = - span[2]/2 + cte_x*l2;
        y = - span[1]/2 + cte_y*l1;

        if (l1>10 && l1<40 && l2 >10 && l2 < 40)
        {
            displ[i][0]= 50; //PONER UN CEEEEEERO

            points.push_back(lattice->getGlobalPoint(l0,l1,l2));
            points_.push_back(lattice->getGlobalPoint(l0,l1,l2));
            displ1.push_back(darray3E{0.25,0,0});

        }

    }




    // [X] Rellenar lattice
    // [X] Sacar coordenadas globales
    // [ ] Seleccionar esas coordenadas globales en mimmo0
    // [ ] Eliminarlas

    //Seleccionar con SelectionByPID
    //Eliminar parte seleccionada
    //Subdividir en algún momento¿?


    // Set Generic input block with the displacements defined above.
    input = new mimmo::GenericInput();
    input->setReadFromFile(false);
    input->setInput(displ);

    // Set Generic output block to write the displacements defined above.
    output = new mimmo::GenericOutput();
    output->setFilename("manipulators_output_00003.csv");
    output->setCSV(true);


/*

    mimmo::MimmoGeometry * mimmo0 = new mimmo::MimmoGeometry();
    mimmo0->setIOMode(IOMode::CONVERT);
    mimmo0->setReadDir("geodata");
    mimmo0->setReadFileType(FileType::STL);
    mimmo0->setReadFilename("sphere2");
    mimmo0->setWriteDir("./");
    mimmo0->setWriteFileType(FileType::STL);
    mimmo0->setWriteFilename("geohandlers_output_00003.0000");

    mimmo::MimmoGeometry * mimmo1 = new mimmo::MimmoGeometry();
    mimmo1->setIOMode(IOMode::CONVERT);
    mimmo1->setReadDir("geodata");
    mimmo1->setReadFileType(FileType::STL);
    mimmo1->setReadFilename("openBox");
    mimmo1->setWriteDir("./");
    mimmo1->setWriteFileType(FileType::STL);
    mimmo1->setWriteFilename("geohandlers_output_00003p1.0000");


    mimmo::SelectionByMapping  * mapSel1 = new mimmo::SelectionByMapping();
    mapSel1->setTolerance(5.0e-01);
    mapSel1->setPlotInExecution(true);


    mimmo::pin::addPin(mimmo0, mapSel1, M_GEOM, M_GEOM);
    mimmo::pin::addPin(mimmo1, mapSel1, M_GEOM, M_GEOM2);

    mimmo::Chain ch0;
    ch0.addObject(mimmo0);
    ch0.addObject(mimmo1);
    ch0.addObject(mapSel1);

    ch0.exec(true);
    */
}
void LossDefect::createDefectVTK()
{}
