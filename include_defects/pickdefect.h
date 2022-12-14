#ifndef PICKDEFECT_H
#define PICKDEFECT_H

#include "defect.h"

class PickDefect : public Defect
{
public:
    PickDefect();
    PickDefect(QVector3D posxyz_, float width_, float depth_, QVector3D normal_);

    void fillLattice();
    void createDefectVTK();

};

#endif // PICKDEFECT_H
