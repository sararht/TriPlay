#ifndef BUMPDEFECT_H
#define BUMPDEFECT_H

#include "defect.h"

class BumpDefect : public Defect
{
public:
    BumpDefect();
    BumpDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_);

    void fillLattice();
    void createDefectVTK();

};

#endif // BUMPEDEFECT_H
