#ifndef LOSSDEFECT_H
#define LOSSDEFECT_H

#include "defect.h"

class LossDefect : public Defect
{
public:
    LossDefect();
    LossDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_);

    void fillLattice();
    void createDefectVTK();


};

#endif // LOSSDEFECT_H
