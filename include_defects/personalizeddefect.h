#ifndef PERSONALIZEDDEFECT_H
#define PERSONALIZEDDEFECT_H

#include "defect.h"


class PersonalizedDefect : public Defect
{
public:
    PersonalizedDefect();
    PersonalizedDefect(QVector3D posxyz_, float length_, float width_, float depth_, QVector3D normal_, float dir_deg_, int cols_raw_, int rows_raw_, float scale_depth_, QVector<float> z_);

    void fillLattice();
    void createDefectVTK();

 protected:
    int cols_raw;
    int rows_raw;
    //float scale_depth;
    QVector<float> z_raw;
};

#endif // PERSONALIZEDDEFECT_H
