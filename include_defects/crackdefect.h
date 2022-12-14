#ifndef DEFECT_CRACK_H
#define DEFECT_CRACK_H

#include "defect.h"

class CrackDefect : public Defect
{
public:
    CrackDefect();
    CrackDefect(QVector<QVector3D> points_, float depth_,float width_crack, QVector3D normal_, int n_columns_first, int n_columns_last);
    CrackDefect(QVector3D origin, QVector3D end, QVector<float> distributionL, QVector<float> distributionW, QVector3D normal, float depth_, float width_crack_, int n_columns_first_, int n_columns_last_);

    void fillLattice();
    void createDefectVTK();


protected:
    int n_points;
    QVector<QVector3D> points;
    int n_columns_first;
    int n_columns_last;
    int direction;
    float width_crack;

};

#endif // DEFECT_CRACK_H
