#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "triangle.h"
#include "qvector3dd.h"

class BoundingBox
{
public:
    BoundingBox();
    BoundingBox(Triangle* tris);
    BoundingBox(std::vector<Triangle*> tris);
    int longestAxis();

    QVector3Dd minPos;
    QVector3Dd maxPos;

};

#endif // BOUNDINGBOX_H
