#include "boundingbox.h"

BoundingBox::BoundingBox()
{}



BoundingBox::BoundingBox(Triangle* tris) //HACER ESTO
{
    QVector3Dd min = tris->v0;
    QVector3Dd max = tris->v0;

    if (tris->v0.x() < min.x()) min.setX(tris->v0.x());
    if (tris->v0.y() < min.y()) min.setY(tris->v0.y());
    if (tris->v0.z() < min.z()) min.setZ(tris->v0.z());
    if (tris->v0.x() > max.x()) max.setX(tris->v0.x());
    if (tris->v0.y() > max.y()) max.setY(tris->v0.y());
    if (tris->v0.z() > max.z()) max.setZ(tris->v0.z());

    if (tris->v1.x() < min.x()) min.setX(tris->v1.x());
    if (tris->v1.y() < min.y()) min.setY(tris->v1.y());
    if (tris->v1.z() < min.z()) min.setZ(tris->v1.z());
    if (tris->v1.x() > max.x()) max.setX(tris->v1.x());
    if (tris->v1.y() > max.y()) max.setY(tris->v1.y());
    if (tris->v1.z() > max.z()) max.setZ(tris->v1.z());

    if (tris->v2.x() < min.x()) min.setX(tris->v2.x());
    if (tris->v2.y() < min.y()) min.setY(tris->v2.y());
    if (tris->v2.z() < min.z()) min.setZ(tris->v2.z());
    if (tris->v2.x() > max.x()) max.setX(tris->v2.x());
    if (tris->v2.y() > max.y()) max.setY(tris->v2.y());
    if (tris->v2.z() > max.z()) max.setZ(tris->v2.z());


    //Calcular min y max para los triangulos
    minPos = QVector3Dd(min.x() - 1, min.y()-1, min.z()-1);
    maxPos = QVector3Dd(max.x() + 1, max.y()+1, max.z()+1);
}

BoundingBox::BoundingBox(std::vector<Triangle*> tris)
{
    QVector3Dd min = tris[0]->v0;
    QVector3Dd max = tris[0]->v0;
    for (uint i=1; i<tris.size();i++)
    {
        if (tris[i]->v0.x() < min.x()) min.setX(tris[i]->v0.x());
        if (tris[i]->v0.y() < min.y()) min.setY(tris[i]->v0.y());
        if (tris[i]->v0.z() < min.z()) min.setZ(tris[i]->v0.z());
        if (tris[i]->v0.x() > max.x()) max.setX(tris[i]->v0.x());
        if (tris[i]->v0.y() > max.y()) max.setY(tris[i]->v0.y());
        if (tris[i]->v0.z() > max.z()) max.setZ(tris[i]->v0.z());

        if (tris[i]->v1.x() < min.x()) min.setX(tris[i]->v1.x());
        if (tris[i]->v1.y() < min.y()) min.setY(tris[i]->v1.y());
        if (tris[i]->v1.z() < min.z()) min.setZ(tris[i]->v1.z());
        if (tris[i]->v1.x() > max.x()) max.setX(tris[i]->v1.x());
        if (tris[i]->v1.y() > max.y()) max.setY(tris[i]->v1.y());
        if (tris[i]->v1.z() > max.z()) max.setZ(tris[i]->v1.z());

        if (tris[i]->v2.x() < min.x()) min.setX(tris[i]->v2.x());
        if (tris[i]->v2.y() < min.y()) min.setY(tris[i]->v2.y());
        if (tris[i]->v2.z() < min.z()) min.setZ(tris[i]->v2.z());
        if (tris[i]->v2.x() > max.x()) max.setX(tris[i]->v2.x());
        if (tris[i]->v2.y() > max.y()) max.setY(tris[i]->v2.y());
        if (tris[i]->v2.z() > max.z()) max.setZ(tris[i]->v2.z());
    }

    //Calcular min y max para los triangulos
    minPos = QVector3Dd(min.x() - 15, min.y()-15, min.z()-15);
    maxPos = QVector3Dd(max.x() + 15, max.y()+15, max.z()+15);

}

int BoundingBox::longestAxis()
{
    float d_x= abs(maxPos.x() - minPos.x());
    float d_y= abs(maxPos.x() - minPos.x());
    float d_z= abs(maxPos.x() - minPos.x());

    if (d_x > d_y && d_x > d_z)
        return 0;

    else if (d_y > d_x && d_y > d_z)
        return 1;

    else
        return 2;

}
