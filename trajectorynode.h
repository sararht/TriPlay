#ifndef TRAJECTORYNODE_H
#define TRAJECTORYNODE_H

#include <QQuaternion>
#include <QVector>
#include "qvector3dd.h"


class trajectoryNode
{
public:
    trajectoryNode();
    trajectoryNode(QVector3Dd pos_, QQuaternion q_);

    QVector3Dd pos() {return _pos;};
    QQuaternion q(){return _q;};

private:
    QVector3Dd _pos;
    QQuaternion _q;
};

#endif // TRAJECTORYNODE_H
