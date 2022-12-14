#include "trajectorynode.h"


trajectoryNode::trajectoryNode()
{

}

trajectoryNode::trajectoryNode(QVector3Dd pos_, QQuaternion q_)
{
    _pos =pos_;
    _q = q_;
}
