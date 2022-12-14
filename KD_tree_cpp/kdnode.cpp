#include "kdnode.h"

KDNode::KDNode()
{}

bool sortOnX_(QVector3Dd v1, QVector3Dd v2)
{
    return v1.x() < v2.x();
}
bool sortOnY_(QVector3Dd v1, QVector3Dd v2)
{
    return v1.y() < v2.y();
}
bool sortOnZ_(QVector3Dd v1, QVector3Dd v2)
{
    return v1.z() < v2.z();
}

bool sortOnX(Triangle *t1, Triangle *t2)
{
    return t1->getMidPoint().x() < t2->getMidPoint().x();
}
bool sortOnY(Triangle *t1, Triangle *t2)
{
    return t1->getMidPoint().y() < t2->getMidPoint().y();
}
bool sortOnZ(Triangle *t1, Triangle *t2)
{
    return t1->getMidPoint().z() < t2->getMidPoint().z();
}

bool lowerBoundX(Triangle *t1, double value)
{
    return t1->getMidPoint().x() < value;
}
bool lowerBoundY(Triangle *t1, double value)
{
    return t1->getMidPoint().y() < value;
}
bool lowerBoundZ(Triangle *t1, double value)
{
    return t1->getMidPoint().z() < value;
}

/*
void KDNode::SAH()
{
    // traversal cost
 //   double KT;

    // triangle intersection cost
 //   double KI;

    //Área bbox
 //   double SA_V;
//    double SA_Vsub; //Área bbox sub

    // probability of hitting the subvoxel Vsub given that the voxel V was hit
    //double P_Vsub_given_V = SA_Vsub/SA_V;

    int NL, NR, PL, PR;
    // bias for the cost function s.t. it is reduced if NL or NR becomes zero
    double lambda;
    if((NL == 0 || NR == 0) && !(PL == 1 || PR == 1))
        lambda = 0.8f;
    else
        lambda = 1.0f;

    // cost C of a complete tree approximated using the cost CV of subdividing the voxel V with a plane p
    //double C = lambda * (KT + KI * (PL * NL + PR * NR));



}
*/

void KDNode::delete_KDNode()
{
    //Liberar memoria de los punteros a triángulo
    for (auto p : triangles)
        delete p;
    triangles.clear();

   // std::vector<Triangle*>().swap(trisModel);
    triangles.shrink_to_fit();

    left.clear();
    right.clear();

}


QSharedPointer<KDNode> KDNode::build(std::vector<Triangle*> &tris, int depth)
{
    QSharedPointer<KDNode> node(new KDNode);
    node->triangles = tris;
    node->left = NULL;
    node->right = NULL;
    node->bbox = BoundingBox();

    if(tris.size() == 0)
        return node;

    if(tris.size() ==1)
    {
        node->bbox = BoundingBox(tris[0]);
        node->left.reset(new KDNode);
        node->right.reset(new KDNode);
        node->left->triangles = std::vector<Triangle*>();
        node->right->triangles = std::vector<Triangle*>();
    }

    node->bbox = BoundingBox(tris);

    std::vector<QVector3Dd> median;
    QVector3Dd midpt(0,0,0);
    for (uint i=0; i<node->triangles.size(); i++)
        midpt = midpt + tris[i]->getMidPoint();

    midpt = midpt/node->triangles.size();

    std::vector<Triangle*> left_tris, right_tris;
    int axis = node->bbox.longestAxis();

    for (uint i=0; i<node->triangles.size();i++)
    {
        // split triangles based on their midpoints side of avg in longest axis
        switch(axis)
        {
            case 0:
                midpt.x() >= tris[i]->getMidPoint().x() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;

            case 1:
                midpt.y() >= tris[i]->getMidPoint().y() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;

            case 2:
                midpt.z() >= tris[i]->getMidPoint().z() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
            break;

        }
    }

    if (depth <= 10/*250*/ && (left_tris.size() > 10 && right_tris.size()>10) ) //2500, 5000, 5000; 20, 10, 10
    {
        node->left = build(left_tris, depth+1);
        node->right = build(right_tris, depth+1);
    }
    else
    {
        node->left.reset(new KDNode);
        node->right.reset(new KDNode);
        node->left->triangles = std::vector<Triangle*>();
        node->right->triangles = std::vector<Triangle*>();
    }

    return node;

}
