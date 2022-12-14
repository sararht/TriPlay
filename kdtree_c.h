

#ifndef KDTREE_C_H
#define KDTREE_C_H


#include <math.h>

//VECTOR 3D
struct vector3D
{
    float x;
    float y;
    float z;
};

inline vector3D sumVector3D(vector3D *v1, vector3D *v2)
{
    vector3D v;
    v.x=v1->x+v2->x;
    v.y=v1->y+v2->y;
    v.z=v1->z+v2->z;

    return v;
}

inline vector3D diffVector3D(vector3D *v1, vector3D *v2)
{
    vector3D v;
    v.x=v1->x-v2->x;
    v.y=v1->y-v2->y;
    v.z=v1->z-v2->z;

    return v;
}

//TRIANGLE
struct Triangle
{
    vector3D v0;
    vector3D v1;
    vector3D v2;
};


inline vector3D getMidPointTriangle(Triangle *tris)
{
    vector3D midpoint;
    midpoint.x=tris->v0.x+tris->v1.x+tris->v2.x;
    midpoint.y=tris->v0.y+tris->v1.y+tris->v2.y;
    midpoint.z=tris->v0.z+tris->v1.z+tris->v2.z;
    return (vector3D{midpoint.x/3, midpoint.y/3, midpoint.z/3});
}

//BOUNDINGBOX
struct BoundingBox
{
    vector3D minPos;
    vector3D maxPos;
};

inline BoundingBox constructBoundingBox(Triangle *tris, int size)
{
    BoundingBox bbox;
    if (size <= 1)
    {
        bbox.minPos = tris->v0;
        bbox.maxPos = tris->v0;

        return bbox;
    }

    vector3D min = tris[0].v0;
    vector3D max = tris[0].v0;

    for (int i=0; i<size;i++)
    {
        auto a = tris[i].v0;
        if (tris[i].v0.x < min.x) min.x = (tris[i].v0.x);
        if (tris[i].v0.y < min.y) min.y = (tris[i].v0.y);
        if (tris[i].v0.z < min.z) min.z = (tris[i].v0.z);
        if (tris[i].v0.x > max.x) max.x = (tris[i].v0.x);
        if (tris[i].v0.y > max.y) max.y = (tris[i].v0.y);
        if (tris[i].v0.z > max.z) max.z = (tris[i].v0.z);
    }

    bbox.minPos = min;
    bbox.maxPos = max;

    return bbox;
}

inline int longestAxisBB(BoundingBox *bbox)
{
    float d_x= fabs(bbox->maxPos.x - bbox->minPos.x);
    float d_y= fabs(bbox->maxPos.y - bbox->minPos.y);
    float d_z= fabs(bbox->maxPos.z - bbox->minPos.z);

    if (d_x > d_y && d_x > d_z)
        return 0;

    else if (d_y > d_x && d_y > d_z)
        return 1;

    else
        return 2;
}

struct TriangleArray
{
    Triangle *tris;
    int reserved_size;
    int count;
};

inline TriangleArray constructTriangleArray(int size)
{
    TriangleArray array;
    array.tris = (Triangle*)malloc(sizeof(Triangle)*size);
    array.reserved_size=size;
    array.count=0;

    return array;
}

//KDTREE
struct KDNode
{
    KDNode *left;
    KDNode *right;
    TriangleArray *triangles;
    BoundingBox bbox;
};

inline void push(TriangleArray *array, Triangle *el)
{
    if (!(array->count < array->reserved_size))
    {

        array->reserved_size *= 2;
        array->tris = (Triangle*) realloc(array->tris,sizeof(Triangle)*array->reserved_size);
    }

    array->tris[array->count++] = *el;
}

inline KDNode* buildKDNode(TriangleArray *array, int size, int depth)
{
    KDNode *node = new KDNode;
    node->triangles = array;
    node->left = NULL;
    node->right = NULL;
    node->bbox = BoundingBox();

    if(size == 0)
        return node;

    if(size == 1)
    {
        constructBoundingBox(array->tris,size);
        node->bbox = constructBoundingBox(array->tris, size);
        node->left = new KDNode();
        node->right = new KDNode();
        node->left->triangles = new TriangleArray;
        node->right->triangles = new TriangleArray;
    }

    node->bbox = constructBoundingBox(array->tris,size);

    vector3D midpt{0,0,0};
    for (int i=0; i<size; i++)
    {
        Triangle a = array->tris[i];

        vector3D aux = getMidPointTriangle(&array->tris[i]);
        midpt = sumVector3D(&midpt,&aux);        
    }

    midpt = {midpt.x/size, midpt.y/size, midpt.z/size};

    TriangleArray *right_tris = new TriangleArray;
    TriangleArray *left_tris = new TriangleArray;

    *right_tris = constructTriangleArray(1);
    *left_tris = constructTriangleArray(1);

    int axis = longestAxisBB(&node->bbox);
    int size_left=0, size_right=0;

    for (int i=0; i<size;i++)
    {
        // split triangles based on their midpoints side of avg in longest axis

        //PUSH BACK Y ACTUALIZAR SIZES¿?
        switch(axis)
        {
            case 0:
                if (midpt.x >= getMidPointTriangle(&array->tris[i]).x)
                {

                    push(right_tris, &array->tris[i]);
                    size_right++;
                }
                else
                {
                    push(left_tris, &array->tris[i]);
                    size_left++;
                }
            break;

            case 1:
                if (midpt.y >= getMidPointTriangle(&array->tris[i]).x)
                {

                    push(right_tris, &array->tris[i]);
                    size_right++;
                }
                else
                {
                    push(left_tris, &array->tris[i]);
                    size_left++;
                }
            break;

            case 2:
                if (midpt.z >= getMidPointTriangle(&array->tris[i]).x)
                {

                    push(right_tris, &array->tris[i]);
                    size_right++;
                }
                else
                {
                    push(left_tris, &array->tris[i]);
                    size_left++;
                }
            break;
        }
    }

    if (size_left == 0 && size_right > 0)
        left_tris = right_tris;
    if (size_right == 0 && size_left > 0)
        right_tris = left_tris;

    if (depth <= 10)
    {
        node->left = buildKDNode(left_tris, size_left, depth+1);
        node->right = buildKDNode(right_tris, size_right, depth+1);
    }
    else
    {
        node->left = new KDNode;
        node->right = new KDNode;
        node->left->triangles = new TriangleArray;
        node->right->triangles = new TriangleArray;
    }

    return node;
}


#endif // KDTREE_C_H

