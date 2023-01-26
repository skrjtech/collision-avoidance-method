#include "Object.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct _Position { double x, y, z; } _Position;
typedef struct _Posture { double x, y, z; } _Posture;
typedef struct _Radian { double x, y, z; } _Radian;
typedef struct _VPosition { double x, y, z; } _VPosition;
typedef struct _VRadian { double x, y, z; } _VRadian;

typedef struct _Object
{
    /* data */
    Position position;   // [mm]   　位置
    Posture posture;     // [deg]　　姿勢
    Radian radian;       // [rad]  　回転
    VPosition vposition; // [mm/s] 　速度
    VRadian vradian;     // [rad/s]　回転速度 
    void 
} _Object;

Position PositionCreate(double x, double y, double z) {
    Position pos;
    pos.x = x;
    pos.y = y;
    pos.z = z;
    return pos;
}
// X[mm], Y[mm], Z[mm]
double PositionGetX(Position pos) { return pos.x; };
double PositionGetY(Position pos) { return pos.y; };
double PositionGetZ(Position pos) { return pos.z; };

// X[deg], Y[deg], Z[deg]
double PostureGetX(Posture pos) { return pos.x; };
double PostureGetY(Posture pos) { return pos.y; };
double PostureGetZ(Posture pos) { return pos.z; };

// X[rad], Y[rad], Z[rad]
double RadianGetX(Radian pos) { return pos.x; };
double RadianGetY(Radian pos) { return pos.y; };
double RadianGetZ(Radian pos) { return pos.z; };

// X[mm/s], Y[mm/s], Z[mm/s]
double VPositionGetX(VPosition pos) { return pos.x; };
double VPositionGetY(VPosition pos) { return pos.y; };
double VPositionGetZ(VPosition pos) { return pos.z; };

// X[rad/s], Y[rad/s], Z[rad/s]
double VRadianGetX(VPosition pos) { return pos.x; };
double VRadianGetY(VPosition pos) { return pos.y; };
double VRadianGetZ(VPosition pos) { return pos.z; };

Object ObjectCreate(void) {
    Object obj = (Object)malloc(sizeof(_Object) * 1);
    return obj;
}

// void ObjectPrint(Object obj) {
//     printf(" x: %.3f |  y: %.3f |  z: %.3f\n", obj->x, obj->y, obj->z);
//     printf(" o: %.3f |  a: %.3f |  t: %.3f\n", obj->o, obj->a, obj->t);
//     printf("rx: %.3f | ry: %.3f | rz: %.3f\n", obj->rx, obj->ry, obj->rz);
//     printf("vx: %.3f | vy: %.3f | vz: %.3f\n", obj->vx, obj->vy, obj->vz);
//     printf(" w: %.3f |\n", obj->w);
// }

void ObjectMove(Object obj) {
    obj->position.x += obj->vposition.x;
    obj->position.y += obj->vposition.y;
    obj->position.z += obj->vposition.z;
}   

Object ObjectDestroy(Object obj) {
    free(obj);
    return (NULL);
}