#include "Object.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct _Object
{
    /* data */
    double x, y, z;          // [mm]   　位置
    double o, a, t;          // [deg]　　姿勢
    double rx, ry, rz;       // [rad]  　回転
    double vx, vy, vz;       // [mm/s] 　速度
    double vrx, vry, vrz;    // [rad/s]　回転速度 
    double w; // [kg] 重さ
} _Object;

double ObjectGetX(Object obj) { return (obj->x); }
double ObjectGetY(Object obj) { return (obj->y); }
double ObjectGetZ(Object obj) { return (obj->z); }

double ObjectGetO(Object obj) { return (obj->o); }
double ObjectGetA(Object obj) { return (obj->a); }
double ObjectGetT(Object obj) { return (obj->t); }

double ObjectGetRX(Object obj) { return (obj->rx); }
double ObjectGetRY(Object obj) { return (obj->ry); }
double ObjectGetRZ(Object obj) { return (obj->rz); }

double ObjectGetVX(Object obj) { return (obj->vx); }
double ObjectGetVY(Object obj) { return (obj->vy); }
double ObjectGetVZ(Object obj) { return (obj->vz); }

double ObjectGetW(Object obj) { return (obj->w); }

Object ObjectCreate(
    double x, double y, double z,
    double o, double a, double t,
    double rx, double ry, double rz,
    double vx, double vy, double vz,
    double w
) {
    Object obj = (Object)malloc(sizeof(_Object));
    obj->x;
    obj->y;
    obj->z;

    obj->o;
    obj->a;
    obj->t;

    obj->rx;
    obj->ry;
    obj->rz;

    obj->vx;
    obj->vy;
    obj->vz;

    obj->w;

    return (obj);
}

void ObjectPrint(Object obj) {
    printf(" x: %.3f |  y: %.3f |  z: %.3f\n", obj->x, obj->y, obj->z);
    printf(" o: %.3f |  a: %.3f |  t: %.3f\n", obj->o, obj->a, obj->t);
    printf("rx: %.3f | ry: %.3f | rz: %.3f\n", obj->rx, obj->ry, obj->rz);
    printf("vx: %.3f | vy: %.3f | vz: %.3f\n", obj->vx, obj->vy, obj->vz);
    printf(" w: %.3f |\n", obj->w);
}

void ObjectMove(Object obj) {
    obj->x += obj->vx;
    obj->y += obj->vy;
    obj->z += obj->vz;
}

void ObjectSetXYZ(Object obj, double x, double y, double z) {
    obj->x = x;
    obj->y = y;
    obj->z = z;
}
void ObjectSetVXYZ(Object obj, double vx, double vy, double vz) {
    obj->vx = vx;
    obj->vy = vy;
    obj->vz = vz;
}

Object ObjectDestroy(Object obj) {
    free(obj);
    return (NULL);
}