#ifndef INCLUDE_OBJECT_HEADER
#define INCLUDE_OBJECT_HEADER

typedef struct _Object *Object;

// X[mm], Y[mm], Z[mm]
double ObjectGetX(Object obj);
double ObjectGetY(Object obj);
double ObjectGetZ(Object obj);

// O[deg], A[deg], T[deg]
double ObjectGetO(Object obj);
double ObjectGetA(Object obj);
double ObjectGetZ(Object obj);

// RX[rad], RY[rad], RZ[rad]
double ObjectGetRX(Object obj);
double ObjectGetRY(Object obj);
double ObjectGetRZ(Object obj);

// VX[mm/s], VY[mm/s], VZ[mm/s]
double ObjectGetVX(Object obj);
double ObjectGetVY(Object obj);
double ObjectGetVZ(Object obj);

// WEIGHT[kg]
double ObjectGetW(Object obj);

// X[mm], Y[mm], Z[mm] 
// O[deg], A[deg], T[deg]
// RX[rad], RY[rad], RZ[rad]
// VX[mm/s], VY[mm/s], VZ[mm/s]
// WEIGHT[kg]
Object ObjectCreate(
    double x, double y, double z,
    double o, double a, double t,
    double rx, double ry, double rz,
    double vx, double vy, double vz,
    double w
);

// 値の表示
void ObjectPrint(Object obj);
// 物体移動
void ObjectMove(Object obj);
// 位置方向の値セット
void ObjectSetXYZ(Object obj, double x, double y, double z);
// 速度の値セット
void ObjectSetVXYZ(Object obj, double vx, double vy, double vz);
// 物体の破壊
Object ObjectDestroy(Object obj);

#endif