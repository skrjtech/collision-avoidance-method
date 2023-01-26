#ifndef INCLUDE_OBJECT_HEADER
#define INCLUDE_OBJECT_HEADER

/* XYZ */
typedef struct _Position Position;      // 座標
typedef struct _Posture Posture;        // 姿勢
typedef struct _Radian Radian;          // 角度
typedef struct _VPosition VPosition;    // 速度
typedef struct _VRadian VRadian;        // 回転速度


typedef struct _Object *Object;

Position PositionCreate(double x, double y, double z);
// X[mm], Y[mm], Z[mm]
double PositionGetX(Position pos);
double PositionGetY(Position pos);
double PositionGetZ(Position pos);

Posture PostureCreate(double x, double y, double z);
// X[deg], Y[deg], Z[deg]
double PostureGetX(Posture pos);
double PostureGetY(Posture pos);
double PostureGetZ(Posture pos);

Radian RadianCreate(double x, double y, double z);
// X[rad], Y[rad], Z[rad]
double RadianGetX(Radian pos);
double RadianGetY(Radian pos);
double RadianGetZ(Radian pos);

VPosition VPositionCreate(double x, double y, double z);
// X[mm/s], Y[mm/s], Z[mm/s]
double VPositionGetX(VPosition pos);
double VPositionGetY(VPosition pos);
double VPositionGetZ(VPosition pos);

VRadian VRadianCreate(double x, double y, double z);
// X[rad/s], Y[rad/s], Z[rad/s]
double VRadianGetX(VPosition pos);
double VRadianGetY(VPosition pos);
double VRadianGetZ(VPosition pos);


Object ObjectCreate(void);

// 値の表示
void ObjectPrint(Object obj);
// 物体移動
void ObjectMove(Object obj);
// 物体の破壊
Object ObjectDestroy(Object obj);

#endif