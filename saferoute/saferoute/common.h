#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

#include <math.h>

/* --- 定数設定 --- */

// #define PI 3.14159265359	            // 円周率 [rad]
#define DEGREE(x) ((x) * M_PI / 180)    // 角度から度に変換

/* --- センサー パラメータ --- */
#define SENSOR_MAX_DISTANCE 10000   // 最大距離 [mm]
#define SENSOR_MAX_RADIAN   270     // 最大角度 [rad]
#define SENSOR_MAX_HEIGHT   177.4   // 地面からの最大高さ [mm]

/* --- 領域格納用配列数 パラメータ --- */
#define ACCURACY_DISTANCE 10 // 測定精度 [mm]
// #define ARRAY_HEIGHT (SENSOR_MAX_DISTANCE / ACCURACY_DISTANCE) // 高さ数
// #define ARRAY_WIDTH  (SENSOR_MAX_DISTANCE / ACCURACY_DISTANCE) // 幅数
#define ARRAY_HEIGHT 10000
#define ARRAY_WIDTH  10000

/* --- 測定可能領域 パラメータ --- */
#define AREA_MAX_HEIGHT     10000 // 最大高さ [mm]
#define AREA_MAX_WIDTH      10000 // 最大幅 [mm]
#define AREA_X_MAX_RADIAN    (SENSOR_MAX_DISTANCE * cos(DEGREE(90))) // X軸最大測定領域   
#define AREA_Y_MAX_RADIAN    (SENSOR_MAX_DISTANCE * sin(DEGREE(90))) // Y軸最大測定領域

/* --- 物体認識領域 パラメータ --- */
#define HANTEI_MAX_HEIGHT     7000 // 最大高さ [mm]
#define HANTEI_MAX_WIDTH      7000 // 最大幅 [mm]
#define HANTEI_X_MAX_RADIAN    (HANTEI_MAX_HEIGHT * cos(DEGREE(90))) // X軸最大測定領域   
#define HANTEI_Y_MAX_RADIAN    (HANTEI_MAX_HEIGHT * sin(DEGREE(90))) // Y軸最大測定領域

/* --- 人間のパラメータ --- */
#define HUMAN_RADIUS        0.2 //人間の半径（肩幅から算出）[m]
#define HUMAN_WEIGHT        4.4 //人間の衝突部位の質量（頭部）[kg]
#define HUMAN_MAX_VELOCITY  3.6 //人間の最大速度[m/s]
#define SIGMA_VELOCITY      0.4 //人間の移動速度の標準偏差


/* --- ロボットのパラメータ --- */
#define ROBOT_RADIUS        0.35 //ロボットの半径[m]
#define ROBOT_WEIGHT        15	 //ロボットの質量[kg]
#define ROBOT_MAX_VELOCITY  0.7  //ロボットの最大速度[m/s]
#define ROBOT_MAX_RAD       6.28 //ロボットの最大旋回速度[rad/s]

/* --- 対人回避軌道計画のパラメータ --- */
#define TIMESTEP            0.5			//単位タイムステップ時間[s]
#define TIMESTEP_NUM        6				//タイムステップ数
#define EVERY_TIMESTEP_NUM  15 + 1		//タイムステップ毎に生成する回避軌道経由点候補の数（0～15）

/* --- 配置に関するパラメータ --- */
// ロボット移動領域範囲
#define AREA_X          4.0     // エリアX [m]
#define AREA_Y          4.0     // エリアY [m]
#define ROBOT_X_INIT    2.0     // 初期位置X [m]
#define ROBOT_Y_INIT    0.0     // 初期位置Y [m]
#define ROBOT_RAD_INIT  PI / 2  // 初期方向 [rad]
#define GOAL_X          2.0     // 目的位置X [m]
#define GOAL_Y          4.0     // 目的位置Y [m]
#define GOAL_DISTANCE     0.35    // 目的位置での接触判定距離 [m]

#endif