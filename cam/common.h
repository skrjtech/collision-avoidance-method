#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

/* --- 定数設定 --- */

#define PI 3.14159265359	        //円周率[rad]

/* --- 人間のパラメータ --- */
#define HUMAN_RADIUS        0.2 //人間の半径（肩幅から算出）[m]
#define HUMAN_WEIGHT        4.4 //人間の衝突部位の質量（頭部）[kg]
#define HUMAN_MAX_VELOCITY  3.6 //人間の最大速度[m/s]
#define SGIMA_VELOCITY      0.4 //人間の移動速度の標準偏差
#define SIGMA_THETA         0.0 //人間の進行方向の標準偏差

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
#define ROBOT_Y_INIT    4.0     // 初期位置Y [m]
#define ROBOT_RAD_INIT  PI / 2  // 初期方向 [rad]
#define GOAL_X          2.0     // 目的位置X [m]
#define GOAL_Y          4.0     // 目的位置Y [m]
#define GOAL_DADIUS     0.35    // 目的位置での接触判定距離 [m]

#endif
