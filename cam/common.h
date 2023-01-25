#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

/* --- 定数設定 --- */

#define PI 3.14159265359	//円周率[rad]

/* --- 人間のパラメータ --- */
#define humanC = 0.2;		//人間の半径（肩幅から算出）[m]
#define humanW = 4.4;		//人間の衝突部位の質量（頭部）[kg]
#define humanVMax = 3.6;		//人間の最大速度[m/s]
#define sigV = 0.4;			//人間の移動速度の標準偏差
#define sigTheta = 0.0;		//人間の進行方向の標準偏差

/* --- ロボットのパラメータ --- */
#define robC = 0.35;			//ロボットの半径[m]
#define robW = 15;			//ロボットの質量[kg]
#define robVMax = 0.7;		//ロボットの最大速度[m/s]
#define robRadMax = 6.28;	//ロボットの最大旋回速度[rad/s]

/* --- 対人回避軌道計画のパラメータ --- */
#define ts = 0.5;			//単位タイムステップ時間[s]
#define nts = 6;				//タイムステップ数
#define N = 15 + 1;		//タイムステップ毎に生成する回避軌道経由点候補の数（0～15）

/* --- 配置に関するパラメータ --- */
#define areaX = 4.0, areaY = 4.0;		//ロボットの移動できる範囲[m]
#define robXInit = 2.0, robYInit = 4.0;	//ロボットの初期位置[m]
#define robRotInit = PI / 2;				//ロボットの初期方向[rad]
#define goalX = 2.0, goalY = 4.0;		//ロボットの目的位置[m]
#define goalC = 0.35;					//ロボットが目的位置に接触したと判定する距離（＝robC）[m]

#endif
