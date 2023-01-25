//アルゴリズムに用いられる定数

#define PI 3.14159265359

//人間のパラメータ
double humanC = 0.2;		//人間の半径（肩幅から算出）[m]
double humanW = 4.4;		//人間の衝突部位の質量（頭部）[kg]
double humanVMax = 3.6;		//人間の最大速度[m/s]
double sigV = 0.4;			//人間の移動速度の標準偏差
double sigTheta = 0.0;		//人間の進行方向の標準偏差

//ロボットのパラメータ
double robC = 0.35;			//ロボットの半径[m]
double robW = 15;			//ロボットの質量[kg]
double robVMax = 0.7;		//ロボットの最大速度[m/s]
double robRadMax = 6.28;	//ロボットの最大旋回速度[rad/s]

//対人回避軌道計画のパラメータ
double ts = 0.5;			//単位タイムステップ時間[s]
int nts = 6;				//タイムステップ数
const int N = 15 + 1;		//タイムステップ毎に生成する回避軌道経由点候補の数（0～15）

//配置に関するパラメータ
double areaX = 4.0, areaY = 4.0;		//ロボットの移動できる範囲[m]
double robXInit = 2.0, robYInit = 4.0;	//ロボットの初期位置[m]
double robRotInit = PI / 2;				//ロボットの初期方向[rad]
double goalX = 2.0, goalY = 4.0;		//ロボットの目的位置[m]
double goalC = 0.35;					//ロボットが目的位置に接触したと判定する距離（＝robC）[m]