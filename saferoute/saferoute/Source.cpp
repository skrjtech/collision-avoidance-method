#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>

#include <string>

#define _USE_MATH_DEFINES //数値演算定数が定義されたヘッダファイルの読み込み

#define PI 3.14159265359

FILE* fp; //FILEポインタの宣言

using std::string;


double humanX = 0.0, humanY = 0.0;	        //人間の現在位置[m]
double humanTheta = 0.0;		        //人間の現在の方向[°]
double xP = 0.0, yP = 0.0;			        //人間の移動予測点[ml
double xV = 0.0;					//ロボットの予測速度[m/s]
double humanC = 0.4;	        //人間の半径(肩幅)[m]
double robC = 0.35;				//ロボットの半径[m]

double colliC = 0;				//人間とロボットの接触距離[m]

double humanV = 1.4 / 2;	        //人間の移動速度[m/s]
									//double humanV = 1.4;


double humanVP = 0.0;			        //人間の移動ノルム

double humanThetaP = 0.0;		        //人間の現在位置からみたある点の方向[°]
double robThetaP = 0.0;					//ロボットのある位置から見た現在の人間の方向[°]

double sigV = 0.8 / 2;	        //人間の移動速度の標準偏差
double sigTheta = 0.0;		        //人間の進行方向の標準偏差

double miuV = 0.0;			        //人間の移動速度の期待値[m/s]

double humanW = 4.4;		        //人間の質量[kg]
double robW = 15;		        //ロボットの質量[kg]

double ts = 0.5 * 1;		        //単位タイムステップ時間[s]
double tp = 0.0;                      //人間の移動予測の時間[s]
int nts = 7;				        //タイムステップ数

double calD = 1.0;		        //人間とロボットの距離がこれ以下になると安全性の計算を開始する[m]

double VV = 0.0;				        //変数１
double VO = 0.0;				        //変数２


double P = 0.0;				        //ある点の確率密度
double S = 0.0;				        //ある点の危害の酷さ
double R = 0.0;				        //衝突リスク値

double robX = 0.0, robY = 0.0;		        //ロボットの位置[m]
double robRot = 0.0;			        //ロボットの現在方向[rad]
double robV = 0.0;			//ロボットの現在速度[m/s]
double robRotV = 0.0;			//ロボットの現在角速度[rad/s]
double robVMax = 1.4 / 2;			//ロボットの最大速度[m/s]
double robRadMax = 6.28;		//ロボットの最大旋回速度[rad/s]


double goalX = 2.5, goalY = 5.0;	//目標位置[m]
double goalC = 0.5;			    //目標位置の半径[m]

								//const int N = 15;				//パーティクルの総数
const int N = 10 + 1;               //パーティクルの総数　N = 1 + pN ^ 1 + pN ^ 2 + ... + pN ^ NTS
int pN;					        //一つのパーティクルから伸びる枝の数

double gD = 0.0;				        //現在のロボットと目標位置の距離[m]
double objDist = 0.0;			        //人間とロボットの距離[m]
double nowR = 0.0;						//現在の衝突リスク値
double X[N][N][N][N][N][N][N], Y[N][N][N][N][N][N][N];		        //N番目のパーティクルの座標[m]
double V[N][N][N][N][N][N][N];                    //N番目のパーティクル座標におけるロボットの速度[m/s]
double A[N][N][N][N][N][N][N];                    //N番目のパーティクル座標におけるロボットの加速度[m/s^2]
double ROT[N][N][N][N][N][N][N];                  //N番目のパーティクル座標におけるロボットの向き[rad]
double ROTV[N][N][N][N][N][N][N];                 //N番目のパーティクル座標におけるロボットの角速度[rad/s]
double ROTA[N][N][N][N][N][N][N];                 //N番目のパーティクル座標におけるロボットの角加速度[rad/s^2]
double DIST[N][N][N][N][N][N][N];                 //N番目のパーティクルの進む距離[m]
double GD[N][N][N][N][N][N][N];			        //各パーティクルと目標位置の距離[m]
double RISKP[N][N][N][N][N][N][N];				//各回避軌道経由点候補の衝突リスク値
double RISKT[N][N][N][N][N][N][N];				//各回避軌道候補の積分リスク値
double PP[N][N][N][N][N][N][N];					//各回避軌道経由点候補における人間の存在確率
												//int prev_p[N];			        //N番目のパーティクルと接続する前のパーティクル番号

double robXP = 0.0, robYP = 0.0;	//最短移動する際の次のロボットの予測位置[m/s]
double robRotP = 0.0;				//最短移動する際の次のロボットの予測角度[rad]
double robRotMax = 0.0;				//単位タイムステップにおけるロボットの最大旋回角度[rad]
double robRotC = 0.0;				//単位タイムステップにおける最短移動の際のロボットの旋回角度[rad]


int frag = 0;

int q[N][N][N][N][N][N][N];				        //安全性評価yes/no=0/1
int r;							//回避軌道生成判定no/yes=0/1
int s;                          //評価値出力用
int safe = 0;						//安全評価yes/no=0/1

int st = 1;						//停止評価yes/no=0/1

int SELECT[7];					//選択されたルート番号

int routing();
int safety();
int humanMoving();

int main() {
	printf("main start\n");

	//乱数の初期化
	srand(time(NULL));

	//ロボットの現在位置を測定
	robX = 2.5;
	robY = 0.0;
	robRot = PI / 2;
	robRotMax = robRadMax * ts;

	//人間の現在位置を測定
	humanX = 5.0;
	humanY = 2.5;
	humanTheta = PI * 2 / 2;
	miuV = humanV;

	colliC = humanC + robC;

	//可能性空間の生成
	double robPAcc = 1.4;				//可能性空間の扇の半径[m/s^2]
	double robPRotAcc = 2.09;			//可能性空間の扇角[rad/s^2]

	fopen_s(&fp, "exam1.csv", "wt");
	fprintf(fp, "robX,robY,humanX,humanY,");
	fprintf(fp, "OD,nowRisk,humPred,S\n");

	printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

	double w, x, y, z;
	w = robX;
	x = robY;
	y = humanX;
	z = humanY;

	objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,", w, x, y, z);
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf\n", objDist, nowR, P, S);



	printf("human pos=(%0.10lf,%0.10lf)\n", humanX, humanY);

	while (robX != goalX || robY != goalY)
	{
		printf("robot start moving\n");

		//ロボットが危険回避のための停止動作を行っているか
		if (st == 0) {
			tp = 1;
			robV = 0.0;
			robRotV = 0.0;
			xP = robX;
			yP = robY;
			xV = robV;

			humanMoving();

			safe = safety();

			w = robX;
			x = robY;
			y = humanX;
			z = humanY;
			objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
			nowR = R;

			string p2string(std::to_string(P));

			fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
			//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, P, S);


			if (safe == 0) {
				st = 1;
				printf("safe route regain\n");
			}
			else {

			}
		}
		else {
			//objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
			tp = 1;
			robRotP = atan2(goalY - robY, goalX - robX);
			robRotC = robRotP - robRot;
			if (robRotC > robRotMax) {
				robRotP = robRot + robRotMax;
			}
			else if (robRotC < -1 * robRotMax) {
				robRotP = robRot - robRotMax;
			}
			robXP = robX + robVMax * ts * cos(robRotP);
			robYP = robY + robVMax * ts * sin(robRotP);
			xP = robXP;
			yP = robYP;
			xV = robV;
			r = routing();

			if (r == 0) {
				//ロボットが人間から十分に離れている場合，目標位置に直進
				printf("robot moving straight\n");
				//robRot = atan((goalY - robY) / (goalX - robX));
				robRotP = atan2(goalY - robY, goalX - robX);
				robRotC = robRotP - robRotP;
				//単位タイムステップにおける最大旋回角度を超過した場合，最大旋回角度に丸める
				if (robRotC > robRotMax) {
					robRot = robRot + robRotMax;
				}
				else if (robRotC < -1 * robRotMax) {
					robRot = robRot - robRotMax;
				}
				else {
					robRot = robRotP;
				}
				robX = robX + robVMax * ts * cos(robRot);
				robY = robY + robVMax * ts * sin(robRot);
				robV = robVMax;
				gD = sqrt(pow(goalX - robX, 2) + pow(goalY - robY, 2));
				printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

				humanMoving();

				safety();

				w = robX;
				x = robY;
				y = humanX;
				z = humanY;
				objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
				nowR = R;
				fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf\n", w, x, y, z, objDist, nowR, P, S);


				if (gD < goalC) {
					break;
				}
			}
			else {
				//人間が接近している場合，パーティクル探索を開始


				//加速度を考慮した回避軌道生成
				printf("robot searching route\n");
				double minGD = 0;
				int G = 0;
				int p = 0;


				//パーティクル数
				X[0][0][0][0][0][0][0] = robX;
				Y[0][0][0][0][0][0][0] = robY;

				ROT[0][0][0][0][0][0][0] = robRot;
				V[0][0][0][0][0][0][0] = robV;
				ROTV[0][0][0][0][0][0][0] = robRotV;
				A[0][0][0][0][0][0][0] = 0;
				ROTA[0][0][0][0][0][0][0] = 0;

				GD[0][0][0][0][0][0][0] = sqrt(pow(goalX - X[0][0][0][0][0][0][0], 2) + pow(goalY - Y[0][0][0][0][0][0][0], 2));

				for (int i = 1; i < N; i++) {
					tp = ts;

					ROTA[i][0][0][0][0][0][0] = ROTA[0][0][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
					ROTV[i][0][0][0][0][0][0] = ROTV[0][0][0][0][0][0][0] + ROTA[i][0][0][0][0][0][0] * ts;

					//最大角速度を超過した場合，最大角速度に丸める
					if (ROTV[i][0][0][0][0][0][0] > robRadMax) {
						ROT[i][0][0][0][0][0][0] = ROT[0][0][0][0][0][0][0] + robRadMax * ts;
					}
					else if (ROTV[i][0][0][0][0][0][0] < -1 * robRadMax) {
						ROT[i][0][0][0][0][0][0] = ROT[0][0][0][0][0][0][0] - robRadMax * ts;
					}
					else {
						ROT[i][0][0][0][0][0][0] = ROT[0][0][0][0][0][0][0] + ROTV[0][0][0][0][0][0][0] * ts + ROTA[i][0][0][0][0][0][0] * ts * ts / 2;
					}
					A[i][0][0][0][0][0][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
					V[i][0][0][0][0][0][0] = V[0][0][0][0][0][0][0] + A[i][0][0][0][0][0][0] * ts;
					//最大速度を超過した場合，最大速度に丸める
					if (V[i][0][0][0][0][0][0] > robVMax) {
						DIST[i][0][0][0][0][0][0] = robVMax * ts;
					}
					else if (V[i][0][0][0][0][0][0] < -1 * robVMax) {
						DIST[i][0][0][0][0][0][0] = -1 * robVMax * ts;
					}
					else {
						DIST[i][0][0][0][0][0][0] = V[0][0][0][0][0][0][0] * ts + A[i][0][0][0][0][0][0] * ts * ts / 2;
					}
					X[i][0][0][0][0][0][0] = X[0][0][0][0][0][0][0] + DIST[i][0][0][0][0][0][0] * cos(ROT[i][0][0][0][0][0][0]);
					Y[i][0][0][0][0][0][0] = Y[0][0][0][0][0][0][0] + DIST[i][0][0][0][0][0][0] * sin(ROT[i][0][0][0][0][0][0]);

					GD[i][0][0][0][0][0][0] = sqrt(pow(goalX - X[i][0][0][0][0][0][0], 2) + pow(goalY - Y[i][0][0][0][0][0][0], 2));

					xP = X[i][0][0][0][0][0][0];
					yP = Y[i][0][0][0][0][0][0];
					xV = V[i][0][0][0][0][0][0];
					q[i][0][0][0][0][0][0] = safety();

					PP[i][0][0][0][0][0][0] = P;
					RISKP[i][0][0][0][0][0][0] = R;

					objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

					if (q[i][0][0][0][0][0][0] == 0 && objDist > colliC) {
						//printf("[%d][0][0][0] is safe\n", i);
						for (int ii = 1; ii < N; ++ii) {
							tp = 2 * ts;

							ROTA[i][ii][0][0][0][0][0] = ROTA[i][0][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
							ROTV[i][ii][0][0][0][0][0] = ROTV[i][0][0][0][0][0][0] + ROTA[i][ii][0][0][0][0][0] * ts;

							//最大角速度を超過した場合，最大角速度に丸める
							if (ROTV[i][ii][0][0][0][0][0] > robRadMax) {
								ROT[i][ii][0][0][0][0][0] = ROT[i][0][0][0][0][0][0] + robRadMax * ts;
							}
							else if (ROTV[i][ii][0][0][0][0][0] < -1 * robRadMax) {
								ROT[i][ii][0][0][0][0][0] = ROT[i][0][0][0][0][0][0] - robRadMax * ts;
							}
							else {
								ROT[i][ii][0][0][0][0][0] = ROT[i][0][0][0][0][0][0] + ROTV[i][0][0][0][0][0][0] * ts + ROTA[i][ii][0][0][0][0][0] * ts * ts / 2;
							}
							A[i][ii][0][0][0][0][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
							V[i][ii][0][0][0][0][0] = V[i][0][0][0][0][0][0] + A[i][ii][0][0][0][0][0] * ts;
							//最大速度を超過した場合，最大速度に丸める
							if (V[i][ii][0][0][0][0][0] > robVMax) {
								DIST[i][ii][0][0][0][0][0] = robVMax * ts;
							}
							else if (V[i][ii][0][0][0][0][0] < -1 * robVMax) {
								DIST[i][ii][0][0][0][0][0] = -1 * robVMax * ts;
							}
							else {
								DIST[i][ii][0][0][0][0][0] = V[i][0][0][0][0][0][0] * ts + A[i][ii][0][0][0][0][0] * ts * ts / 2;
							}
							X[i][ii][0][0][0][0][0] = X[i][0][0][0][0][0][0] + DIST[i][ii][0][0][0][0][0] * cos(ROT[i][ii][0][0][0][0][0]);
							Y[i][ii][0][0][0][0][0] = Y[i][0][0][0][0][0][0] + DIST[i][ii][0][0][0][0][0] * sin(ROT[i][ii][0][0][0][0][0]);

							GD[i][ii][0][0][0][0][0] = sqrt(pow(goalX - X[i][ii][0][0][0][0][0], 2) + pow(goalY - Y[i][ii][0][0][0][0][0], 2));

							xP = X[i][ii][0][0][0][0][0];
							yP = Y[i][ii][0][0][0][0][0];
							xV = V[i][ii][0][0][0][0][0];
							q[i][ii][0][0][0][0][0] = safety();

							PP[i][ii][0][0][0][0][0] = P;
							RISKP[i][ii][0][0][0][0][0] = R;

							objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

							if (q[i][ii][0][0][0][0][0] == 0 && objDist > colliC) {
								//printf("[%d][%d][0][0] is safe\n", i, ii);
								for (int iii = 1; iii < N; ++iii) {
									tp = 3 * ts;

									ROTA[i][ii][iii][0][0][0][0] = ROTA[i][ii][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
									ROTV[i][ii][iii][0][0][0][0] = ROTV[i][ii][0][0][0][0][0] + ROTA[i][ii][iii][0][0][0][0] * ts;

									//最大角速度を超過した場合，最大角速度に丸める
									if (ROTV[i][ii][iii][0][0][0][0] > robRadMax) {
										ROT[i][ii][iii][0][0][0][0] = ROT[i][ii][0][0][0][0][0] + robRadMax * ts;
									}
									else if (ROTV[i][ii][iii][0][0][0][0] < -1 * robRadMax) {
										ROT[i][ii][iii][0][0][0][0] = ROT[i][ii][0][0][0][0][0] - robRadMax * ts;
									}
									else {
										ROT[i][ii][iii][0][0][0][0] = ROT[i][ii][0][0][0][0][0] + ROTV[i][ii][0][0][0][0][0] * ts + ROTA[i][ii][iii][0][0][0][0] * ts * ts / 2;
									}
									A[i][ii][iii][0][0][0][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
									V[i][ii][iii][0][0][0][0] = V[i][ii][0][0][0][0][0] + A[i][ii][iii][0][0][0][0] * ts;
									//最大速度を超過した場合，最大速度に丸める
									if (V[i][ii][iii][0][0][0][0] > robVMax) {
										DIST[i][ii][iii][0][0][0][0] = robVMax * ts;
									}
									else if (V[i][ii][iii][0][0][0][0] < -1 * robVMax) {
										DIST[i][ii][iii][0][0][0][0] = -1 * robVMax * ts;
									}
									else {
										DIST[i][ii][iii][0][0][0][0] = V[i][ii][0][0][0][0][0] * ts + A[i][ii][iii][0][0][0][0] * ts * ts / 2;
									}
									X[i][ii][iii][0][0][0][0] = X[i][ii][0][0][0][0][0] + DIST[i][ii][iii][0][0][0][0] * cos(ROT[i][ii][iii][0][0][0][0]);
									Y[i][ii][iii][0][0][0][0] = Y[i][ii][0][0][0][0][0] + DIST[i][ii][iii][0][0][0][0] * sin(ROT[i][ii][iii][0][0][0][0]);

									GD[i][ii][iii][0][0][0][0] = sqrt(pow(goalX - X[i][ii][iii][0][0][0][0], 2) + pow(goalY - Y[i][ii][iii][0][0][0][0], 2));

									xP = X[i][ii][iii][0][0][0][0];
									yP = Y[i][ii][iii][0][0][0][0];
									xV = V[i][ii][iii][0][0][0][0];
									q[i][ii][iii][0][0][0][0] = safety();

									PP[i][ii][iii][0][0][0][0] = P;
									RISKP[i][ii][iii][0][0][0][0] = R;

									objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

									if (q[i][ii][iii][0][0][0][0] == 0 && objDist > colliC) {
										//printf("[%d][%d][%d][0] is safe\n", i, ii, iii);
										for (int iv = 1; iv < N; ++iv) {
											tp = 4 * ts;

											ROTA[i][ii][iii][iv][0][0][0] = ROTA[i][ii][iii][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
											ROTV[i][ii][iii][iv][0][0][0] = ROTV[i][ii][iii][0][0][0][0] + ROTA[i][ii][iii][iv][0][0][0] * ts;

											//最大角速度を超過した場合，最大角速度に丸める
											if (ROTV[i][ii][iii][iv][0][0][0] > robRadMax) {
												ROT[i][ii][iii][iv][0][0][0] = ROT[i][ii][iii][0][0][0][0] + robRadMax * ts;
											}
											else if (ROTV[i][ii][iii][iv][0][0][0] < -1 * robRadMax) {
												ROT[i][ii][iii][iv][0][0][0] = ROT[i][ii][iii][0][0][0][0] - robRadMax * ts;
											}
											else {
												ROT[i][ii][iii][iv][0][0][0] = ROT[i][ii][iii][0][0][0][0] + ROTV[i][ii][iii][0][0][0][0] * ts + ROTA[i][ii][iii][iv][0][0][0] * ts * ts / 2;
											}
											A[i][ii][iii][iv][0][0][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
											V[i][ii][iii][iv][0][0][0] = V[i][ii][iii][0][0][0][0] + A[i][ii][iii][iv][0][0][0] * ts;
											//最大速度を超過した場合，最大速度に丸める
											if (V[i][ii][iii][iv][0][0][0] > robVMax) {
												DIST[i][ii][iii][iv][0][0][0] = robVMax * ts;
											}
											else if (V[i][ii][iii][iv][0][0][0] < -1 * robVMax) {
												DIST[i][ii][iii][iv][0][0][0] = -1 * robVMax * ts;
											}
											else {
												DIST[i][ii][iii][iv][0][0][0] = V[i][ii][iii][0][0][0][0] * ts + A[i][ii][iii][iv][0][0][0] * ts * ts / 2;
											}
											X[i][ii][iii][iv][0][0][0] = X[i][ii][iii][0][0][0][0] + DIST[i][ii][iii][iv][0][0][0] * cos(ROT[i][ii][iii][iv][0][0][0]);
											Y[i][ii][iii][iv][0][0][0] = Y[i][ii][iii][0][0][0][0] + DIST[i][ii][iii][iv][0][0][0] * sin(ROT[i][ii][iii][iv][0][0][0]);

											GD[i][ii][iii][iv][0][0][0] = sqrt(pow(goalX - X[i][ii][iii][iv][0][0][0], 2) + pow(goalY - Y[i][ii][iii][iv][0][0][0], 2));

											xP = X[i][ii][iii][iv][0][0][0];
											yP = Y[i][ii][iii][iv][0][0][0];
											xV = V[i][ii][iii][iv][0][0][0];
											q[i][ii][iii][iv][0][0][0] = safety();

											PP[i][ii][iii][iv][0][0][0] = P;
											RISKP[i][ii][iii][iv][0][0][0] = R;

											objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

											if (q[i][ii][iii][iv][0][0][0] == 0 && objDist > colliC) {
												//printf("[%d][%d][%d][%d] is safe\n", i, ii, iii, iv);
												for (int v = 1; v < N + 1; ++v) {
													tp = 5 * ts;

													ROTA[i][ii][iii][iv][v][0][0] = ROTA[i][ii][iii][iv][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
													ROTV[i][ii][iii][iv][v][0][0] = ROTV[i][ii][iii][iv][0][0][0] + ROTA[i][ii][iii][iv][v][0][0] * ts;

													//最大角速度を超過した場合，最大角速度に丸める
													if (ROTV[i][ii][iii][iv][v][0][0] > robRadMax) {
														ROT[i][ii][iii][iv][v][0][0] = ROT[i][ii][iii][iv][0][0][0] + robRadMax * ts;
													}
													else if (ROTV[i][ii][iii][iv][v][0][0] < -1 * robRadMax) {
														ROT[i][ii][iii][iv][v][0][0] = ROT[i][ii][iii][iv][0][0][0] - robRadMax * ts;
													}
													else {
														ROT[i][ii][iii][iv][v][0][0] = ROT[i][ii][iii][iv][0][0][0] + ROTV[i][ii][iii][iv][0][0][0] * ts + ROTA[i][ii][iii][iv][v][0][0] * ts * ts / 2;
													}
													A[i][ii][iii][iv][v][0][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
													V[i][ii][iii][iv][v][0][0] = V[i][ii][iii][iv][0][0][0] + A[i][ii][iii][iv][v][0][0] * ts;
													//最大速度を超過した場合，最大速度に丸める
													if (V[i][ii][iii][iv][v][0][0] > robVMax) {
														DIST[i][ii][iii][iv][v][0][0] = robVMax * ts;
													}
													else if (V[i][ii][iii][iv][v][0][0] < -1 * robVMax) {
														DIST[i][ii][iii][iv][v][0][0] = -1 * robVMax * ts;
													}
													else {
														DIST[i][ii][iii][iv][v][0][0] = V[i][ii][iii][iv][0][0][0] * ts + A[i][ii][iii][iv][v][0][0] * ts * ts / 2;
													}
													X[i][ii][iii][iv][v][0][0] = X[i][ii][iii][iv][0][0][0] + DIST[i][ii][iii][iv][v][0][0] * cos(ROT[i][ii][iii][iv][v][0][0]);
													Y[i][ii][iii][iv][v][0][0] = Y[i][ii][iii][iv][0][0][0] + DIST[i][ii][iii][iv][v][0][0] * sin(ROT[i][ii][iii][iv][v][0][0]);

													GD[i][ii][iii][iv][v][0][0] = sqrt(pow(goalX - X[i][ii][iii][iv][v][0][0], 2) + pow(goalY - Y[i][ii][iii][iv][v][0][0], 2));

													xP = X[i][ii][iii][iv][v][0][0];
													yP = Y[i][ii][iii][iv][v][0][0];
													xV = V[i][ii][iii][iv][v][0][0];
													q[i][ii][iii][iv][v][0][0] = safety();

													PP[i][ii][iii][iv][v][0][0] = P;
													RISKP[i][ii][iii][iv][v][0][0] = R;

													objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

													if (q[i][ii][iii][iv][v][0][0] == 0 && objDist > colliC) {
														//printf("[%d][%d][%d][%d][%d] is safe\n", i, ii, iii, iv, v);
														for (int vi = 1; vi < N + 1; ++vi) {
															tp = 6 * ts;

															ROTA[i][ii][iii][iv][v][vi][0] = ROTA[i][ii][iii][iv][v][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
															ROTV[i][ii][iii][iv][v][vi][0] = ROTV[i][ii][iii][iv][v][0][0] + ROTA[i][ii][iii][iv][v][vi][0] * ts;

															//最大角速度を超過した場合，最大角速度に丸める
															if (ROTV[i][ii][iii][iv][v][vi][0] > robRadMax) {
																ROT[i][ii][iii][iv][v][vi][0] = ROT[i][ii][iii][iv][v][0][0] + robRadMax * ts;
															}
															else if (ROTV[i][ii][iii][iv][v][vi][0] < -1 * robRadMax) {
																ROT[i][ii][iii][iv][v][vi][0] = ROT[i][ii][iii][iv][v][0][0] - robRadMax * ts;
															}
															else {
																ROT[i][ii][iii][iv][v][vi][0] = ROT[i][ii][iii][iv][v][0][0] + ROTV[i][ii][iii][iv][v][0][0] * ts + ROTA[i][ii][iii][iv][v][vi][0] * ts * ts / 2;
															}
															A[i][ii][iii][iv][v][vi][0] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
															V[i][ii][iii][iv][v][vi][0] = V[i][ii][iii][iv][v][0][0] + A[i][ii][iii][iv][v][vi][0] * ts;
															//最大速度を超過した場合，最大速度に丸める
															if (V[i][ii][iii][iv][v][vi][0] > robVMax) {
																DIST[i][ii][iii][iv][v][vi][0] = robVMax * ts;
															}
															else if (V[i][ii][iii][iv][v][vi][0] < -1 * robVMax) {
																DIST[i][ii][iii][iv][v][vi][0] = -1 * robVMax * ts;
															}
															else {
																DIST[i][ii][iii][iv][v][vi][0] = V[i][ii][iii][iv][v][0][0] * ts + A[i][ii][iii][iv][v][vi][0] * ts * ts / 2;
															}
															X[i][ii][iii][iv][v][vi][0] = X[i][ii][iii][iv][v][0][0] + DIST[i][ii][iii][iv][v][vi][0] * cos(ROT[i][ii][iii][iv][v][vi][0]);
															Y[i][ii][iii][iv][v][vi][0] = Y[i][ii][iii][iv][v][0][0] + DIST[i][ii][iii][iv][v][vi][0] * sin(ROT[i][ii][iii][iv][v][vi][0]);

															GD[i][ii][iii][iv][v][vi][0] = sqrt(pow(goalX - X[i][ii][iii][iv][v][vi][0], 2) + pow(goalY - Y[i][ii][iii][iv][v][vi][0], 2));

															xP = X[i][ii][iii][iv][v][vi][0];
															yP = Y[i][ii][iii][iv][v][vi][0];
															xV = V[i][ii][iii][iv][v][vi][0];
															q[i][ii][iii][iv][v][vi][0] = safety();

															PP[i][ii][iii][iv][v][vi][0] = P;
															RISKP[i][ii][iii][iv][v][vi][0] = R;

															objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

															if (q[i][ii][iii][iv][v][vi][0] == 0 && objDist > colliC) {
																//printf("[%d][%d][%d][%d][%d][%d] is safe\n", i, ii, iii, iv, v, vi);
																for (int vii = 1; vii < N + 1; ++vii) {
																	tp = 7 * ts;

																	ROTA[i][ii][iii][iv][v][vi][vii] = ROTA[i][ii][iii][iv][v][vi][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
																	ROTV[i][ii][iii][iv][v][vi][vii] = ROTV[i][ii][iii][iv][v][vi][0] + ROTA[i][ii][iii][iv][v][vi][vii] * ts;

																	//最大角速度を超過した場合，最大角速度に丸める
																	if (ROTV[i][ii][iii][iv][v][vi][vii] > robRadMax) {
																		ROT[i][ii][iii][iv][v][vi][vii] = ROT[i][ii][iii][iv][v][vi][0] + robRadMax * ts;
																	}
																	else if (ROTV[i][ii][iii][iv][v][vi][vii] < -1 * robRadMax) {
																		ROT[i][ii][iii][iv][v][vi][vii] = ROT[i][ii][iii][iv][v][vi][0] - robRadMax * ts;
																	}
																	else {
																		ROT[i][ii][iii][iv][v][vi][vii] = ROT[i][ii][iii][iv][v][vi][0] + ROTV[i][ii][iii][iv][v][vi][0] * ts + ROTA[i][ii][iii][iv][v][vi][vii] * ts * ts / 2;
																	}
																	A[i][ii][iii][iv][v][vi][vii] = (double)rand() / 32767.0 * 2 * robPAcc - robPAcc;
																	V[i][ii][iii][iv][v][vi][vii] = V[i][ii][iii][iv][v][vi][0] + A[i][ii][iii][iv][v][vi][vii] * ts;
																	//最大速度を超過した場合，最大速度に丸める
																	if (V[i][ii][iii][iv][v][vi][vii] > robVMax) {
																		DIST[i][ii][iii][iv][v][vi][vii] = robVMax * ts;
																	}
																	else if (V[i][ii][iii][iv][v][vi][vii] < -1 * robVMax) {
																		DIST[i][ii][iii][iv][v][vi][vii] = -1 * robVMax * ts;
																	}
																	else {
																		DIST[i][ii][iii][iv][v][vi][vii] = V[i][ii][iii][iv][v][vi][0] * ts + A[i][ii][iii][iv][v][vi][vii] * ts * ts / 2;
																	}
																	X[i][ii][iii][iv][v][vi][vii] = X[i][ii][iii][iv][v][vi][0] + DIST[i][ii][iii][iv][v][vi][vii] * cos(ROT[i][ii][iii][iv][v][vi][vii]);
																	Y[i][ii][iii][iv][v][vi][vii] = Y[i][ii][iii][iv][v][vi][0] + DIST[i][ii][iii][iv][v][vi][vii] * sin(ROT[i][ii][iii][iv][v][vi][vii]);

																	GD[i][ii][iii][iv][v][vi][vii] = sqrt(pow(goalX - X[i][ii][iii][iv][v][vi][vii], 2) + pow(goalY - Y[i][ii][iii][iv][v][vi][vii], 2));

																	xP = X[i][ii][iii][iv][v][vi][vii];
																	yP = Y[i][ii][iii][iv][v][vi][vii];
																	xV = V[i][ii][iii][iv][v][vi][vii];
																	q[i][ii][iii][iv][v][vi][vii] = safety();

																	PP[i][ii][iii][iv][v][vi][vii] = P;
																	RISKP[i][ii][iii][iv][v][vi][vii] = R;

																	objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

																	if (q[i][ii][iii][iv][v][vi][vii] == 0 && objDist > colliC) {
																		//printf("[%d][%d][%d][%d][%d][%d][%d] is safe\n", i, ii, iii, iv, v, vi, vii);
																	}
																	else {
																		GD[i][ii][iii][iv][v][vi][vii] = INFINITY;
																	}
																}
															}
															else {
																GD[i][ii][iii][iv][v][vi][0] = INFINITY;
															}
														}
													}
													else {
														GD[i][ii][iii][iv][v][0][0] = INFINITY;
													}
												}

											}
											else {
												GD[i][ii][iii][iv][0][0][0] = INFINITY;
											}
										}
									}
									else {
										GD[i][ii][iii][0][0][0][0] = INFINITY;
									}
								}
							}
							else {
								GD[i][ii][0][0][0][0][0] = INFINITY;
							}
						}
					}
					else {
						GD[i][0][0][0][0][0][0] = INFINITY;
					}
				}
				tp = ts;

				/*
				for (int i = 0; i < N; ++i) {
				printf("GD[%d][][] = %0.10lf\n", i, GD[i]);
				}

				for (int i = 0; i < N; ++i) {
				printf("X[%d] = %0.10lf\n", i, X[i]);
				printf("Y[%d] = %0.10lf\n", i, Y[i]);
				}

				for (int i = 0; i < N; ++i) {
				printf("V[%d] = %0.10lf\n", i, V[i]);
				}

				for (int i = 0; i < N; ++i) {
				printf("DIST[%d] = %0.10lf\n", i, DIST[i]);
				}
				*/





				// 目標位置に最も近いパーティクルの番号を取得
				minGD = GD[0][0][0][0][0][0][0];
				//printf("G = %p\n", GD[0]);
				G = 0;
				for (int t1 = 1; t1 < N; ++t1) {
					for (int t2 = 0; t2 < N; ++t2) {
						for (int t3 = 0; t3 < N; ++t3) {
							for (int t4 = 0; t4 < N; ++t4) {
								for (int t5 = 0; t5 < N + 1; ++t5) {
									for (int t6 = 0; t6 < N + 1; ++t6) {
										for (int t7 = 0; t7 < N + 1; ++t7) {
											if (GD[t1][t2][t3][t4][t5][t6][t7] != 0) {
												if (minGD > GD[t1][t2][t3][t4][t5][t6][t7]) {
													minGD = GD[t1][t2][t3][t4][t5][t6][t7];
													G = t1;
													nowR = RISKP[t1][0][0][0][0][0][0];
													P = PP[t1][0][0][0][0][0][0];
												}
											}
										}
									}
								}
							}
						}
					}

				}

				// 目標位置に最も近いパーティクルから現在位置までのパーティクル座標を取得

				p = G;

				if (p == 0) {
					//安全な経路が存在しないので停止する
					printf("safe route is not detected! route replaning...\n");

					/*
					w = robX;
					x = robY;
					y = humanX;
					z = humanY;
					objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
					nowR = R;
					string p2string(std::to_string(P));

					fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
					//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf\n", w, x, y, z, objDist, nowR, P, S);
					*/
					st = 0;
					printf("stop=%d\n", st);
				}
				else {
					//安全な経路の一歩目を移動
					printf("selected route is %d\n", p);
					robX = X[p][0][0][0][0][0][0];
					robY = Y[p][0][0][0][0][0][0];
					robRot = ROT[p][0][0][0][0][0][0];
					robV = V[p][0][0][0][0][0][0];
					robRotV = ROTV[p][0][0][0][0][0][0];
					gD = sqrt(pow(goalX - robX, 2) + pow(goalY - robY, 2));
					printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

					humanMoving();

					w = robX;
					x = robY;
					y = humanX;
					z = humanY;
					objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
					nowR = R;
					string p2string(std::to_string(P));

					fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
					//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf\n", w, x, y, z, objDist, nowR, P, S);

					if (gD < goalC) {
						break;
					}
				}

				for (int t1 = 1; t1 < N; ++t1) {
					for (int t2 = 0; t2 < N; ++t2) {
						for (int t3 = 0; t3 < N; ++t3) {
							for (int t4 = 0; t4 < N; ++t4) {
								for (int t5 = 0; t5 < N + 1; ++t5) {
									for (int t6 = 0; t6 < N + 1; ++t6) {
										for (int t7 = 0; t7 < N + 1; ++t7) {
											//配列の初期化
											X[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											Y[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											V[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											A[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											ROT[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											ROTV[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											ROTA[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											DIST[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											GD[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											RISKP[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											RISKT[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											PP[t1][t2][t3][t4][t5][t6][t7] = 0.0;
											q[t1][t2][t3][t4][t5][t6][t7] = 0;

										}
									}
								}
							}

						}
					}

				}




			}

		}

	}






	printf("human pos=(%0.10lf,%0.10lf)\n", humanX, humanY);
	fclose(fp);

	//移動処理が終了したことの合図
	printf("done!");
}

//安全性の判定関数（安全なら0，危険なら1を返す）
int safety() {
	//printf("safety function\n");
	//人間のある点における存在確率密度の計算
	double A, B;
	double robVX = 0.0, robVY = 0.0;
	double relVX = 0.0, relVY = 0.0, rV = 0.0;
	double robRotH = 0.0;

	robRotH = atan2(humanY - robY, humanX - robX);

	xP = xP + colliC * cos(robRotH);
	yP = yP + colliC * sin(robRotH);

	humanVP = sqrt(pow(xP - humanX, 2) + pow(yP - humanY, 2)) / tp;
	//printf("humanVP=%0.10lf\n", humanVP);

	humanThetaP = atan2(yP - humanY, xP - humanX) - humanTheta;
	//printf("humanThetaP=%0.10lf\n", humanThetaP);

	robThetaP = atan2(humanY - yP, humanX - xP);

	sigTheta = (1.35 * PI) / (8 * (humanV + 0.00001));
	//printf("sigTheta=%0.10lf\n", sigTheta);


	VV = (humanVP - miuV) / sigV;
	//printf("VV=%0.10lf\n", VV);

	VO = humanThetaP / sigTheta;
	//printf("VO=%0.10lf\n", VO);

	A = exp((pow(VV, 2) + pow(VO, 2)) / -2);
	//printf("A=%0.10lf\n", A);

	B = 2 * PI * sigV * sigTheta;
	//printf("B=%0.10lf\n", B);

	P = A / B;
	//printf("P=%0.10lf	", P);


	//ある点における危害の酷さの計算
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);
	if (relVX > 0 && relVY > 0) {
		rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	}
	else {
		rV = 0.0;
	}
	S = humanW * robW * pow(rV, 2) / (humanW + robW) / 2;
	//printf("S=%0.10lf	", S);

	//衝突リスクの計算
	R = P * S;
	//printf("risk num=%0.10lf\n", R);

	//安全性の判定
	if (R > 0.00000011 * nts) {
		return 1;
	}
	else {
		return 0;
	}
}

//回避軌道生成開始判定関数（開始しないなら0，開始するなら1を返す）
int routing() {
	//printf("routing function\n");
	//人間のある点における存在確率密度の計算
	double A, B;
	double robVX = 0.0, robVY = 0.0;
	double relVX = 0.0, relVY = 0.0, rV = 0.0;
	double robRotH = 0.0;

	robRotH = atan2(humanY - robY, humanX - robX);

	xP = xP + colliC * cos(robRotH);
	yP = yP + colliC * sin(robRotH);

	humanVP = sqrt(pow(xP - humanX, 2) + pow(yP - humanY, 2)) / tp;
	printf("humanVP=%0.10lf\n", humanVP);

	humanThetaP = atan2(yP - humanY, xP - humanX) - humanTheta;
	printf("humanThetaP=%0.10lf\n", humanThetaP);

	robThetaP = atan2(humanY - yP, humanX - xP);

	sigTheta = (1.35 * PI) / (8 * (humanV + 0.00001));
	printf("sigTheta=%0.10lf\n", sigTheta);


	VV = (humanVP - miuV) / sigV;
	printf("VV=%0.10lf\n", VV);

	VO = humanThetaP / sigTheta;
	printf("VO=%0.10lf\n", VO);

	A = exp((pow(VV, 2) + pow(VO, 2)) / -2);
	printf("A=%0.10lf\n", A);

	B = 2 * PI * sigV * sigTheta;
	printf("B=%0.10lf\n", B);

	P = A / B;
	printf("P=%0.10lf\n", P);

	/*
	//ある点における危害の酷さの計算
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	S = humanW * robW * pow(rV, 2) / (humanW + robW) / 2;
	//printf("S=%0.10lf\n", S);

	//衝突リスクの計算
	R = P * S;
	//printf("risk num=%0.10lf\n", R);
	*/

	//回避軌道生成開始の判定
	if (P > 0.000001) {
		return 1;
	}
	else {
		return 0;
	}
}

int humanMoving() {
	//人間のタイムステップ毎の移動を処理
	//humanX = humanX + humanV * ts * cos((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanY = humanY + humanV * ts * sin((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	humanX = humanX - humanV * ts;
	//humanY = humanY - humanV * ts;

	/*//方向転換45°
	if (humanX > 2.5) {
	humanX = humanX - humanV * ts;
	}
	else {
	humanTheta = PI * 1 / 4;
	humanX = humanX + humanV * ts;
	humanY = humanY + humanV * ts;
	}*/

	/*//方向転換180°
	if (frag == 1) {
	humanTheta = PI * 0 / 4;
	humanX = humanX + humanV * ts;
	//humanY = humanY - humanV * ts;
	}
	else if (humanX > 2.5) {
	humanX = humanX - humanV * ts;
	}
	else {
	frag = 1;
	humanY = humanY - 0.5;
	}*/

	return 0;
}