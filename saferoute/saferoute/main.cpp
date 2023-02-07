#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include "common.h"
#include <string>

#define _USE_MATH_DEFINES //数値演算定数が定義されたヘッダファイルの読み込み

#define PI 3.14159265359

FILE* fp; //FILEポインタの宣言

using std::string;


double humanX = 0.0, humanY = 0.0;	        //人間の現在位置[m]
double humanTheta = 0.0;		        //人間の現在の方向[°]
double humanXP, humanYP;				//想定される人間の位置[m]
double xP = 0.0, yP = 0.0;			        //人間の移動予測点[ml
double xV = 0.0;					//ロボットの予測速度[m/s]
double colliC = 0;				//人間とロボットの接触距離[m]

								//double humanV = 1.4 / 2;	        //人間の移動速度[m/s]
double humanV = 0.7;								//double humanV = 1.4;

double humanVP = 0.0;			        //人間の移動ノルム

double humanThetaP = 0.0;		        //人間の現在位置からみたある点の方向[°]
double robThetaP = 0.0;					//ロボットのある位置から見た現在の人間の方向[°]


double miuV = 0.0;			        //人間の移動速度の期待値[m/s]

double SIGMA_THETA = 0.0; //人間の進行方向の標準偏差

double tp = 0.0;                      //人間の移動予測の時間[s]


double VV = 0.0;				        //変数１
double VO = 0.0;				        //変数２


double P = 0.0;				        //ある点の確率密度
double S = 0.0;				        //ある点の危害の酷さ
double R = 0.0;				        //衝突リスク値

double robX = 0.0, robY = 0.0;		        //ロボットの位置[m]
double robRot = 0.0;			        //ロボットの現在方向[rad]
double robV = 0.0;			//ロボットの現在速度[m/s]
double robRotV = 0.0;			//ロボットの現在角速度[rad/s]

								//double ROBOT_MAX_RAD = 1.57;
double robDist = 0.0;			//ロボットが進む距離[m]
int pN;					        //一つのパーティクルから伸びる枝の数

double humanGD = 0.0;				//現在の人間と人間の目的位置の距離[m]
double gD = 0.0;				        //現在のロボットと目標位置の距離[m]
double objDist = 0.0;			        //人間とロボットの距離[m]
double nowR = 0.0;						//現在の衝突リスク値
double X[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM], Y[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];		        //N番目のパーティクルの座標[m]
double V[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                    //N番目のパーティクル座標におけるロボットの速度[m/s]
double A[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                    //N番目のパーティクル座標におけるロボットの加速度[m/s^2]
double ROT[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                  //N番目のパーティクル座標におけるロボットの向き[rad]
double ROTV[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N番目のパーティクル座標におけるロボットの角速度[rad/s]
double ROTA[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N番目のパーティクル座標におけるロボットの角加速度[rad/s^2]
double DIST[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N番目のパーティクルの進む距離[m]
double GD[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];			        //各パーティクルと目標位置の距離[m]
double RISKP[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				//各回避軌道経由点候補の衝突リスク値
double RISKT[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				//各回避軌道候補の積分リスク値
double PP[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];					//各回避軌道経由点候補における人間の存在確率
																																					//int prev_p[EVERY_TIMESTEP_NUM];			        //N番目のパーティクルと接続する前のパーティクル番号

double robXP = 0.0, robYP = 0.0;	//最短移動する際の次のロボットの予測位置[m/s]
double robRotP = 0.0;				//最短移動する際の次のロボットの予測角度[rad]
double robRotMax = 0.0;				//単位タイムステップにおけるロボットの最大旋回角度[rad]
double robRotC = 0.0;				//単位タイムステップにおける最短移動の際のロボットの旋回角度[rad]


int frag = 0;

int q[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				        //安全性評価yes/no=0/1
int r;							//回避軌道生成判定no/yes=0/1
int s;                          //評価値出力用
int safe = 0;						//安全評価yes/no=0/1

int st = 1;						//停止評価yes/no=0/1

								//int SELECT[6];					//選択されたルート番号

int routing();
int safety();
int humanMoving();

int main() {
	printf("main start\n");

	//乱数の初期化
	srand(time(NULL));

	//ロボットの現在位置を測定
	robX = ROBOT_X_INIT;
	robY = ROBOT_Y_INIT;
	robV = 0.0;
	robRot = ROBOT_RAD_INIT;
	robRotMax = ROBOT_MAX_RAD * TIMESTEP;

	//人間の現在位置を測定
	humanX = 1.5;
	humanY = 4.0;
	humanTheta = PI * -1 / 2;
	miuV = humanV;

	colliC = HUMAN_RADIUS + ROBOT_RADIUS;

	//可能性空間の生成
	double robPAcc = 1.4;				//可能性空間の扇の半径[m/s^2]
	double robPRotAcc = 2.09;			//可能性空間の扇角[rad/s^2]

										//結果表示ファイルの作成
	fopen_s(&fp, "exam1.csv", "wt");
	fprintf(fp, "robX,robY,humanX,humanY,");
	fprintf(fp, "OD,nowRisk,humPred,S,st\n");

	//printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);
	tp = 1;
	xP = robX;
	yP = robY;
	xV = robV;
	humanXP = humanX;
	humanYP = humanY;
	safety();

	double w, x, y, z;
	w = robX;
	x = robY;
	y = humanX;
	z = humanY;

	objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));

	//時刻ごとの諸変数を表示
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,", w, x, y, z);
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", objDist, nowR, P, S, st);



	//printf("human pos=(%0.10lf,%0.10lf)\n", humanX, humanY);

	//ロボットが目的位置に到達していない場合，移動を継続する
	while (robX != GOAL_X || robY != GOAL_Y)
	{
		//printf("robot start moving\n");

		//ロボットが危険回避のための停止動作を行っているか
		if (st == 0) {
			//printf("robot stopping\n");
			tp = 0.5;

			//角速度を0に近づける
			if (robRotV > 0) {
				robRotV = robRotV - robPRotAcc * TIMESTEP;
				if (robRotV > 0) {
					robRot = robRot + robRotV * TIMESTEP - robPRotAcc * TIMESTEP * TIMESTEP / 2;
					if (robRot > PI) {
						robRot = robRot - 2 * PI;
					}
					else if (robRot < -1 * PI) {
						robRot = robRot + 2 * PI;
					}
				}
				else {
					robRotV = 0;
				}
			}
			else if (robRotV < 0) {
				robRotV = robRotV + robPRotAcc * TIMESTEP;
				if (robRotV < 0) {
					robRot = robRot + robRotV * TIMESTEP + robPRotAcc * TIMESTEP * TIMESTEP / 2;
					if (robRot > PI) {
						robRot = robRot - 2 * PI;
					}
					else if (robRot < -1 * PI) {
						robRot = robRot + 2 * PI;
					}
				}
				else {
					robRotV = 0;
				}
			}

			//最大速度を超過した場合，最大速度に丸める
			if (robV > 0) {
				robV = robV - robPAcc * TIMESTEP;
				if (robV > 0) {
					robDist = robV * TIMESTEP - robPAcc * TIMESTEP * TIMESTEP / 2;
				}
				else {
					robV = 0;
					robDist = 0;
				}
			}
			else if (robV < 0) {
				robV = robV + robPAcc * TIMESTEP;
				if (robV < 0) {
					robDist = robV * TIMESTEP + robPAcc * TIMESTEP * TIMESTEP / 2;
				}
				else {
					robV = 0;
					robDist = 0;
				}
			}

			robX = robX + robDist * cos(robRot);
			robY = robY + robDist * sin(robRot);

			if (robX > AREA_X) {
				robX = AREA_X;
			}
			else if (robX < 0) {
				robX = 0;
			}
			if (robY > AREA_Y) {
				robY = AREA_Y;
			}
			else if (robY < 0) {
				robY = 0;
			}

			xP = robX;
			yP = robY;
			xV = robV;
			humanXP = humanX + humanV * 1 * TIMESTEP * cos(humanTheta);
			humanYP = humanY + humanV * 1 * TIMESTEP * sin(humanTheta);
			safe = safety();

			humanMoving();

			//safety();

			w = robX;
			x = robY;
			y = humanX;
			z = humanY;
			objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
			nowR = R;

			printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

			//string p2string(std::to_string(P));

			//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
			fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", w, x, y, z, objDist, nowR, P, S, st);


			if (safe == 0) {
				st = 1;
				//printf("safe route regain\n");
			}
			else {

			}
		}
		else {
			//objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
			tp = 1;
			robRotP = atan2(GOAL_Y - robY, GOAL_X - robX);

			robRotC = robRotP - robRot;
			if (robRotC > robRotMax) {
				robRotP = robRot + robRotMax;
				if (robRotP > PI) {
					robRotP = robRotP - 2 * PI;
				}
				else if (robRotP < -1 * PI) {
					robRotP = robRotP + 2 * PI;
				}
			}
			else if (robRotC < -1 * robRotMax) {
				robRotP = robRot - robRotMax;
				if (robRotP > PI) {
					robRotP = robRotP - 2 * PI;
				}
				else if (robRotP < -1 * PI) {
					robRotP = robRotP + 2 * PI;
				}
			}

			robXP = robX + ROBOT_MAX_VELOCITY * 1 * TIMESTEP * cos(robRotP);
			robYP = robY + ROBOT_MAX_VELOCITY * 1 * TIMESTEP * sin(robRotP);
			xP = robXP;
			yP = robYP;
			xV = ROBOT_MAX_VELOCITY;
			humanXP = humanX + humanV * cos(humanTheta);
			humanYP = humanY + humanV * sin(humanTheta);
			r = routing();

			if (r == 0) {
				//ロボットが人間から十分に離れている場合，目標位置に直進
				//printf("robot moving straight\n");
				//robRot = atan((GOAL_Y  - robY) / (GOAL_X - robX));
				robRotP = atan2(GOAL_Y - robY, GOAL_X - robX);
				robRotC = robRotP - robRotP;
				//単位タイムステップにおける最大旋回角度を超過した場合，最大旋回角度に丸める
				if (robRotC > robRotMax) {
					robRot = robRot + robRotMax;
					if (robRot > PI) {
						robRot = robRot - 2 * PI;
					}
					else if (robRot < -1 * PI) {
						robRot = robRot + 2 * PI;
					}
				}
				else if (robRotC < -1 * robRotMax) {
					robRot = robRot - robRotMax;
					if (robRot > PI) {
						robRot = robRot - 2 * PI;
					}
					else if (robRot < -1 * PI) {
						robRot = robRot + 2 * PI;
					}
				}
				else {
					robRot = robRotP;
				}
				robX = robX + ROBOT_MAX_VELOCITY * TIMESTEP * cos(robRot);
				robY = robY + ROBOT_MAX_VELOCITY * TIMESTEP * sin(robRot);
				if (robX > AREA_X) {
					robX = AREA_X;
				}
				else if (robX < 0) {
					robX = 0;
				}
				if (robY > AREA_Y) {
					robY = AREA_Y;
				}
				else if (robY < 0) {
					robY = 0;
				}

				robV = ROBOT_MAX_VELOCITY;
				gD = sqrt(pow(GOAL_X - robX, 2) + pow(GOAL_Y - robY, 2));
				printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

				tp = 0.5;

				xP = robXP;
				yP = robYP;
				xV = robV;
				humanXP = humanX + humanV * 1 * TIMESTEP * cos(humanTheta);
				humanYP = humanY + humanV * 1 * TIMESTEP * sin(humanTheta);
				safety();

				humanMoving();

				//safety();

				w = robX;
				x = robY;
				y = humanX;
				z = humanY;
				objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
				nowR = R;


				//string p2string(std::to_string(P));

				//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
				fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", w, x, y, z, objDist, nowR, P, S, st);


				if (gD < GOAL_DISTANCE) {
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
				X[0][0][0][0][0][0] = robX;
				Y[0][0][0][0][0][0] = robY;

				ROT[0][0][0][0][0][0] = robRot;
				V[0][0][0][0][0][0] = robV;
				ROTV[0][0][0][0][0][0] = robRotV;
				A[0][0][0][0][0][0] = 0;
				ROTA[0][0][0][0][0][0] = 0;

				GD[0][0][0][0][0][0] = sqrt(pow(GOAL_X - X[0][0][0][0][0][0], 2) + pow(GOAL_Y - Y[0][0][0][0][0][0], 2));

				//各回避軌道経由点をランダムな位置に生成し，安全性を確認する
				for (int i = 1; i < EVERY_TIMESTEP_NUM; i++) {
					tp = TIMESTEP;

					ROTA[i][0][0][0][0][0] = ROTA[0][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
					ROTV[i][0][0][0][0][0] = ROTV[0][0][0][0][0][0] + ROTA[i][0][0][0][0][0] * TIMESTEP;

					//最大角速度を超過した場合，最大角速度に丸める
					if (ROTV[i][0][0][0][0][0] > ROBOT_MAX_RAD) {
						ROT[i][0][0][0][0][0] = ROT[0][0][0][0][0][0] + ROBOT_MAX_RAD * TIMESTEP;
						if (ROT[i][0][0][0][0][0] > PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] - 2 * PI;
						}
						else if (ROT[i][0][0][0][0][0] < -1 * PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] + 2 * PI;
						}
					}
					else if (ROTV[i][0][0][0][0][0] < -1 * ROBOT_MAX_RAD) {
						ROT[i][0][0][0][0][0] = ROT[0][0][0][0][0][0] - ROBOT_MAX_RAD * TIMESTEP;
						if (ROT[i][0][0][0][0][0] > PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] - 2 * PI;
						}
						else if (ROT[i][0][0][0][0][0] < -1 * PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] + 2 * PI;
						}
					}
					else {
						ROT[i][0][0][0][0][0] = ROT[0][0][0][0][0][0] + ROTV[0][0][0][0][0][0] * TIMESTEP + ROTA[i][0][0][0][0][0] * TIMESTEP * TIMESTEP / 2;
						if (ROT[i][0][0][0][0][0] > PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] - 2 * PI;
						}
						else if (ROT[i][0][0][0][0][0] < -1 * PI) {
							ROT[i][0][0][0][0][0] = ROT[i][0][0][0][0][0] + 2 * PI;
						}
					}
					A[i][0][0][0][0][0] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
					V[i][0][0][0][0][0] = V[0][0][0][0][0][0] + A[i][0][0][0][0][0] * TIMESTEP;
					//最大速度を超過した場合，最大速度に丸める
					if (V[i][0][0][0][0][0] > ROBOT_MAX_VELOCITY) {
						DIST[i][0][0][0][0][0] = ROBOT_MAX_VELOCITY * TIMESTEP;
					}
					else if (V[i][0][0][0][0][0] < -1 * ROBOT_MAX_VELOCITY) {
						DIST[i][0][0][0][0][0] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
					}
					else {
						DIST[i][0][0][0][0][0] = V[0][0][0][0][0][0] * TIMESTEP + A[i][0][0][0][0][0] * TIMESTEP * TIMESTEP / 2;
					}
					X[i][0][0][0][0][0] = X[0][0][0][0][0][0] + DIST[i][0][0][0][0][0] * cos(ROT[i][0][0][0][0][0]);
					Y[i][0][0][0][0][0] = Y[0][0][0][0][0][0] + DIST[i][0][0][0][0][0] * sin(ROT[i][0][0][0][0][0]);
					if (X[i][0][0][0][0][0] > AREA_X) {
						X[i][0][0][0][0][0] = AREA_X;
					}
					else if (X[i][0][0][0][0][0] < 0) {
						X[i][0][0][0][0][0] = 0;
					}
					if (Y[i][0][0][0][0][0] > AREA_Y) {
						Y[i][0][0][0][0][0] = AREA_Y;
					}
					else if (Y[i][0][0][0][0][0] < 0) {
						Y[i][0][0][0][0][0] = 0;
					}


					GD[i][0][0][0][0][0] = sqrt(pow(GOAL_X - X[i][0][0][0][0][0], 2) + pow(GOAL_Y - Y[i][0][0][0][0][0], 2));

					xP = X[i][0][0][0][0][0];
					yP = Y[i][0][0][0][0][0];
					xV = V[i][0][0][0][0][0];
					humanXP = humanX + humanV * 1 * TIMESTEP * cos(humanTheta);
					humanYP = humanY + humanV * 1 * TIMESTEP * sin(humanTheta);
					q[i][0][0][0][0][0] = safety();

					PP[i][0][0][0][0][0] = P;
					RISKP[i][0][0][0][0][0] = R;
					RISKT[i][0][0][0][0][0] = RISKP[i][0][0][0][0][0];

					objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

					if (q[i][0][0][0][0][0] == 0 /*&& objDist > colliC*/) {
						//printf("[%d][0][0][0] is safe\n", i);
						for (int ii = 1; ii < EVERY_TIMESTEP_NUM; ++ii) {
							tp = 1 * TIMESTEP;

							ROTA[i][ii][0][0][0][0] = ROTA[i][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
							ROTV[i][ii][0][0][0][0] = ROTV[i][0][0][0][0][0] + ROTA[i][ii][0][0][0][0] * TIMESTEP;

							//最大角速度を超過した場合，最大角速度に丸める
							if (ROTV[i][ii][0][0][0][0] > ROBOT_MAX_RAD) {
								ROT[i][ii][0][0][0][0] = ROT[i][0][0][0][0][0] + ROBOT_MAX_RAD * TIMESTEP;
								if (ROT[i][ii][0][0][0][0] > PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] - 2 * PI;
								}
								else if (ROT[i][ii][0][0][0][0] < -1 * PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] + 2 * PI;
								}
							}
							else if (ROTV[i][ii][0][0][0][0] < -1 * ROBOT_MAX_RAD) {
								ROT[i][ii][0][0][0][0] = ROT[i][0][0][0][0][0] - ROBOT_MAX_RAD * TIMESTEP;
								if (ROT[i][ii][0][0][0][0] > PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] - 2 * PI;
								}
								else if (ROT[i][ii][0][0][0][0] < -1 * PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] + 2 * PI;
								}
							}
							else {
								ROT[i][ii][0][0][0][0] = ROT[i][0][0][0][0][0] + ROTV[i][0][0][0][0][0] * TIMESTEP + ROTA[i][ii][0][0][0][0] * TIMESTEP * TIMESTEP / 2;
								if (ROT[i][ii][0][0][0][0] > PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] - 2 * PI;
								}
								else if (ROT[i][ii][0][0][0][0] < -1 * PI) {
									ROT[i][ii][0][0][0][0] = ROT[i][ii][0][0][0][0] + 2 * PI;
								}
							}
							A[i][ii][0][0][0][0] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
							V[i][ii][0][0][0][0] = V[i][0][0][0][0][0] + A[i][ii][0][0][0][0] * TIMESTEP;
							//最大速度を超過した場合，最大速度に丸める
							if (V[i][ii][0][0][0][0] > ROBOT_MAX_VELOCITY) {
								DIST[i][ii][0][0][0][0] = ROBOT_MAX_VELOCITY * TIMESTEP;
							}
							else if (V[i][ii][0][0][0][0] < -1 * ROBOT_MAX_VELOCITY) {
								DIST[i][ii][0][0][0][0] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
							}
							else {
								DIST[i][ii][0][0][0][0] = V[i][0][0][0][0][0] * TIMESTEP + A[i][ii][0][0][0][0] * TIMESTEP * TIMESTEP / 2;
							}
							X[i][ii][0][0][0][0] = X[i][0][0][0][0][0] + DIST[i][ii][0][0][0][0] * cos(ROT[i][ii][0][0][0][0]);
							Y[i][ii][0][0][0][0] = Y[i][0][0][0][0][0] + DIST[i][ii][0][0][0][0] * sin(ROT[i][ii][0][0][0][0]);
							if (X[i][ii][0][0][0][0] > AREA_X) {
								X[i][ii][0][0][0][0] = AREA_X;
							}
							else if (X[i][ii][0][0][0][0] < 0) {
								X[i][ii][0][0][0][0] = 0;
							}
							if (Y[i][ii][0][0][0][0] > AREA_Y) {
								Y[i][ii][0][0][0][0] = AREA_Y;
							}
							else if (Y[i][ii][0][0][0][0] < 0) {
								Y[i][ii][0][0][0][0] = 0;
							}

							GD[i][ii][0][0][0][0] = sqrt(pow(GOAL_X - X[i][ii][0][0][0][0], 2) + pow(GOAL_Y - Y[i][ii][0][0][0][0], 2));

							xP = X[i][ii][0][0][0][0];
							yP = Y[i][ii][0][0][0][0];
							xV = V[i][ii][0][0][0][0];
							humanXP = humanX + 2 * humanV * TIMESTEP * cos(humanTheta);
							humanYP = humanY + 2 * humanV * TIMESTEP * sin(humanTheta);
							q[i][ii][0][0][0][0] = safety();

							PP[i][ii][0][0][0][0] = P;
							RISKP[i][ii][0][0][0][0] = R;
							RISKT[i][ii][0][0][0][0] = RISKP[i][0][0][0][0][0] + RISKP[i][ii][0][0][0][0];

							objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

							if (q[i][ii][0][0][0][0] == 0 /*&& objDist > colliC*/) {
								//printf("[%d][%d][0][0] is safe\n", i, ii);
								for (int iii = 1; iii < EVERY_TIMESTEP_NUM; ++iii) {
									tp = 1 * TIMESTEP;

									ROTA[i][ii][iii][0][0][0] = ROTA[i][ii][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
									ROTV[i][ii][iii][0][0][0] = ROTV[i][ii][0][0][0][0] + ROTA[i][ii][iii][0][0][0] * TIMESTEP;

									//最大角速度を超過した場合，最大角速度に丸める
									if (ROTV[i][ii][iii][0][0][0] > ROBOT_MAX_RAD) {
										ROT[i][ii][iii][0][0][0] = ROT[i][ii][0][0][0][0] + ROBOT_MAX_RAD * TIMESTEP;
										if (ROT[i][ii][iii][0][0][0] > PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] - 2 * PI;
										}
										else if (ROT[i][ii][iii][0][0][0] < -1 * PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] + 2 * PI;
										}
									}
									else if (ROTV[i][ii][iii][0][0][0] < -1 * ROBOT_MAX_RAD) {
										ROT[i][ii][iii][0][0][0] = ROT[i][ii][0][0][0][0] - ROBOT_MAX_RAD * TIMESTEP;
										if (ROT[i][ii][iii][0][0][0] > PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] - 2 * PI;
										}
										else if (ROT[i][ii][iii][0][0][0] < -1 * PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] + 2 * PI;
										}
									}
									else {
										ROT[i][ii][iii][0][0][0] = ROT[i][ii][0][0][0][0] + ROTV[i][ii][0][0][0][0] * TIMESTEP + ROTA[i][ii][iii][0][0][0] * TIMESTEP * TIMESTEP / 2;
										if (ROT[i][ii][iii][0][0][0] > PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] - 2 * PI;
										}
										else if (ROT[i][ii][iii][0][0][0] < -1 * PI) {
											ROT[i][ii][iii][0][0][0] = ROT[i][ii][iii][0][0][0] + 2 * PI;
										}
									}
									A[i][ii][iii][0][0][0] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
									V[i][ii][iii][0][0][0] = V[i][ii][0][0][0][0] + A[i][ii][iii][0][0][0] * TIMESTEP;
									//最大速度を超過した場合，最大速度に丸める
									if (V[i][ii][iii][0][0][0] > ROBOT_MAX_VELOCITY) {
										DIST[i][ii][iii][0][0][0] = ROBOT_MAX_VELOCITY * TIMESTEP;
									}
									else if (V[i][ii][iii][0][0][0] < -1 * ROBOT_MAX_VELOCITY) {
										DIST[i][ii][iii][0][0][0] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
									}
									else {
										DIST[i][ii][iii][0][0][0] = V[i][ii][0][0][0][0] * TIMESTEP + A[i][ii][iii][0][0][0] * TIMESTEP * TIMESTEP / 2;
									}
									X[i][ii][iii][0][0][0] = X[i][ii][0][0][0][0] + DIST[i][ii][iii][0][0][0] * cos(ROT[i][ii][iii][0][0][0]);
									Y[i][ii][iii][0][0][0] = Y[i][ii][0][0][0][0] + DIST[i][ii][iii][0][0][0] * sin(ROT[i][ii][iii][0][0][0]);
									if (X[i][ii][iii][0][0][0] > AREA_X) {
										X[i][ii][iii][0][0][0] = AREA_X;
									}
									else if (X[i][ii][iii][0][0][0] < 0) {
										X[i][ii][iii][0][0][0] = 0;
									}
									if (Y[i][ii][iii][0][0][0] > AREA_Y) {
										Y[i][ii][iii][0][0][0] = AREA_Y;
									}
									else if (Y[i][ii][iii][0][0][0] < 0) {
										Y[i][ii][iii][0][0][0] = 0;
									}

									GD[i][ii][iii][0][0][0] = sqrt(pow(GOAL_X - X[i][ii][iii][0][0][0], 2) + pow(GOAL_Y - Y[i][ii][iii][0][0][0], 2));

									xP = X[i][ii][iii][0][0][0];
									yP = Y[i][ii][iii][0][0][0];
									xV = V[i][ii][iii][0][0][0];
									humanXP = humanX + 3 * humanV * TIMESTEP * cos(humanTheta);
									humanYP = humanY + 3 * humanV * TIMESTEP * sin(humanTheta);
									q[i][ii][iii][0][0][0] = safety();

									PP[i][ii][iii][0][0][0] = P;
									RISKP[i][ii][iii][0][0][0] = R;
									RISKT[i][ii][iii][0][0][0] = RISKP[i][0][0][0][0][0] + RISKP[i][ii][0][0][0][0] + RISKP[i][ii][iii][0][0][0];

									objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

									if (q[i][ii][iii][0][0][0] == 0 /*&& objDist > colliC*/) {
										//printf("[%d][%d][%d][0] is safe\n", i, ii, iii);
										for (int iv = 1; iv < EVERY_TIMESTEP_NUM; ++iv) {
											tp = 1 * TIMESTEP;

											ROTA[i][ii][iii][iv][0][0] = ROTA[i][ii][iii][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
											ROTV[i][ii][iii][iv][0][0] = ROTV[i][ii][iii][0][0][0] + ROTA[i][ii][iii][iv][0][0] * TIMESTEP;

											//最大角速度を超過した場合，最大角速度に丸める
											if (ROTV[i][ii][iii][iv][0][0] > ROBOT_MAX_RAD) {
												ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][0][0][0] + ROBOT_MAX_RAD * TIMESTEP;
												if (ROT[i][ii][iii][iv][0][0] > PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] - 2 * PI;
												}
												else if (ROT[i][ii][iii][iv][0][0] < -1 * PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] + 2 * PI;
												}
											}
											else if (ROTV[i][ii][iii][iv][0][0] < -1 * ROBOT_MAX_RAD) {
												ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][0][0][0] - ROBOT_MAX_RAD * TIMESTEP;
												if (ROT[i][ii][iii][iv][0][0] > PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] - 2 * PI;
												}
												else if (ROT[i][ii][iii][iv][0][0] < -1 * PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] + 2 * PI;
												}
											}
											else {
												ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][0][0][0] + ROTV[i][ii][iii][0][0][0] * TIMESTEP + ROTA[i][ii][iii][iv][0][0] * TIMESTEP * TIMESTEP / 2;
												if (ROT[i][ii][iii][iv][0][0] > PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] - 2 * PI;
												}
												else if (ROT[i][ii][iii][iv][0][0] < -1 * PI) {
													ROT[i][ii][iii][iv][0][0] = ROT[i][ii][iii][iv][0][0] + 2 * PI;
												}
											}
											A[i][ii][iii][iv][0][0] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
											V[i][ii][iii][iv][0][0] = V[i][ii][iii][0][0][0] + A[i][ii][iii][iv][0][0] * TIMESTEP;
											//最大速度を超過した場合，最大速度に丸める
											if (V[i][ii][iii][iv][0][0] > ROBOT_MAX_VELOCITY) {
												DIST[i][ii][iii][iv][0][0] = ROBOT_MAX_VELOCITY * TIMESTEP;
											}
											else if (V[i][ii][iii][iv][0][0] < -1 * ROBOT_MAX_VELOCITY) {
												DIST[i][ii][iii][iv][0][0] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
											}
											else {
												DIST[i][ii][iii][iv][0][0] = V[i][ii][iii][0][0][0] * TIMESTEP + A[i][ii][iii][iv][0][0] * TIMESTEP * TIMESTEP / 2;
											}
											X[i][ii][iii][iv][0][0] = X[i][ii][iii][0][0][0] + DIST[i][ii][iii][iv][0][0] * cos(ROT[i][ii][iii][iv][0][0]);
											Y[i][ii][iii][iv][0][0] = Y[i][ii][iii][0][0][0] + DIST[i][ii][iii][iv][0][0] * sin(ROT[i][ii][iii][iv][0][0]);
											if (X[i][ii][iii][iv][0][0] > AREA_X) {
												X[i][ii][iii][iv][0][0] = AREA_X;
											}
											else if (X[i][ii][iii][iv][0][0] < 0) {
												X[i][ii][iii][iv][0][0] = 0;
											}
											if (Y[i][ii][iii][iv][0][0] > AREA_Y) {
												Y[i][ii][iii][iv][0][0] = AREA_Y;
											}
											else if (Y[i][ii][iii][iv][0][0] < 0) {
												Y[i][ii][iii][iv][0][0] = 0;
											}

											GD[i][ii][iii][iv][0][0] = sqrt(pow(GOAL_X - X[i][ii][iii][iv][0][0], 2) + pow(GOAL_Y - Y[i][ii][iii][iv][0][0], 2));

											xP = X[i][ii][iii][iv][0][0];
											yP = Y[i][ii][iii][iv][0][0];
											xV = V[i][ii][iii][iv][0][0];
											humanXP = humanX + 4 * humanV * TIMESTEP * cos(humanTheta);
											humanYP = humanY + 4 * humanV * TIMESTEP * sin(humanTheta);
											q[i][ii][iii][iv][0][0] = safety();

											PP[i][ii][iii][iv][0][0] = P;
											RISKP[i][ii][iii][iv][0][0] = R;
											RISKT[i][ii][iii][iv][0][0] = RISKP[i][0][0][0][0][0] + RISKP[i][ii][0][0][0][0] + RISKP[i][ii][iii][0][0][0] + RISKP[i][ii][iii][iv][0][0];

											objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

											if (q[i][ii][iii][iv][0][0] == 0 /*&& objDist > colliC*/) {
												//printf("[%d][%d][%d][%d] is safe\n", i, ii, iii, iv);
												for (int v = 1; v < EVERY_TIMESTEP_NUM; ++v) {
													tp = 1 * TIMESTEP;

													ROTA[i][ii][iii][iv][v][0] = ROTA[i][ii][iii][iv][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
													ROTV[i][ii][iii][iv][v][0] = ROTV[i][ii][iii][iv][0][0] + ROTA[i][ii][iii][iv][v][0] * TIMESTEP;

													//最大角速度を超過した場合，最大角速度に丸める
													if (ROTV[i][ii][iii][iv][v][0] > ROBOT_MAX_RAD) {
														ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][0][0] + ROBOT_MAX_RAD * TIMESTEP;
														if (ROT[i][ii][iii][iv][v][0] > PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] - 2 * PI;
														}
														else if (ROT[i][ii][iii][iv][v][0] < -1 * PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] + 2 * PI;
														}
													}
													else if (ROTV[i][ii][iii][iv][v][0] < -1 * ROBOT_MAX_RAD) {
														ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][0][0] - ROBOT_MAX_RAD * TIMESTEP;
														if (ROT[i][ii][iii][iv][v][0] > PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] - 2 * PI;
														}
														else if (ROT[i][ii][iii][iv][v][0] < -1 * PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] + 2 * PI;
														}
													}
													else {
														ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][0][0] + ROTV[i][ii][iii][iv][0][0] * TIMESTEP + ROTA[i][ii][iii][iv][v][0] * TIMESTEP * TIMESTEP / 2;
														if (ROT[i][ii][iii][iv][v][0] > PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] - 2 * PI;
														}
														else if (ROT[i][ii][iii][iv][v][0] < -1 * PI) {
															ROT[i][ii][iii][iv][v][0] = ROT[i][ii][iii][iv][v][0] + 2 * PI;
														}
													}
													A[i][ii][iii][iv][v][0] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
													V[i][ii][iii][iv][v][0] = V[i][ii][iii][iv][0][0] + A[i][ii][iii][iv][v][0] * TIMESTEP;
													//最大速度を超過した場合，最大速度に丸める
													if (V[i][ii][iii][iv][v][0] > ROBOT_MAX_VELOCITY) {
														DIST[i][ii][iii][iv][v][0] = ROBOT_MAX_VELOCITY * TIMESTEP;
													}
													else if (V[i][ii][iii][iv][v][0] < -1 * ROBOT_MAX_VELOCITY) {
														DIST[i][ii][iii][iv][v][0] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
													}
													else {
														DIST[i][ii][iii][iv][v][0] = V[i][ii][iii][iv][0][0] * TIMESTEP + A[i][ii][iii][iv][v][0] * TIMESTEP * TIMESTEP / 2;
													}
													X[i][ii][iii][iv][v][0] = X[i][ii][iii][iv][0][0] + DIST[i][ii][iii][iv][v][0] * cos(ROT[i][ii][iii][iv][v][0]);
													Y[i][ii][iii][iv][v][0] = Y[i][ii][iii][iv][0][0] + DIST[i][ii][iii][iv][v][0] * sin(ROT[i][ii][iii][iv][v][0]);
													if (X[i][ii][iii][iv][v][0] > AREA_X) {
														X[i][ii][iii][iv][v][0] = AREA_X;
													}
													else if (X[i][ii][iii][iv][v][0] < 0) {
														X[i][ii][iii][iv][v][0] = 0;
													}
													if (Y[i][ii][iii][iv][v][0] > AREA_Y) {
														Y[i][ii][iii][iv][v][0] = AREA_Y;
													}
													else if (Y[i][ii][iii][iv][v][0] < 0) {
														Y[i][ii][iii][iv][v][0] = 0;
													}

													GD[i][ii][iii][iv][v][0] = sqrt(pow(GOAL_X - X[i][ii][iii][iv][v][0], 2) + pow(GOAL_Y - Y[i][ii][iii][iv][v][0], 2));

													xP = X[i][ii][iii][iv][v][0];
													yP = Y[i][ii][iii][iv][v][0];
													xV = V[i][ii][iii][iv][v][0];
													humanXP = humanX + 5 * humanV * TIMESTEP * cos(humanTheta);
													humanYP = humanY + 5 * humanV * TIMESTEP * sin(humanTheta);
													q[i][ii][iii][iv][v][0] = safety();

													PP[i][ii][iii][iv][v][0] = P;
													RISKP[i][ii][iii][iv][v][0] = R;
													RISKT[i][ii][iii][iv][v][0] = RISKP[i][0][0][0][0][0] + RISKP[i][ii][0][0][0][0] + RISKP[i][ii][iii][0][0][0] + RISKP[i][ii][iii][iv][0][0] + RISKP[i][ii][iii][iv][v][0];

													objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

													if (q[i][ii][iii][iv][v][0] == 0 /*&& objDist > colliC*/) {
														//printf("[%d][%d][%d][%d][%d] is safe\n", i, ii, iii, iv, v);
														for (int vi = 1; vi < EVERY_TIMESTEP_NUM; ++vi) {
															tp = 1 * TIMESTEP;

															ROTA[i][ii][iii][iv][v][vi] = ROTA[i][ii][iii][iv][v][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
															ROTV[i][ii][iii][iv][v][vi] = ROTV[i][ii][iii][iv][v][0] + ROTA[i][ii][iii][iv][v][vi] * TIMESTEP;

															//最大角速度を超過した場合，最大角速度に丸める
															if (ROTV[i][ii][iii][iv][v][vi] > ROBOT_MAX_RAD) {
																ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][0] + ROBOT_MAX_RAD * TIMESTEP;
																if (ROT[i][ii][iii][iv][v][vi] > PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] - 2 * PI;
																}
																else if (ROT[i][ii][iii][iv][v][vi] < -1 * PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] + 2 * PI;
																}
															}
															else if (ROTV[i][ii][iii][iv][v][vi] < -1 * ROBOT_MAX_RAD) {
																ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][0] - ROBOT_MAX_RAD * TIMESTEP;
																if (ROT[i][ii][iii][iv][v][vi] > PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] - 2 * PI;
																}
																else if (ROT[i][ii][iii][iv][v][vi] < -1 * PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] + 2 * PI;
																}
															}
															else {
																ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][0] + ROTV[i][ii][iii][iv][v][0] * TIMESTEP + ROTA[i][ii][iii][iv][v][vi] * TIMESTEP * TIMESTEP / 2;
																if (ROT[i][ii][iii][iv][v][vi] > PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] - 2 * PI;
																}
																else if (ROT[i][ii][iii][iv][v][vi] < -1 * PI) {
																	ROT[i][ii][iii][iv][v][vi] = ROT[i][ii][iii][iv][v][vi] + 2 * PI;
																}
															}
															A[i][ii][iii][iv][v][vi] = (double)rand() / 32767.0 * 3 / 2 * robPAcc - 1 / 2 * robPAcc;
															V[i][ii][iii][iv][v][vi] = V[i][ii][iii][iv][v][0] + A[i][ii][iii][iv][v][vi] * TIMESTEP;
															//最大速度を超過した場合，最大速度に丸める
															if (V[i][ii][iii][iv][v][vi] > ROBOT_MAX_VELOCITY) {
																DIST[i][ii][iii][iv][v][vi] = ROBOT_MAX_VELOCITY * TIMESTEP;
															}
															else if (V[i][ii][iii][iv][v][vi] < -1 * ROBOT_MAX_VELOCITY) {
																DIST[i][ii][iii][iv][v][vi] = -1 * ROBOT_MAX_VELOCITY * TIMESTEP;
															}
															else {
																DIST[i][ii][iii][iv][v][vi] = V[i][ii][iii][iv][v][0] * TIMESTEP + A[i][ii][iii][iv][v][vi] * TIMESTEP * TIMESTEP / 2;
															}
															X[i][ii][iii][iv][v][vi] = X[i][ii][iii][iv][v][0] + DIST[i][ii][iii][iv][v][vi] * cos(ROT[i][ii][iii][iv][v][vi]);
															Y[i][ii][iii][iv][v][vi] = Y[i][ii][iii][iv][v][0] + DIST[i][ii][iii][iv][v][vi] * sin(ROT[i][ii][iii][iv][v][vi]);
															if (X[i][ii][iii][iv][v][vi] > AREA_X) {
																X[i][ii][iii][iv][v][vi] = AREA_X;
															}
															else if (X[i][ii][iii][iv][v][vi] < 0) {
																X[i][ii][iii][iv][v][vi] = 0;
															}
															if (Y[i][ii][iii][iv][v][vi] > AREA_Y) {
																Y[i][ii][iii][iv][v][vi] = AREA_Y;
															}
															else if (Y[i][ii][iii][iv][v][vi] < 0) {
																Y[i][ii][iii][iv][v][vi] = 0;
															}

															GD[i][ii][iii][iv][v][vi] = sqrt(pow(GOAL_X - X[i][ii][iii][iv][v][vi], 2) + pow(GOAL_Y - Y[i][ii][iii][iv][v][vi], 2));

															xP = X[i][ii][iii][iv][v][vi];
															yP = Y[i][ii][iii][iv][v][vi];
															xV = V[i][ii][iii][iv][v][vi];
															humanXP = humanX + 6 * humanV * TIMESTEP * cos(humanTheta);
															humanYP = humanY + 6 * humanV * TIMESTEP * sin(humanTheta);
															q[i][ii][iii][iv][v][vi] = safety();

															PP[i][ii][iii][iv][v][vi] = P;
															RISKP[i][ii][iii][iv][v][vi] = R;
															RISKT[i][ii][iii][iv][v][vi] = RISKP[i][0][0][0][0][0] + RISKP[i][ii][0][0][0][0] + RISKP[i][ii][iii][0][0][0] + RISKP[i][ii][iii][iv][0][0] + RISKP[i][ii][iii][iv][v][0] + RISKP[i][ii][iii][iv][v][vi];

															objDist = sqrt(pow(humanX - xP, 2) + pow(humanY - yP, 2));

															if (q[i][ii][iii][iv][v][vi] == 0 /*&& objDist > colliC*/) {
																//printf("[%d][%d][%d][%d][%d][%d] is safe\n", i, ii, iii, iv, v, vi);
															}
															else {
																GD[i][ii][iii][iv][v][vi] = INFINITY;
															}
														}
													}
													else {
														GD[i][ii][iii][iv][v][0] = INFINITY;
													}
												}

											}
											else {
												GD[i][ii][iii][iv][0][0] = INFINITY;
											}
										}
									}
									else {
										GD[i][ii][iii][0][0][0] = INFINITY;
									}
								}
							}
							else {
								GD[i][ii][0][0][0][0] = INFINITY;
							}
						}
					}
					else {
						GD[i][0][0][0][0][0] = INFINITY;
					}
				}
				tp = TIMESTEP;

				/*
				for (int i = 0; i < EVERY_TIMESTEP_NUM; ++i) {
				printf("GD[%d][][] = %0.10lf\n", i, GD[i]);
				}
				for (int i = 0; i < EVERY_TIMESTEP_NUM; ++i) {
				printf("X[%d] = %0.10lf\n", i, X[i]);
				printf("Y[%d] = %0.10lf\n", i, Y[i]);
				}
				for (int i = 0; i < EVERY_TIMESTEP_NUM; ++i) {
				printf("V[%d] = %0.10lf\n", i, V[i]);
				}
				for (int i = 0; i < EVERY_TIMESTEP_NUM; ++i) {
				printf("DIST[%d] = %0.10lf\n", i, DIST[i]);
				}
				*/





				// 目標位置に最も近い（＝最高効率）の回避軌道を探索
				minGD = GD[0][0][0][0][0][0];
				//printf("G = %p\n", GD[0]);
				G = 0;
				for (int t1 = 1; t1 < EVERY_TIMESTEP_NUM; ++t1) {
					for (int t2 = 0; t2 < EVERY_TIMESTEP_NUM; ++t2) {
						for (int t3 = 0; t3 < EVERY_TIMESTEP_NUM; ++t3) {
							for (int t4 = 0; t4 < EVERY_TIMESTEP_NUM; ++t4) {
								for (int t5 = 0; t5 < EVERY_TIMESTEP_NUM; ++t5) {
									for (int t6 = 0; t6 < EVERY_TIMESTEP_NUM; ++t6) {
										for (int t7 = 0; t7 < EVERY_TIMESTEP_NUM; ++t7) {
											if (RISKT[t1][t2][t3][t4][t5][t6] < 0.00000011) {
												if (GD[t1][t2][t3][t4][t5][t6] != 0) {
													if (minGD > GD[t1][t2][t3][t4][t5][t6]) {
														minGD = GD[t1][t2][t3][t4][t5][t6];
														G = t1;
														nowR = RISKP[t1][0][0][0][0][0];
														P = PP[t1][0][0][0][0][0];
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

				// 目標位置に最も近いパーティクルから現在位置までのパーティクル座標を取得

				p = G;

				if (p == 0) {
					//安全な経路が存在しないので停止する
					//printf("safe route is not detected! route replaning...\n");

					/*
					w = robX;
					x = robY;
					y = humanX;
					z = humanY;
					objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
					nowR = R;
					string p2string(std::to_string(P));
					fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
					//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", w, x, y, z, objDist, nowR, P, S, st);
					*/
					st = 0;
					//printf("stop=%d\n", st);
				}
				else {
					//安全な経路の一歩目を移動
					//printf("selected route is %d\n", p);
					robX = X[p][0][0][0][0][0];
					robY = Y[p][0][0][0][0][0];
					robRot = ROT[p][0][0][0][0][0];
					robV = V[p][0][0][0][0][0];
					robRotV = ROTV[p][0][0][0][0][0];
					gD = sqrt(pow(GOAL_X - robX, 2) + pow(GOAL_Y - robY, 2));
					printf("robot pos=(%0.10lf,%0.10lf)\n", robX, robY);

					xP = robX;
					yP = robY;
					xV = robV;
					humanXP = humanX;
					humanYP = humanY;

					//safety();

					humanMoving();

					//safety();

					w = robX;
					x = robY;
					y = humanX;
					z = humanY;
					objDist = sqrt(pow(humanX - robX, 2) + pow(humanY - robY, 2));
					//nowR = R;
					//string p2string(std::to_string(P));

					//fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%s,%0.10lf\n", w, x, y, z, objDist, nowR, p2string.c_str(), S);
					fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", w, x, y, z, objDist, nowR, P, S, st);



					if (gD < GOAL_DISTANCE) {
						break;
					}
				}

				for (int t1 = 1; t1 < EVERY_TIMESTEP_NUM; ++t1) {
					for (int t2 = 0; t2 < EVERY_TIMESTEP_NUM; ++t2) {
						for (int t3 = 0; t3 < EVERY_TIMESTEP_NUM; ++t3) {
							for (int t4 = 0; t4 < EVERY_TIMESTEP_NUM; ++t4) {
								for (int t5 = 0; t5 < EVERY_TIMESTEP_NUM; ++t5) {
									for (int t6 = 0; t6 < EVERY_TIMESTEP_NUM; ++t6) {
										for (int t7 = 0; t7 < EVERY_TIMESTEP_NUM; ++t7) {
											//配列の初期化
											X[t1][t2][t3][t4][t5][t6] = 0.0;
											Y[t1][t2][t3][t4][t5][t6] = 0.0;
											V[t1][t2][t3][t4][t5][t6] = 0.0;
											A[t1][t2][t3][t4][t5][t6] = 0.0;
											ROT[t1][t2][t3][t4][t5][t6] = 0.0;
											ROTV[t1][t2][t3][t4][t5][t6] = 0.0;
											ROTA[t1][t2][t3][t4][t5][t6] = 0.0;
											DIST[t1][t2][t3][t4][t5][t6] = 0.0;
											GD[t1][t2][t3][t4][t5][t6] = 0.0;
											RISKP[t1][t2][t3][t4][t5][t6] = 0.0;
											RISKT[t1][t2][t3][t4][t5][t6] = 0.0;
											PP[t1][t2][t3][t4][t5][t6] = 0.0;
											q[t1][t2][t3][t4][t5][t6] = 0;

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






	//printf("human pos=(%0.10lf,%0.10lf)\n", humanX, humanY);
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
	double robRotH = 0.0;		//ロボットから見た人間の方向[rad]

	robRotH = atan2(humanYP - robY, humanXP - robX);

	//予測目標の算出
	xP = xP + colliC * cos(robRotH);
	yP = yP + colliC * sin(robRotH);

	humanVP = sqrt(pow(xP - humanXP, 2) + pow(yP - humanYP, 2)) / tp;
	//printf("humanVP=%0.10lf\n", humanVP);

	if (humanVP > HUMAN_MAX_VELOCITY) {
		P = 0;						//人間が移動できる速度を超えているので人間の存在確率は0
									//printf("P=%0.10lf\n", P);
	}
	else {
		humanThetaP = atan2(yP - humanYP, xP - humanXP) - humanTheta;
		if (humanThetaP > PI) {
			humanThetaP = humanThetaP - 2 * PI;
		}
		else if (humanThetaP < -1 * PI) {
			humanThetaP = humanThetaP + 2 * PI;
		}
		//printf("humanThetaP=%0.10lf\n", humanThetaP);

		robThetaP = atan2(humanYP - yP, humanXP - xP);

		SIGMA_THETA = (1.35 * PI) / (8 * (humanV + 0.00001));
		//printf("SIGMA_THETA=%0.10lf\n", SIGMA_THETA);


		VV = (humanVP - miuV) / SIGMA_VELOCITY / 1;
		//printf("VV=%0.10lf\n", VV);

		VO = humanThetaP / SIGMA_THETA * 1;
		//printf("VO=%0.10lf\n", VO);

		A = exp((pow(VV, 2) + pow(VO, 2)) / -2);
		//printf("A=%0.10lf\n", A);

		B = 2 * PI * SIGMA_VELOCITY * SIGMA_THETA;
		//printf("B=%0.10lf\n", B);

		P = A / B;
		//printf("P=%0.10lf	", P);
	}




	//ある点における危害の酷さの計算
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);

	/*
	//人間とロボットの進路が交わらない場合，相対速度が存在しないものとする
	if (relVX < 0 || relVY < 0) {
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	}
	else {
	rV = 0.0;
	}
	*/
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	S = HUMAN_WEIGHT * ROBOT_WEIGHT * pow(rV, 2) / (HUMAN_WEIGHT + ROBOT_WEIGHT) / 2;
	//printf("S=%0.10lf	", S);

	//衝突リスクの計算
	R = P * S;
	//printf("risk num=%0.10lf\n", R);

	//安全性の判定
	if (R > 0.00000011 / TIMESTEP_NUM) {
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

	robRotH = atan2(humanYP - robY, humanXP - robX);

	xP = xP + colliC * cos(robRotH);
	yP = yP + colliC * sin(robRotH);

	humanVP = sqrt(pow(xP - humanXP, 2) + pow(yP - humanYP, 2)) / tp;
	//printf("humanVP=%0.10lf\n", humanVP);

	if (humanVP > HUMAN_MAX_VELOCITY) {
		P = 0;						//人間が移動できる速度を超えているので人間の存在確率は0
									//printf("P=%0.10lf\n", P);
	}
	else {
		humanThetaP = atan2(yP - humanYP, xP - humanXP) - humanTheta;
		if (humanThetaP > PI) {
			humanThetaP = humanThetaP - 2 * PI;
		}
		else if (humanThetaP < -1 * PI) {
			humanThetaP = humanThetaP + 2 * PI;
		}
		//printf("humanThetaP=%0.10lf\n", humanThetaP);

		robThetaP = atan2(humanYP - yP, humanXP - xP);

		SIGMA_THETA = (1.35 * PI) / (8 * (humanV + 0.00001));
		//printf("SIGMA_THETA=%0.10lf\n", SIGMA_THETA);


		VV = (humanVP - miuV) / SIGMA_VELOCITY / 1;
		//printf("VV=%0.10lf\n", VV);

		VO = humanThetaP / SIGMA_THETA * 1;
		//printf("VO=%0.10lf\n", VO);

		A = exp((pow(VV, 2) + pow(VO, 2)) / -2);
		//printf("A=%0.10lf\n", A);

		B = 2 * PI * SIGMA_VELOCITY * SIGMA_THETA;
		//printf("B=%0.10lf\n", B);

		P = A / B;
		//printf("P=%0.10lf\n", P);
	}


	/*
	//ある点における危害の酷さの計算
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	S = HUMAN_WEIGHT * ROBOT_WEIGHT * pow(rV, 2) / (HUMAN_WEIGHT + ROBOT_WEIGHT) / 2;
	//printf("S=%0.10lf\n", S);
	//衝突リスクの計算
	R = P * S;
	//printf("risk num=%0.10lf\n", R);
	*/

	//回避軌道生成開始の判定
	if (P > 0.0000001) {
		return 1;
	}
	else {
		return 0;
	}
}

int humanMoving() {
	//人間のタイムステップ毎の移動を処理
	//humanX = humanX + humanV * TIMESTEP * cos((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanY = humanY + humanV * TIMESTEP * sin((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanX = humanX - humanV * TIMESTEP;
	//humanY = humanY - humanV * TIMESTEP;

	//humanGD = sqrt(pow(humanX - HUMAN_GOAL_X, 2) + pow(humanY - HUMAN_GOAL_Y, 2));
	//if (humanGD > HUMAN_GOAL_DISTANCE) {
	humanX = humanX + humanV * TIMESTEP * cos(humanTheta);
	humanY = humanY + humanV * TIMESTEP * sin(humanTheta);
	//}



	/*//方向転換45°
	if (humanX > 2.5) {
	humanX = humanX - humanV * TIMESTEP;
	}
	else {
	humanTheta = PI * 1 / 4;
	humanX = humanX + humanV * TIMESTEP;
	humanY = humanY + humanV * TIMESTEP;
	}*/

	/*//方向転換180°
	if (frag == 1) {
	humanTheta = PI * 0 / 4;
	humanX = humanX + humanV * TIMESTEP;
	//humanY = humanY - humanV * TIMESTEP;
	}
	else if (humanX > 2.5) {
	humanX = humanX - humanV * TIMESTEP;
	}
	else {
	frag = 1;
	humanY = humanY - 0.5;
	}*/

	return 0;
}