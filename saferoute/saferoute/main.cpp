#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include "common.h"
#include <string>

#define _USE_MATH_DEFINES //���l���Z�萔����`���ꂽ�w�b�_�t�@�C���̓ǂݍ���

#define PI 3.14159265359

FILE* fp; //FILE�|�C���^�̐錾

using std::string;


double humanX = 0.0, humanY = 0.0;	        //�l�Ԃ̌��݈ʒu[m]
double humanTheta = 0.0;		        //�l�Ԃ̌��݂̕���[��]
double humanXP, humanYP;				//�z�肳���l�Ԃ̈ʒu[m]
double xP = 0.0, yP = 0.0;			        //�l�Ԃ̈ړ��\���_[ml
double xV = 0.0;					//���{�b�g�̗\�����x[m/s]
double colliC = 0;				//�l�Ԃƃ��{�b�g�̐ڐG����[m]

								//double humanV = 1.4 / 2;	        //�l�Ԃ̈ړ����x[m/s]
double humanV = 0.7;								//double humanV = 1.4;

double humanVP = 0.0;			        //�l�Ԃ̈ړ��m����

double humanThetaP = 0.0;		        //�l�Ԃ̌��݈ʒu����݂�����_�̕���[��]
double robThetaP = 0.0;					//���{�b�g�̂���ʒu���猩�����݂̐l�Ԃ̕���[��]


double miuV = 0.0;			        //�l�Ԃ̈ړ����x�̊��Ғl[m/s]

double SIGMA_THETA = 0.0; //�l�Ԃ̐i�s�����̕W���΍�

double tp = 0.0;                      //�l�Ԃ̈ړ��\���̎���[s]


double VV = 0.0;				        //�ϐ��P
double VO = 0.0;				        //�ϐ��Q


double P = 0.0;				        //����_�̊m�����x
double S = 0.0;				        //����_�̊�Q�̍���
double R = 0.0;				        //�Փ˃��X�N�l

double robX = 0.0, robY = 0.0;		        //���{�b�g�̈ʒu[m]
double robRot = 0.0;			        //���{�b�g�̌��ݕ���[rad]
double robV = 0.0;			//���{�b�g�̌��ݑ��x[m/s]
double robRotV = 0.0;			//���{�b�g�̌��݊p���x[rad/s]

								//double ROBOT_MAX_RAD = 1.57;
double robDist = 0.0;			//���{�b�g���i�ދ���[m]
int pN;					        //��̃p�[�e�B�N������L�т�}�̐�

double humanGD = 0.0;				//���݂̐l�ԂƐl�Ԃ̖ړI�ʒu�̋���[m]
double gD = 0.0;				        //���݂̃��{�b�g�ƖڕW�ʒu�̋���[m]
double objDist = 0.0;			        //�l�Ԃƃ��{�b�g�̋���[m]
double nowR = 0.0;						//���݂̏Փ˃��X�N�l
double X[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM], Y[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];		        //N�Ԗڂ̃p�[�e�B�N���̍��W[m]
double V[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                    //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̑��x[m/s]
double A[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                    //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̉����x[m/s^2]
double ROT[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                  //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̌���[rad]
double ROTV[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̊p���x[rad/s]
double ROTA[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̊p�����x[rad/s^2]
double DIST[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];                 //N�Ԗڂ̃p�[�e�B�N���̐i�ދ���[m]
double GD[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];			        //�e�p�[�e�B�N���ƖڕW�ʒu�̋���[m]
double RISKP[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				//�e����O���o�R�_���̏Փ˃��X�N�l
double RISKT[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				//�e����O�����̐ϕ����X�N�l
double PP[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];					//�e����O���o�R�_���ɂ�����l�Ԃ̑��݊m��
																																					//int prev_p[EVERY_TIMESTEP_NUM];			        //N�Ԗڂ̃p�[�e�B�N���Ɛڑ�����O�̃p�[�e�B�N���ԍ�

double robXP = 0.0, robYP = 0.0;	//�ŒZ�ړ�����ۂ̎��̃��{�b�g�̗\���ʒu[m/s]
double robRotP = 0.0;				//�ŒZ�ړ�����ۂ̎��̃��{�b�g�̗\���p�x[rad]
double robRotMax = 0.0;				//�P�ʃ^�C���X�e�b�v�ɂ����郍�{�b�g�̍ő����p�x[rad]
double robRotC = 0.0;				//�P�ʃ^�C���X�e�b�v�ɂ�����ŒZ�ړ��̍ۂ̃��{�b�g�̐���p�x[rad]


int frag = 0;

int q[EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM][EVERY_TIMESTEP_NUM];				        //���S���]��yes/no=0/1
int r;							//����O����������no/yes=0/1
int s;                          //�]���l�o�͗p
int safe = 0;						//���S�]��yes/no=0/1

int st = 1;						//��~�]��yes/no=0/1

								//int SELECT[6];					//�I�����ꂽ���[�g�ԍ�

int routing();
int safety();
int humanMoving();

int main() {
	printf("main start\n");

	//�����̏�����
	srand(time(NULL));

	//���{�b�g�̌��݈ʒu�𑪒�
	robX = ROBOT_X_INIT;
	robY = ROBOT_Y_INIT;
	robV = 0.0;
	robRot = ROBOT_RAD_INIT;
	robRotMax = ROBOT_MAX_RAD * TIMESTEP;

	//�l�Ԃ̌��݈ʒu�𑪒�
	humanX = 1.5;
	humanY = 4.0;
	humanTheta = PI * -1 / 2;
	miuV = humanV;

	colliC = HUMAN_RADIUS + ROBOT_RADIUS;

	//�\����Ԃ̐���
	double robPAcc = 1.4;				//�\����Ԃ̐�̔��a[m/s^2]
	double robPRotAcc = 2.09;			//�\����Ԃ̐�p[rad/s^2]

										//���ʕ\���t�@�C���̍쐬
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

	//�������Ƃ̏��ϐ���\��
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,", w, x, y, z);
	fprintf(fp, "%0.10lf,%0.10lf,%0.10lf,%0.10lf,%d\n", objDist, nowR, P, S, st);



	//printf("human pos=(%0.10lf,%0.10lf)\n", humanX, humanY);

	//���{�b�g���ړI�ʒu�ɓ��B���Ă��Ȃ��ꍇ�C�ړ����p������
	while (robX != GOAL_X || robY != GOAL_Y)
	{
		//printf("robot start moving\n");

		//���{�b�g���댯����̂��߂̒�~������s���Ă��邩
		if (st == 0) {
			//printf("robot stopping\n");
			tp = 0.5;

			//�p���x��0�ɋ߂Â���
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

			//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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
				//���{�b�g���l�Ԃ���\���ɗ���Ă���ꍇ�C�ڕW�ʒu�ɒ��i
				//printf("robot moving straight\n");
				//robRot = atan((GOAL_Y  - robY) / (GOAL_X - robX));
				robRotP = atan2(GOAL_Y - robY, GOAL_X - robX);
				robRotC = robRotP - robRotP;
				//�P�ʃ^�C���X�e�b�v�ɂ�����ő����p�x�𒴉߂����ꍇ�C�ő����p�x�Ɋۂ߂�
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
				//�l�Ԃ��ڋ߂��Ă���ꍇ�C�p�[�e�B�N���T�����J�n


				//�����x���l����������O������
				printf("robot searching route\n");
				double minGD = 0;
				int G = 0;
				int p = 0;


				//�p�[�e�B�N����
				X[0][0][0][0][0][0] = robX;
				Y[0][0][0][0][0][0] = robY;

				ROT[0][0][0][0][0][0] = robRot;
				V[0][0][0][0][0][0] = robV;
				ROTV[0][0][0][0][0][0] = robRotV;
				A[0][0][0][0][0][0] = 0;
				ROTA[0][0][0][0][0][0] = 0;

				GD[0][0][0][0][0][0] = sqrt(pow(GOAL_X - X[0][0][0][0][0][0], 2) + pow(GOAL_Y - Y[0][0][0][0][0][0], 2));

				//�e����O���o�R�_�������_���Ȉʒu�ɐ������C���S�����m�F����
				for (int i = 1; i < EVERY_TIMESTEP_NUM; i++) {
					tp = TIMESTEP;

					ROTA[i][0][0][0][0][0] = ROTA[0][0][0][0][0][0] + (double)rand() / 32767.0 * 2 * robPRotAcc - robPRotAcc;
					ROTV[i][0][0][0][0][0] = ROTV[0][0][0][0][0][0] + ROTA[i][0][0][0][0][0] * TIMESTEP;

					//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
					//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

							//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
							//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

									//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
									//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

											//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
											//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

													//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
													//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

															//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
															//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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





				// �ڕW�ʒu�ɍł��߂��i���ō������j�̉���O����T��
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

				// �ڕW�ʒu�ɍł��߂��p�[�e�B�N�����猻�݈ʒu�܂ł̃p�[�e�B�N�����W���擾

				p = G;

				if (p == 0) {
					//���S�Ȍo�H�����݂��Ȃ��̂Œ�~����
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
					//���S�Ȍo�H�̈���ڂ��ړ�
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
											//�z��̏�����
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

	//�ړ��������I���������Ƃ̍��}
	printf("done!");
}

//���S���̔���֐��i���S�Ȃ�0�C�댯�Ȃ�1��Ԃ��j
int safety() {
	//printf("safety function\n");
	//�l�Ԃ̂���_�ɂ����鑶�݊m�����x�̌v�Z
	double A, B;
	double robVX = 0.0, robVY = 0.0;
	double relVX = 0.0, relVY = 0.0, rV = 0.0;
	double robRotH = 0.0;		//���{�b�g���猩���l�Ԃ̕���[rad]

	robRotH = atan2(humanYP - robY, humanXP - robX);

	//�\���ڕW�̎Z�o
	xP = xP + colliC * cos(robRotH);
	yP = yP + colliC * sin(robRotH);

	humanVP = sqrt(pow(xP - humanXP, 2) + pow(yP - humanYP, 2)) / tp;
	//printf("humanVP=%0.10lf\n", humanVP);

	if (humanVP > HUMAN_MAX_VELOCITY) {
		P = 0;						//�l�Ԃ��ړ��ł��鑬�x�𒴂��Ă���̂Ől�Ԃ̑��݊m����0
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




	//����_�ɂ������Q�̍����̌v�Z
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);

	/*
	//�l�Ԃƃ��{�b�g�̐i�H�������Ȃ��ꍇ�C���Α��x�����݂��Ȃ����̂Ƃ���
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

	//�Փ˃��X�N�̌v�Z
	R = P * S;
	//printf("risk num=%0.10lf\n", R);

	//���S���̔���
	if (R > 0.00000011 / TIMESTEP_NUM) {
		return 1;
	}
	else {
		return 0;
	}
}

//����O�������J�n����֐��i�J�n���Ȃ��Ȃ�0�C�J�n����Ȃ�1��Ԃ��j
int routing() {
	//printf("routing function\n");
	//�l�Ԃ̂���_�ɂ����鑶�݊m�����x�̌v�Z
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
		P = 0;						//�l�Ԃ��ړ��ł��鑬�x�𒴂��Ă���̂Ől�Ԃ̑��݊m����0
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
	//����_�ɂ������Q�̍����̌v�Z
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	S = HUMAN_WEIGHT * ROBOT_WEIGHT * pow(rV, 2) / (HUMAN_WEIGHT + ROBOT_WEIGHT) / 2;
	//printf("S=%0.10lf\n", S);
	//�Փ˃��X�N�̌v�Z
	R = P * S;
	//printf("risk num=%0.10lf\n", R);
	*/

	//����O�������J�n�̔���
	if (P > 0.0000001) {
		return 1;
	}
	else {
		return 0;
	}
}

int humanMoving() {
	//�l�Ԃ̃^�C���X�e�b�v���̈ړ�������
	//humanX = humanX + humanV * TIMESTEP * cos((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanY = humanY + humanV * TIMESTEP * sin((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanX = humanX - humanV * TIMESTEP;
	//humanY = humanY - humanV * TIMESTEP;

	//humanGD = sqrt(pow(humanX - HUMAN_GOAL_X, 2) + pow(humanY - HUMAN_GOAL_Y, 2));
	//if (humanGD > HUMAN_GOAL_DISTANCE) {
	humanX = humanX + humanV * TIMESTEP * cos(humanTheta);
	humanY = humanY + humanV * TIMESTEP * sin(humanTheta);
	//}



	/*//�����]��45��
	if (humanX > 2.5) {
	humanX = humanX - humanV * TIMESTEP;
	}
	else {
	humanTheta = PI * 1 / 4;
	humanX = humanX + humanV * TIMESTEP;
	humanY = humanY + humanV * TIMESTEP;
	}*/

	/*//�����]��180��
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