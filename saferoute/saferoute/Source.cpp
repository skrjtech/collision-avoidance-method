#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>

#include <string>

#define _USE_MATH_DEFINES //���l���Z�萔����`���ꂽ�w�b�_�t�@�C���̓ǂݍ���

#define PI 3.14159265359

FILE* fp; //FILE�|�C���^�̐錾

using std::string;


double humanX = 0.0, humanY = 0.0;	        //�l�Ԃ̌��݈ʒu[m]
double humanTheta = 0.0;		        //�l�Ԃ̌��݂̕���[��]
double xP = 0.0, yP = 0.0;			        //�l�Ԃ̈ړ��\���_[ml
double xV = 0.0;					//���{�b�g�̗\�����x[m/s]
double humanC = 0.4;	        //�l�Ԃ̔��a(����)[m]
double robC = 0.35;				//���{�b�g�̔��a[m]

double colliC = 0;				//�l�Ԃƃ��{�b�g�̐ڐG����[m]

double humanV = 1.4 / 2;	        //�l�Ԃ̈ړ����x[m/s]
									//double humanV = 1.4;


double humanVP = 0.0;			        //�l�Ԃ̈ړ��m����

double humanThetaP = 0.0;		        //�l�Ԃ̌��݈ʒu����݂�����_�̕���[��]
double robThetaP = 0.0;					//���{�b�g�̂���ʒu���猩�����݂̐l�Ԃ̕���[��]

double sigV = 0.8 / 2;	        //�l�Ԃ̈ړ����x�̕W���΍�
double sigTheta = 0.0;		        //�l�Ԃ̐i�s�����̕W���΍�

double miuV = 0.0;			        //�l�Ԃ̈ړ����x�̊��Ғl[m/s]

double humanW = 4.4;		        //�l�Ԃ̎���[kg]
double robW = 15;		        //���{�b�g�̎���[kg]

double ts = 0.5 * 1;		        //�P�ʃ^�C���X�e�b�v����[s]
double tp = 0.0;                      //�l�Ԃ̈ړ��\���̎���[s]
int nts = 7;				        //�^�C���X�e�b�v��

double calD = 1.0;		        //�l�Ԃƃ��{�b�g�̋���������ȉ��ɂȂ�ƈ��S���̌v�Z���J�n����[m]

double VV = 0.0;				        //�ϐ��P
double VO = 0.0;				        //�ϐ��Q


double P = 0.0;				        //����_�̊m�����x
double S = 0.0;				        //����_�̊�Q�̍���
double R = 0.0;				        //�Փ˃��X�N�l

double robX = 0.0, robY = 0.0;		        //���{�b�g�̈ʒu[m]
double robRot = 0.0;			        //���{�b�g�̌��ݕ���[rad]
double robV = 0.0;			//���{�b�g�̌��ݑ��x[m/s]
double robRotV = 0.0;			//���{�b�g�̌��݊p���x[rad/s]
double robVMax = 1.4 / 2;			//���{�b�g�̍ő呬�x[m/s]
double robRadMax = 6.28;		//���{�b�g�̍ő���񑬓x[rad/s]


double goalX = 2.5, goalY = 5.0;	//�ڕW�ʒu[m]
double goalC = 0.5;			    //�ڕW�ʒu�̔��a[m]

								//const int N = 15;				//�p�[�e�B�N���̑���
const int N = 10 + 1;               //�p�[�e�B�N���̑����@N = 1 + pN ^ 1 + pN ^ 2 + ... + pN ^ NTS
int pN;					        //��̃p�[�e�B�N������L�т�}�̐�

double gD = 0.0;				        //���݂̃��{�b�g�ƖڕW�ʒu�̋���[m]
double objDist = 0.0;			        //�l�Ԃƃ��{�b�g�̋���[m]
double nowR = 0.0;						//���݂̏Փ˃��X�N�l
double X[N][N][N][N][N][N][N], Y[N][N][N][N][N][N][N];		        //N�Ԗڂ̃p�[�e�B�N���̍��W[m]
double V[N][N][N][N][N][N][N];                    //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̑��x[m/s]
double A[N][N][N][N][N][N][N];                    //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̉����x[m/s^2]
double ROT[N][N][N][N][N][N][N];                  //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̌���[rad]
double ROTV[N][N][N][N][N][N][N];                 //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̊p���x[rad/s]
double ROTA[N][N][N][N][N][N][N];                 //N�Ԗڂ̃p�[�e�B�N�����W�ɂ����郍�{�b�g�̊p�����x[rad/s^2]
double DIST[N][N][N][N][N][N][N];                 //N�Ԗڂ̃p�[�e�B�N���̐i�ދ���[m]
double GD[N][N][N][N][N][N][N];			        //�e�p�[�e�B�N���ƖڕW�ʒu�̋���[m]
double RISKP[N][N][N][N][N][N][N];				//�e����O���o�R�_���̏Փ˃��X�N�l
double RISKT[N][N][N][N][N][N][N];				//�e����O�����̐ϕ����X�N�l
double PP[N][N][N][N][N][N][N];					//�e����O���o�R�_���ɂ�����l�Ԃ̑��݊m��
												//int prev_p[N];			        //N�Ԗڂ̃p�[�e�B�N���Ɛڑ�����O�̃p�[�e�B�N���ԍ�

double robXP = 0.0, robYP = 0.0;	//�ŒZ�ړ�����ۂ̎��̃��{�b�g�̗\���ʒu[m/s]
double robRotP = 0.0;				//�ŒZ�ړ�����ۂ̎��̃��{�b�g�̗\���p�x[rad]
double robRotMax = 0.0;				//�P�ʃ^�C���X�e�b�v�ɂ����郍�{�b�g�̍ő����p�x[rad]
double robRotC = 0.0;				//�P�ʃ^�C���X�e�b�v�ɂ�����ŒZ�ړ��̍ۂ̃��{�b�g�̐���p�x[rad]


int frag = 0;

int q[N][N][N][N][N][N][N];				        //���S���]��yes/no=0/1
int r;							//����O����������no/yes=0/1
int s;                          //�]���l�o�͗p
int safe = 0;						//���S�]��yes/no=0/1

int st = 1;						//��~�]��yes/no=0/1

int SELECT[7];					//�I�����ꂽ���[�g�ԍ�

int routing();
int safety();
int humanMoving();

int main() {
	printf("main start\n");

	//�����̏�����
	srand(time(NULL));

	//���{�b�g�̌��݈ʒu�𑪒�
	robX = 2.5;
	robY = 0.0;
	robRot = PI / 2;
	robRotMax = robRadMax * ts;

	//�l�Ԃ̌��݈ʒu�𑪒�
	humanX = 5.0;
	humanY = 2.5;
	humanTheta = PI * 2 / 2;
	miuV = humanV;

	colliC = humanC + robC;

	//�\����Ԃ̐���
	double robPAcc = 1.4;				//�\����Ԃ̐�̔��a[m/s^2]
	double robPRotAcc = 2.09;			//�\����Ԃ̐�p[rad/s^2]

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

		//���{�b�g���댯����̂��߂̒�~������s���Ă��邩
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
				//���{�b�g���l�Ԃ���\���ɗ���Ă���ꍇ�C�ڕW�ʒu�ɒ��i
				printf("robot moving straight\n");
				//robRot = atan((goalY - robY) / (goalX - robX));
				robRotP = atan2(goalY - robY, goalX - robX);
				robRotC = robRotP - robRotP;
				//�P�ʃ^�C���X�e�b�v�ɂ�����ő����p�x�𒴉߂����ꍇ�C�ő����p�x�Ɋۂ߂�
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
				//�l�Ԃ��ڋ߂��Ă���ꍇ�C�p�[�e�B�N���T�����J�n


				//�����x���l����������O������
				printf("robot searching route\n");
				double minGD = 0;
				int G = 0;
				int p = 0;


				//�p�[�e�B�N����
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

					//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
					//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

							//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
							//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

									//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
									//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

											//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
											//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

													//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
													//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

															//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
															//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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

																	//�ő�p���x�𒴉߂����ꍇ�C�ő�p���x�Ɋۂ߂�
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
																	//�ő呬�x�𒴉߂����ꍇ�C�ő呬�x�Ɋۂ߂�
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





				// �ڕW�ʒu�ɍł��߂��p�[�e�B�N���̔ԍ����擾
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

				// �ڕW�ʒu�ɍł��߂��p�[�e�B�N�����猻�݈ʒu�܂ł̃p�[�e�B�N�����W���擾

				p = G;

				if (p == 0) {
					//���S�Ȍo�H�����݂��Ȃ��̂Œ�~����
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
					//���S�Ȍo�H�̈���ڂ��ړ�
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
											//�z��̏�����
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


	//����_�ɂ������Q�̍����̌v�Z
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

	//�Փ˃��X�N�̌v�Z
	R = P * S;
	//printf("risk num=%0.10lf\n", R);

	//���S���̔���
	if (R > 0.00000011 * nts) {
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
	//����_�ɂ������Q�̍����̌v�Z
	robVX = xV * cos(robRot);
	robVY = xV * sin(robRot);
	relVX = robVX * cos(robThetaP) - humanVP * cos(humanThetaP);
	relVY = robVY * sin(robThetaP) - humanVP * sin(humanThetaP);
	rV = sqrt(pow(relVX, 2) + pow(relVY, 2));
	S = humanW * robW * pow(rV, 2) / (humanW + robW) / 2;
	//printf("S=%0.10lf\n", S);

	//�Փ˃��X�N�̌v�Z
	R = P * S;
	//printf("risk num=%0.10lf\n", R);
	*/

	//����O�������J�n�̔���
	if (P > 0.000001) {
		return 1;
	}
	else {
		return 0;
	}
}

int humanMoving() {
	//�l�Ԃ̃^�C���X�e�b�v���̈ړ�������
	//humanX = humanX + humanV * ts * cos((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	//humanY = humanY + humanV * ts * sin((double)rand() / 32767.0 * PI / 16 - PI / 32 + humanTheta);
	humanX = humanX - humanV * ts;
	//humanY = humanY - humanV * ts;

	/*//�����]��45��
	if (humanX > 2.5) {
	humanX = humanX - humanV * ts;
	}
	else {
	humanTheta = PI * 1 / 4;
	humanX = humanX + humanV * ts;
	humanY = humanY + humanV * ts;
	}*/

	/*//�����]��180��
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