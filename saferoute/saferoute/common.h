//�A���S���Y���ɗp������萔

#define PI 3.14159265359

//�l�Ԃ̃p�����[�^
double humanC = 0.2;		//�l�Ԃ̔��a�i��������Z�o�j[m]
double humanW = 4.4;		//�l�Ԃ̏Փ˕��ʂ̎��ʁi�����j[kg]
double humanVMax = 3.6;		//�l�Ԃ̍ő呬�x[m/s]
double sigV = 0.4;			//�l�Ԃ̈ړ����x�̕W���΍�
double sigTheta = 0.0;		//�l�Ԃ̐i�s�����̕W���΍�

//���{�b�g�̃p�����[�^
double robC = 0.35;			//���{�b�g�̔��a[m]
double robW = 15;			//���{�b�g�̎���[kg]
double robVMax = 0.7;		//���{�b�g�̍ő呬�x[m/s]
double robRadMax = 6.28;	//���{�b�g�̍ő���񑬓x[rad/s]

//�ΐl����O���v��̃p�����[�^
double ts = 0.5;			//�P�ʃ^�C���X�e�b�v����[s]
int nts = 6;				//�^�C���X�e�b�v��
const int N = 15 + 1;		//�^�C���X�e�b�v���ɐ����������O���o�R�_���̐��i0�`15�j

//�z�u�Ɋւ���p�����[�^
double areaX = 4.0, areaY = 4.0;		//���{�b�g�̈ړ��ł���͈�[m]
double robXInit = 2.0, robYInit = 4.0;	//���{�b�g�̏����ʒu[m]
double robRotInit = PI / 2;				//���{�b�g�̏�������[rad]
double goalX = 2.0, goalY = 4.0;		//���{�b�g�̖ړI�ʒu[m]
double goalC = 0.35;					//���{�b�g���ړI�ʒu�ɐڐG�����Ɣ��肷�鋗���i��robC�j[m]