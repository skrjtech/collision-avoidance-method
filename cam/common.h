#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

/* --- �萔�ݒ� --- */

#define PI 3.14159265359	//�~����[rad]

/* --- �l�Ԃ̃p�����[�^ --- */
#define humanC = 0.2;		//�l�Ԃ̔��a�i��������Z�o�j[m]
#define humanW = 4.4;		//�l�Ԃ̏Փ˕��ʂ̎��ʁi�����j[kg]
#define humanVMax = 3.6;		//�l�Ԃ̍ő呬�x[m/s]
#define sigV = 0.4;			//�l�Ԃ̈ړ����x�̕W���΍�
#define sigTheta = 0.0;		//�l�Ԃ̐i�s�����̕W���΍�

/* --- ���{�b�g�̃p�����[�^ --- */
#define robC = 0.35;			//���{�b�g�̔��a[m]
#define robW = 15;			//���{�b�g�̎���[kg]
#define robVMax = 0.7;		//���{�b�g�̍ő呬�x[m/s]
#define robRadMax = 6.28;	//���{�b�g�̍ő���񑬓x[rad/s]

/* --- �ΐl����O���v��̃p�����[�^ --- */
#define ts = 0.5;			//�P�ʃ^�C���X�e�b�v����[s]
#define nts = 6;				//�^�C���X�e�b�v��
#define N = 15 + 1;		//�^�C���X�e�b�v���ɐ����������O���o�R�_���̐��i0�`15�j

/* --- �z�u�Ɋւ���p�����[�^ --- */
#define areaX = 4.0, areaY = 4.0;		//���{�b�g�̈ړ��ł���͈�[m]
#define robXInit = 2.0, robYInit = 4.0;	//���{�b�g�̏����ʒu[m]
#define robRotInit = PI / 2;				//���{�b�g�̏�������[rad]
#define goalX = 2.0, goalY = 4.0;		//���{�b�g�̖ړI�ʒu[m]
#define goalC = 0.35;					//���{�b�g���ړI�ʒu�ɐڐG�����Ɣ��肷�鋗���i��robC�j[m]

#endif
