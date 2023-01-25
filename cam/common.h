#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

/* --- �萔�ݒ� --- */

#define PI 3.14159265359	        //�~����[rad]

/* --- �l�Ԃ̃p�����[�^ --- */
#define HUMAN_RADIUS        0.2 //�l�Ԃ̔��a�i��������Z�o�j[m]
#define HUMAN_WEIGHT        4.4 //�l�Ԃ̏Փ˕��ʂ̎��ʁi�����j[kg]
#define HUMAN_MAX_VELOCITY  3.6 //�l�Ԃ̍ő呬�x[m/s]
#define SGIMA_VELOCITY      0.4 //�l�Ԃ̈ړ����x�̕W���΍�
#define SIGMA_THETA         0.0 //�l�Ԃ̐i�s�����̕W���΍�

/* --- ���{�b�g�̃p�����[�^ --- */
#define ROBOT_RADIUS        0.35 //���{�b�g�̔��a[m]
#define ROBOT_WEIGHT        15	 //���{�b�g�̎���[kg]
#define ROBOT_MAX_VELOCITY  0.7  //���{�b�g�̍ő呬�x[m/s]
#define ROBOT_MAX_RAD       6.28 //���{�b�g�̍ő���񑬓x[rad/s]

/* --- �ΐl����O���v��̃p�����[�^ --- */
#define TIMESTEP            0.5			//�P�ʃ^�C���X�e�b�v����[s]
#define TIMESTEP_NUM        6				//�^�C���X�e�b�v��
#define EVERY_TIMESTEP_NUM  15 + 1		//�^�C���X�e�b�v���ɐ����������O���o�R�_���̐��i0�`15�j

/* --- �z�u�Ɋւ���p�����[�^ --- */
// ���{�b�g�ړ��̈�͈�
#define AREA_X          4.0     // �G���AX [m]
#define AREA_Y          4.0     // �G���AY [m]
#define ROBOT_X_INIT    2.0     // �����ʒuX [m]
#define ROBOT_Y_INIT    4.0     // �����ʒuY [m]
#define ROBOT_RAD_INIT  PI / 2  // �������� [rad]
#define GOAL_X          2.0     // �ړI�ʒuX [m]
#define GOAL_Y          4.0     // �ړI�ʒuY [m]
#define GOAL_DADIUS     0.35    // �ړI�ʒu�ł̐ڐG���苗�� [m]

#endif
