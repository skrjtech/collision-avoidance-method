#ifndef COMMON_HEADER_FILE
#define COMMON_HEADER_FILE

#include <math.h>

/* --- �萔�ݒ� --- */

// #define PI 3.14159265359	            // �~���� [rad]
#define DEGREE(x) ((x) * M_PI / 180)    // �p�x����x�ɕϊ�

/* --- �Z���T�[ �p�����[�^ --- */
#define SENSOR_MAX_DISTANCE 10000   // �ő勗�� [mm]
#define SENSOR_MAX_RADIAN   270     // �ő�p�x [rad]
#define SENSOR_MAX_HEIGHT   177.4   // �n�ʂ���̍ő卂�� [mm]

/* --- �̈�i�[�p�z�� �p�����[�^ --- */
#define ACCURACY_DISTANCE 10 // ���萸�x [mm]
// #define ARRAY_HEIGHT (SENSOR_MAX_DISTANCE / ACCURACY_DISTANCE) // ������
// #define ARRAY_WIDTH  (SENSOR_MAX_DISTANCE / ACCURACY_DISTANCE) // ����
#define ARRAY_HEIGHT 10000
#define ARRAY_WIDTH  10000

/* --- ����\�̈� �p�����[�^ --- */
#define AREA_MAX_HEIGHT     10000 // �ő卂�� [mm]
#define AREA_MAX_WIDTH      10000 // �ő啝 [mm]
#define AREA_X_MAX_RADIAN    (SENSOR_MAX_DISTANCE * cos(DEGREE(90))) // X���ő呪��̈�   
#define AREA_Y_MAX_RADIAN    (SENSOR_MAX_DISTANCE * sin(DEGREE(90))) // Y���ő呪��̈�

/* --- ���̔F���̈� �p�����[�^ --- */
#define HANTEI_MAX_HEIGHT     7000 // �ő卂�� [mm]
#define HANTEI_MAX_WIDTH      7000 // �ő啝 [mm]
#define HANTEI_X_MAX_RADIAN    (HANTEI_MAX_HEIGHT * cos(DEGREE(90))) // X���ő呪��̈�   
#define HANTEI_Y_MAX_RADIAN    (HANTEI_MAX_HEIGHT * sin(DEGREE(90))) // Y���ő呪��̈�

/* --- �l�Ԃ̃p�����[�^ --- */
#define HUMAN_RADIUS        0.2 //�l�Ԃ̔��a�i��������Z�o�j[m]
#define HUMAN_WEIGHT        4.4 //�l�Ԃ̏Փ˕��ʂ̎��ʁi�����j[kg]
#define HUMAN_MAX_VELOCITY  3.6 //�l�Ԃ̍ő呬�x[m/s]
#define SIGMA_VELOCITY      0.4 //�l�Ԃ̈ړ����x�̕W���΍�


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
#define ROBOT_Y_INIT    0.0     // �����ʒuY [m]
#define ROBOT_RAD_INIT  PI / 2  // �������� [rad]
#define GOAL_X          2.0     // �ړI�ʒuX [m]
#define GOAL_Y          4.0     // �ړI�ʒuY [m]
#define GOAL_DISTANCE     0.35    // �ړI�ʒu�ł̐ڐG���苗�� [m]

#endif