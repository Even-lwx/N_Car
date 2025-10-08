/*
 * QuaternionEKF.h
 *
 *  Created on: 2024��3��9��
 *      Author: huawei
 */

#ifndef CODE_QUATERNIONEKF_H_
#define CODE_QUATERNIONEKF_H_


#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif





typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // ��Ԫ������ֵ
    float GyroBias[3]; // ��������ƫ����ֵ

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // ��Ԫ�����¹�������
    float Q2; // ��������ƫ��������
    float R;  // ���ٶȼ���������

    float dt; // ��̬��������
    mat ChiSquare;
    float ChiSquare_Data[1];      // ���������⺯��
    float ChiSquareTestThreshold; // ����������ֵ
    float lambda;                 // ��������

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);//get pitch
float Get_Roll(void);//get roll
float Get_Yaw(void);//get yaw



#endif /* CODE_QUATERNIONEKF_H_ */
