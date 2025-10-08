/*
 * EKF_Platform.h
 *
 *  Created on: 2024��3��9��
 *      Author: huawei
 */

#ifndef CODE_EKF_PLATFORM_H_
#define CODE_EKF_PLATFORM_H_

#include "Ifx_LutLSincosF32.h"
#include "Ifx_LutAtan2F32.h"
#include "matrix.h"   //armƽ̨�Ĳ���Ҫ���� ֱ��ʹ��dsp���
//�������Բ�ͬƽ̨�����Ǻ��� �������Ӣ����� ��������Լ�����

#define arm_cos_f32 Ifx_LutLSincosF32_cos  //Ӣ�����cos
#define arm_atan2_f32 Ifx_LutAtan2F32_float32 //Ӣ����ķ�atan2

//================================

//=====================================================================
//�����Ǿ���⣬�����arm�ں˵ľͿ��Բ�������matrix�⣬��������Լ�ʵ�ֵ�
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32




#endif /* CODE_EKF_PLATFORM_H_ */
