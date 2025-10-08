/*
 * EKF_Platform.h
 *
 *  Created on: 2024年3月9日
 *      Author: huawei
 */

#ifndef CODE_EKF_PLATFORM_H_
#define CODE_EKF_PLATFORM_H_

#include "Ifx_LutLSincosF32.h"
#include "Ifx_LutAtan2F32.h"
#include "matrix.h"   //arm平台的不需要引用 直接使用dsp库的
//这个是针对不同平台的三角函数 下面的是英飞凌的 其他大家自己补充

#define arm_cos_f32 Ifx_LutLSincosF32_cos  //英飞凌的cos
#define arm_atan2_f32 Ifx_LutAtan2F32_float32 //英飞凌的反atan2

//================================

//=====================================================================
//下面是矩阵库，如果是arm内核的就可以不用引用matrix库，这个是我自己实现的
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32




#endif /* CODE_EKF_PLATFORM_H_ */
