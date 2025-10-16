/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有,未经允许不得用于商业用途,
 * 欢迎各位使用并传播本程序,修改内容时必须保留逐飞科技的版权声明。
 *
 * @file             voltage
 * @company          成都逐飞科技有限公司
 * @author           N-Car Project
 * @version          查看 libraries/doc 文件夹内 version 文件 版本说明
 * @Software         AURIX Development Studio
 * @Target core      TC264D
 * @date             2025
 ********************************************************************************************************************/

#ifndef _VOLTAGE_H_
#define _VOLTAGE_H_

#include "zf_common_typedef.h"

// 电压采样端口定义
#define VOLTAGE_PORT ADC0_CH11_A11

// ADC分辨率配置
#define VOLTAGE_ADC_BITS ADC_12BIT
#define VOLTAGE_ADC_MAX_VALUE 4096 // 12位ADC最大值

// 电压计算参数 (根据逐飞科技学习主板分压电路)
// 分压比为 1:4, 最大量程 36.3V
#define VOLTAGE_DIVIDER_RATIO 4.0f
#define VOLTAGE_MAX_RANGE 36.3f

extern float voltage_calibration_factor; // 校准系数

// 函数接口
void voltage_init(void);
uint16 voltage_get_adc(void);
float voltage_get_value(void);
uint16 voltage_get_adc_filtered(void);
float voltage_get_calibrated(float calibration_factor);

#endif // _VOLTAGE_H_
