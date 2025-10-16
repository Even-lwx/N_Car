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

#include "voltage.h"
#include "zf_common_headfile.h"

// 滑动平均滤波缓冲区
#define VOLTAGE_FILTER_SIZE 20
static uint16 adc_buffer[VOLTAGE_FILTER_SIZE] = {0};
static uint8 buffer_index = 0;
static uint8 sample_count = 0; // 已采集的样本数

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电压采样初始化
// 参数说明     void
// 返回参数     void
// 使用示例     voltage_init();
// 备注信息     建议电池检测用 ADC2
//-------------------------------------------------------------------------------------------------------------------
void voltage_init(void)
{
    adc_init(VOLTAGE_PORT, VOLTAGE_ADC_BITS);

    // 重置滤波器状态
    buffer_index = 0;
    sample_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取滤波后的ADC值
// 参数说明     void
// 返回参数     uint16      滤波后的ADC值 (0-4095)
// 使用示例     uint16 adc_value = voltage_get_adc_filtered();
// 备注信息     使用滑动平均滤波，减少ADC波动
//-------------------------------------------------------------------------------------------------------------------
uint16 voltage_get_adc_filtered(void)
{
    // 读取新的ADC值
    uint16 new_adc = adc_convert(VOLTAGE_PORT);

    // 更新缓冲区
    adc_buffer[buffer_index] = new_adc;
    buffer_index = (buffer_index + 1) % VOLTAGE_FILTER_SIZE;

    // 增加采样计数（最大到VOLTAGE_FILTER_SIZE）
    if (sample_count < VOLTAGE_FILTER_SIZE)
    {
        sample_count++;
    }

    // 如果缓冲区未填满，直接返回当前ADC值（不滤波）
    if (sample_count < VOLTAGE_FILTER_SIZE)
    {
        return new_adc;
    }

    // 缓冲区已填满，计算滑动平均值
    uint32 sum = 0;
    for (uint8 i = 0; i < VOLTAGE_FILTER_SIZE; i++)
    {
        sum += adc_buffer[i];
    }

    return (uint16)(sum / VOLTAGE_FILTER_SIZE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取ADC原始值
// 参数说明     void
// 返回参数     uint16      ADC原始值 (0-4095)
// 使用示例     uint16 adc_value = voltage_get_adc();
// 备注信息     12位ADC的原始采样值，已应用滑动平均滤波
//-------------------------------------------------------------------------------------------------------------------
uint16 voltage_get_adc(void)
{
    return voltage_get_adc_filtered();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电压值
// 参数说明     void
// 返回参数     float       电压值 (单位: V)
// 使用示例     float voltage = voltage_get_value();
// 备注信息     根据逐飞科技学习主板分压电路计算
//              分压比为 1:4, 最大量程 36.3V
//              由于电阻误差和ADC误差, 测量值可能与实际电压有偏差
//              使用滑动平均滤波后的ADC值进行计算
//-------------------------------------------------------------------------------------------------------------------
float voltage_get_value(void)
{
    uint16 adc_value = voltage_get_adc_filtered();
    float voltage = VOLTAGE_MAX_RANGE * adc_value / VOLTAGE_ADC_MAX_VALUE;
    return voltage;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取校准后的电压值
// 参数说明     calibration_factor      校准系数
// 返回参数     float                   校准后的电压值 (单位: V)
// 使用示例     float voltage = voltage_get_calibrated(1.05);
// 备注信息     如果测量值与实际电压有偏差, 可以通过校准系数进行矫正
//              校准系数 = 实际电压 / 测量电压
//              例如: 实际电压12.0V, 测量值11.5V, 则校准系数 = 12.0/11.5 = 1.043
//-------------------------------------------------------------------------------------------------------------------
float voltage_get_calibrated(float calibration_factor)
{
    return voltage_get_value() * calibration_factor;
}
