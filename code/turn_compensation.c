/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 文件名称          turn_compensation.c - 转弯补偿控制器实现
 * 功能说明          补偿转弯时由于离心力和舵机角度导致的机械零点漂移
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 ********************************************************************************************************************/

#include "turn_compensation.h"
#include "zf_common_headfile.h"
#include <math.h>

// *************************** 全局变量定义 ***************************
float turn_comp_k_angle = 0.1f;   // 舵机角度增益系数（默认0.1，建议范围: 0.01 ~ 1.0）
float turn_comp_k_speed = 0.01f;  // 速度增益系数（默认0.01，建议范围: 0.001 ~ 0.1）
float turn_comp_max = 8.0f;       // 最大补偿角度限制（±8度）
float servo_center_angle = 90.0f; // 舵机中点角度（默认90度）

// *************************** 内部变量 ***************************
static float current_compensation = 0.0f; // 当前补偿值（用于调试）

// *************************** 函数实现 ***************************

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化转弯补偿控制器
// 参数说明     void
// 返回参数     void
// 使用示例     turn_compensation_init();
// 备注信息     在系统初始化时调用，设置默认参数
//-------------------------------------------------------------------------------------------------------------------
void turn_compensation_init(void)
{
    // 参数已在全局变量定义时初始化
    // 这里可以添加额外的初始化逻辑（如果需要）
    current_compensation = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算转弯补偿角度（动态零点补偿算法）
// 参数说明     servo_angle: 当前舵机角度（度）
//              speed: 当前速度（编码器反馈值）
// 返回参数     float: 补偿角度（度）
// 使用示例     float compensation = turn_compensation_calculate(servo_angle, encoder[1]);
// 备注信息     物理意义：车辆转向时需要补偿两部分：
//              1. 基础补偿：舵机偏转导致的静态重心偏移（与速度无关，速度为0时也存在）
//              2. 动态补偿：转向产生的离心力（与速度平方成正比）
//              补偿公式：dynamic_zero = K_angle * servo_deviation + K_speed * speed^2 * servo_deviation
//              正值表示需要向左倾斜补偿，负值表示向右倾斜补偿
//-------------------------------------------------------------------------------------------------------------------
float turn_compensation_calculate(float servo_angle, float speed)
{
    // 1. 计算舵机偏离中点的角度
    float servo_deviation = servo_angle - servo_center_angle;

    // 2. 基础补偿：基于舵机角度（即使速度为0也需要补偿重心偏移）
    float base_compensation = turn_comp_k_angle * servo_deviation;

    // 3. 动态补偿：速度越快，离心力越大，需要额外补偿
    //    速度平方项：模拟离心力与速度平方成正比的关系
    float dynamic_compensation = turn_comp_k_speed * fabs(speed) * fabs(speed) * servo_deviation;

    // 4. 总补偿 = 基础补偿 + 动态补偿
    //    - 基础补偿：补偿静态重心偏移（速度为0时仍存在）
    //    - 动态补偿：补偿离心力影响（速度越快影响越大）
    float dynamic_zero = base_compensation + dynamic_compensation;

    // 5. 限幅处理：防止过度补偿
    if (dynamic_zero > turn_comp_max)
    {
        dynamic_zero = turn_comp_max;
    }
    else if (dynamic_zero < -turn_comp_max)
    {
        dynamic_zero = -turn_comp_max;
    }

    // 保存当前补偿值（用于调试显示）
    current_compensation = dynamic_zero;

    return dynamic_zero;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前补偿值（用于调试显示）
// 参数说明     void
// 返回参数     float: 当前补偿角度（度）
// 使用示例     float comp = turn_compensation_get_current();
// 备注信息     返回上次计算的补偿值
//-------------------------------------------------------------------------------------------------------------------
float turn_compensation_get_current(void)
{
    return current_compensation;
}
