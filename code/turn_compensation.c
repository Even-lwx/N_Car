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
float turn_comp_k_servo = 0.1f;   // 舵机补偿系数（补偿重心偏移，默认0.1，建议范围: 0.01 ~ 1.0）
float turn_comp_k_speed = 0.01f;  // 速度补偿系数（补偿离心力，默认0.01，建议范围: 0.001 ~ 0.1）
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
// 函数简介     计算转弯补偿角度（分离式补偿算法）
// 参数说明     servo_angle: 当前舵机角度（度）
//              speed: 当前速度（编码器反馈值）
// 返回参数     float: 补偿角度（度）
// 使用示例     float compensation = turn_compensation_calculate(servo_angle, encoder[1]);
// 备注信息     物理意义：
//              1. 舵机补偿：舵机偏转导致的静态重心偏移（与速度无关）
//              2. 速度补偿：转向产生的离心力（只与速度和转向方向有关）
//              算法优势：分离设计避免了舵机偏差误差在高速时被放大
//              补偿公式：
//              - 舵机补偿 = K_servo × servo_deviation
//              - 速度补偿 = K_speed × speed² × sign(servo_deviation)
//              - 总补偿 = 舵机补偿 + 速度补偿
//              正值表示需要向左倾斜补偿，负值表示向右倾斜补偿
//-------------------------------------------------------------------------------------------------------------------
float turn_compensation_calculate(float servo_angle, float speed)
{
    // 1. 计算舵机偏离中点的角度
    float servo_deviation = servo_angle - servo_center_angle;

    // 2. 舵机补偿：只补偿舵机偏转导致的静态重心偏移
    //    即使速度为0，舵机偏转也会造成重心偏移
    float servo_compensation = turn_comp_k_servo * servo_deviation;

    // 3. 速度补偿：只补偿高速转向时的离心力
    //    关键：只取转向方向（正负），不取舵机偏差的具体数值
    //    这样避免了舵机偏差误差在高速时被放大
    float turn_direction = 0.0f;
    if (servo_deviation > 0.5f)        // 向右转
        turn_direction = 1.0f;
    else if (servo_deviation < -0.5f)  // 向左转
        turn_direction = -1.0f;
    // else 直行时 turn_direction = 0

    // 速度补偿：与速度平方成正比（模拟离心力）
    float speed_compensation = turn_comp_k_speed * fabs(speed) * fabs(speed) * turn_direction;

    // 4. 总补偿 = 舵机补偿 + 速度补偿
    //    - 舵机补偿：补偿静态重心偏移（低速高速都存在）
    //    - 速度补偿：补偿动态离心力（只在高速时显著）
    float total_compensation = servo_compensation + speed_compensation;

    // 5. 限幅处理：防止过度补偿
    if (total_compensation > turn_comp_max)
    {
        total_compensation = turn_comp_max;
    }
    else if (total_compensation < -turn_comp_max)
    {
        total_compensation = -turn_comp_max;
    }

    // 保存当前补偿值（用于调试显示）
    current_compensation = total_compensation;

    return total_compensation;
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
