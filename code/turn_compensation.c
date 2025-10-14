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


// *************************** 全局变量定义 ***************************
float turn_comp_k_servo = 0.7f;            // 死区阈值（补偿绝对值小于此值时视为0，默认0.7度）
float turn_comp_k_speed = 1.0f;            // 动态补偿增益（默认1.0，建议范围: 0.1 ~ 10.0）
float turn_comp_max = 8.0f;                // 最大补偿角度限制（±8度）
float servo_center_angle = 90.0f;          // 舵机中点角度（默认90度）
float turn_comp_image_threshold = 5.0f;    // 图像误差阈值（误差小于此值时不补偿，默认5.0）

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
//              image_error: 图像中线偏差（用于判断是否需要补偿）
// 返回参数     float: 补偿角度（度）
// 使用示例     float compensation = turn_compensation_calculate(servo_angle, encoder[1], image_error);
// 备注信息     补偿公式：
//              dynamic_zero = (servo_angle - servo_center) × speed × gain / 10
//              然后限幅到 [-turn_comp_max, +turn_comp_max]
//              如果 |image_error| < image_error_threshold，则补偿为0（直行不补偿）
//              如果补偿绝对值 < deadzone，则补偿为0（死区处理）
//              正值表示需要向左倾斜补偿，负值表示向右倾斜补偿
//-------------------------------------------------------------------------------------------------------------------
float turn_compensation_calculate(float servo_angle, float speed, float image_error)
{
    // 1. 图像误差检查：如果图像误差很小（接近直行），不需要补偿
    if (fabsf(image_error) < turn_comp_image_threshold)
    {
        current_compensation = 0.0f;
        return 0.0f;
    }

    // 2. 计算动态零点补偿：(舵机偏差) × 速度绝对值 × 增益 / 10
    float servo_deviation = servo_angle - servo_center_angle;
    float dynamic_zero = servo_deviation * (float)fabs(speed) * turn_comp_k_speed / 10.0f;

    // 3. 限幅处理：防止过度补偿
    if (dynamic_zero > turn_comp_max)
    {
        dynamic_zero = turn_comp_max;
    }
    else if (dynamic_zero < -turn_comp_max)
    {
        dynamic_zero = -turn_comp_max;
    }

    // 4. 死区处理：小于阈值的补偿视为0（避免微小抖动）
    if (fabsf(dynamic_zero) < turn_comp_k_servo)
    {
        dynamic_zero = 0.0f;
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
