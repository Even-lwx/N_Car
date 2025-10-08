/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 文件名称          pid.h - 动量轮三环PID控制系统（简化版）
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 ********************************************************************************************************************/

#ifndef PID_H
#define PID_H

#include "zf_common_headfile.h"
#include "motor.h"
#include "imu.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// PID控制器结构体
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累积
    float derivative;   // 微分项
    float output;       // 输出值
    float max_integral; // 积分限幅
    float max_output;   // 输出限幅
} PID_Controller;

// 全局变量声明
extern PID_Controller gyro_pid;        // 角速度环PID
extern PID_Controller angle_pid;       // 角度环PID
extern PID_Controller speed_pid;       // 速度环PID
extern PID_Controller drive_speed_pid; // 行进轮速度环PID

extern float target_angle;       // 目标角度
extern float target_speed;       // 目标速度
extern float target_gyro_rate;   // 目标角速度
extern float target_drive_speed; // 行进轮目标速度

extern float filtered_angle;        // 滤波后的角度
extern float filtered_speed;        // 滤波后的速度
extern float filtered_gyro;         // 滤波后的角速度
extern float filtered_motor_output; // 滤波后的电机输出

// 控制标志
extern volatile bool enable; // 使能标志（volatile防止编译器优化）

// 控制变量
extern float angle_offset;
extern uint32_t control_counter;

// 控制优化参数
extern float angle_deadzone;   // 角度死区
extern float angle_protection; // 角度保护阈值
extern float angle_gain_scale; // 角度环死区增益缩放
extern float gyro_gain_scale;  // 角速度环死区增益缩放

// 输出平滑参数（导出到菜单）
extern float output_filter_coeff; // 输出滤波系数

// 函数声明
void pid_init(void);
void pid_reset(void);
float pid_calculate(PID_Controller *pid, float target, float current);

void data_acquisition(void);
void gyro_loop_control(int angle_control);
void angle_loop_control(int speed_control);
void speed_loop_control(void);
void control(void);

void set_target_speed(float speed);
void set_target_angle(float angle);
void set_target_drive_speed(float speed);
void set_gyro_pid_params(float kp, float ki, float kd);
void set_angle_pid_params(float kp, float ki, float kd);
void set_speed_pid_params(float kp, float ki, float kd);
void set_drive_speed_pid_params(float kp, float ki, float kd);

// 输出滤波器参数设置函数
void set_output_filter_coeff(float coeff);

float get_encoder_speed(void);
void get_pid_status(float *gyro_error, float *angle_error, float *speed_error,
                    float *gyro_output, float *angle_output, float *speed_output);
void drive_speed_loop_control(void);

// 控制优化参数设置
void set_angle_deadzone(float deadzone);
void set_angle_protection(float protection);
float get_angle_deadzone(void);
float get_angle_protection(void);

#endif
