
#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_headfile.h"

// *************************** 电机控制相关宏定义 ***************************
#define MAX_PWM_VALUE (10000) // 最大PWM数值输出限制
#define MAX_DUTY (30)         // 最大占空比输出限制（兼容性保留）

// 动量轮电机相关定义
#define MOMENTUM_WHEEL_DIR (P02_4)                           // 动量轮电机方向输出端口
#define MOMENTUM_WHEEL_PWM (ATOM0_CH5_P02_5)                 // 动量轮电机 PWM输出端口
#define MOMENTUM_WHEEL_ENCODER_TIM (TIM2_ENCODER)            // 动量轮编码器定时器
#define MOMENTUM_WHEEL_ENCODER_PLUS (TIM2_ENCODER_CH1_P33_7) // 动量轮编码器计数端口
#define MOMENTUM_WHEEL_ENCODER_DIR (TIM2_ENCODER_CH2_P33_6)  // 动量轮编码器方向采值端口

// 行进轮电机相关定义
#define DRIVE_WHEEL_DIR (P02_6)                           // 行进轮电机方向输出端口
#define DRIVE_WHEEL_PWM (ATOM0_CH7_P02_7)                 // 行进轮电机 PWM输出端口
#define DRIVE_WHEEL_ENCODER_TIM (TIM5_ENCODER)            // 行进轮编码器定时器
#define DRIVE_WHEEL_ENCODER_PLUS (TIM5_ENCODER_CH1_P10_3) // 行进轮编码器计数端口
#define DRIVE_WHEEL_ENCODER_DIR (TIM5_ENCODER_CH2_P10_1)  // 行进轮编码器方向采值端口

// 兼容性定义（保持向后兼容）
#define DIR_CH1 MOMENTUM_WHEEL_DIR
#define PWM_CH1 MOMENTUM_WHEEL_PWM
#define DIR_CH2 DRIVE_WHEEL_DIR
#define PWM_CH2 DRIVE_WHEEL_PWM
#define ENCODER1_TIM MOMENTUM_WHEEL_ENCODER_TIM
#define ENCODER1_PLUS MOMENTUM_WHEEL_ENCODER_PLUS
#define ENCODER1_DIR MOMENTUM_WHEEL_ENCODER_DIR
#define ENCODER2_TIM DRIVE_WHEEL_ENCODER_TIM
#define ENCODER2_PLUS DRIVE_WHEEL_ENCODER_PLUS
#define ENCODER2_DIR DRIVE_WHEEL_ENCODER_DIR

// *************************** 全局变量声明 ***************************
extern int8 duty;        // 当前占空比
extern bool dir;         // 计数方向
extern int16 encoder[2]; // 编码器值

// *************************** 函数声明 ***************************
void motor_init(void);           // 电机初始化函数
void motor_encoder_update(void); // 编码器数据更新

// 新增的独立电机控制函数
void momentum_wheel_control(int16 pwm_value); // 动量轮电机控制函数 参数：带符号的PWM数值(-10000到0到0-10000)
void drive_wheel_control(int16 pwm_value);    // 行进轮电机控制函数 参数：带符号的PWM数值(-10000到0到0-10000)

#endif
