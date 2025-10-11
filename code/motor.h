
#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_headfile.h"

// *************************** 电机控制相关宏定义 ***************************
#define MAX_PWM_VALUE (10000) // 最大PWM数值输出限制
#define MAX_DUTY (30)         // 最大占空比输出限制（兼容性保留）

// 电机保护参数
#define STALL_DETECT_PWM_THRESHOLD (3000)  // 堵转检测PWM阈值（超过此值才检测堵转）
#define STALL_DETECT_ENCODER_THRESHOLD (2) // 堵转检测编码器阈值（低于此值认为堵转）
#define STALL_DETECT_TIME_MS (1000)        // 堵转检测时间（毫秒）
#define DIR_CHANGE_MIN_INTERVAL_MS (100)   // 方向切换最小间隔（毫秒）
#define PROTECTION_DISABLE_TIME_MS (2000)  // 保护触发后禁用电机时间（毫秒）

// 蜂鸣器引脚定义（根据实际硬件修改）
#define BUZZER_PIN (P33_10)           // 蜂鸣器引脚，请根据实际硬件修改
#define BUZZER_BEEP_COUNT (2)         // 保护触发时蜂鸣器鸣叫次数
#define BUZZER_BEEP_DURATION_MS (100) // 每次鸣叫持续时间（毫秒）
#define BUZZER_BEEP_INTERVAL_MS (100) // 鸣叫间隔时间（毫秒）

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
void motor_init(void);                    // 电机初始化函数
void motor_encoder_update(void);          // 编码器数据更新（同时更新两个编码器）
void motor_encoder_update_momentum(void); // 动量轮编码器数据更新（仅更新encoder[0]）
void motor_encoder_update_drive(void);    // 行进轮编码器数据更新（仅更新encoder[1]）
void motor_protection_update(void);       // 电机保护更新（需要定时调用，建议1ms周期）

// 新增的独立电机控制函数
void momentum_wheel_control(int16 pwm_value); // 动量轮电机控制函数 参数：带符号的PWM数值(-10000到0到0-10000)
void drive_wheel_control(int16 pwm_value);    // 行进轮电机控制函数 参数：带符号的PWM数值(-10000到0到0-10000)

// 电机保护相关函数
void motor_reset_protection(void);             // 重置电机保护状态（解除保护）
bool motor_is_stall_protected(uint8 motor_id); // 查询电机是否处于堵转保护状态 (0=动量轮, 1=行进轮)

#endif
