
#ifndef _SERVO_H_
#define _SERVO_H_

#define SERVO_MOTOR_PWM (ATOM1_CH1_P33_9) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)             // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX (50)  // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX (150) // 定义主板上舵机活动范围 角度

// 舵机初始化
void servo_init(void);
// 设置舵机角度
void servo_set_angle(float angle);
// 获取当前舵机角度
float servo_get_angle(void);

#endif
