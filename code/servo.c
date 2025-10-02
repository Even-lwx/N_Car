#include "servo.h"
#include "zf_common_headfile.h"



// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

float servo_motor_duty = 90.0; // 舵机当前角度

// 舵机初始化函数
void servo_init(void)
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_DUTY(90)); // 默认中位
    servo_motor_duty = 90.0f;
}

// 设置舵机角度（角度范围自动裁剪）
void servo_set_angle(float angle)
{
    // 限制角度范围
    if (angle < SERVO_MOTOR_L_MAX)
        angle = SERVO_MOTOR_L_MAX;
    if (angle > SERVO_MOTOR_R_MAX)
        angle = SERVO_MOTOR_R_MAX;
    servo_motor_duty = angle;
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(angle));
}

// 获取当前舵机角度
float servo_get_angle(void)
{
    return servo_motor_duty;
}