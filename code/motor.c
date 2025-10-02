// *************************** 头文件包含 ***************************
#include "motor.h"
#include "zf_common_headfile.h"
// *************************** 全局变量定义 ***************************
int8 duty = 0;          // 当前占空比
bool dir = true;        // 计数方向
int16 encoder[2] = {0}; // 编码器值

// *************************** 函数实现 ***************************

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     motor_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // PWM和方向控制引脚初始化
    pwm_init(PWM_CH1, 1000, 0);                                  // PWM 通道1 初始化频率1KHz 占空比初始为0
    gpio_init(DIR_CH1, GPO, GPIO_HIGH, GPO_PUSH_PULL);           // 初始化电机1方向输出引脚
    encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR); // 初始化编码器1采值引脚及定时器

    pwm_init(PWM_CH2, 1000, 0);                                  // PWM 通道2 初始化频率1KHz 占空比初始为0
    gpio_init(DIR_CH2, GPO, GPIO_HIGH, GPO_PUSH_PULL);           // 初始化电机2方向输出引脚
    encoder_dir_init(ENCODER2_TIM, ENCODER2_PLUS, ENCODER2_DIR); // 初始化编码器2采值引脚及定时器
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     动量轮电机控制函数
// 参数说明     pwm_value: 带符号的PWM数值，范围-10000到10000，正数正转，负数反转，0停止
// 返回参数     void
// 使用示例     momentum_wheel_control(5000);   // 向右转，PWM数值5000
//              momentum_wheel_control(-3000);  // 向左转，PWM数值3000
//              momentum_wheel_control(0);      // 停止
// 备注信息     控制动量轮电机的转速和方向，直接传入PWM数值
//-------------------------------------------------------------------------------------------------------------------
void momentum_wheel_control(int16 pwm_value)
{
    // 限制PWM数值范围
    if (pwm_value > MAX_PWM_VALUE)
        pwm_value = MAX_PWM_VALUE;
    if (pwm_value < -MAX_PWM_VALUE)
        pwm_value = -MAX_PWM_VALUE;

    if (pwm_value >= 0) // 正转或停止
    {
        pwm_set_duty(MOMENTUM_WHEEL_PWM, pwm_value); // 直接设置PWM数值
        gpio_set_level(MOMENTUM_WHEEL_DIR, 1);       // 设置方向为正转
    }
    else // 反转
    {
        pwm_set_duty(MOMENTUM_WHEEL_PWM, -pwm_value); // 设置PWM数值（取绝对值）
        gpio_set_level(MOMENTUM_WHEEL_DIR, 0);        // 设置方向为反转
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     行进轮电机控制函数
// 参数说明     pwm_value: 带符号的PWM数值，范围-10000到10000，正数正转，负数反转，0停止
// 返回参数     void
// 使用示例     drive_wheel_control(7500);   // 正转，PWM数值7500
//              drive_wheel_control(-2000);  // 反转，PWM数值2000
//              drive_wheel_control(0);      // 停止
// 备注信息     控制行进轮电机的转速和方向，直接传入PWM数值
//-------------------------------------------------------------------------------------------------------------------
void drive_wheel_control(int16 pwm_value)
{
    // 限制PWM数值范围
    if (pwm_value > MAX_PWM_VALUE)
        pwm_value = MAX_PWM_VALUE;
    if (pwm_value < -MAX_PWM_VALUE)
        pwm_value = -MAX_PWM_VALUE;

    if (pwm_value >= 0) // 正转或停止
    {
        pwm_set_duty(DRIVE_WHEEL_PWM, pwm_value); // 直接设置PWM数值
        gpio_set_level(DRIVE_WHEEL_DIR, 1);       // 设置方向为正转
    }
    else // 反转
    {
        pwm_set_duty(DRIVE_WHEEL_PWM, -pwm_value); // 设置PWM数值（取绝对值）
        gpio_set_level(DRIVE_WHEEL_DIR, 0);        // 设置方向为反转
    }
}

// 使用示例     motor_encoder_update();  // 在中断中调用
// 备注信息     该函数在定时器中断中被调用，用于读取和清除编码器数据
//-------------------------------------------------------------------------------------------------------------------
void motor_encoder_update(void)
{
    encoder[0] = encoder_get_count(ENCODER1_TIM); // 采集对应编码器数据
    encoder[1] = encoder_get_count(ENCODER2_TIM); // 采集对应编码器数据
    encoder_clear_count(ENCODER1_TIM);            // 清除对应计数
    encoder_clear_count(ENCODER2_TIM);            // 清除对应计数
}

//
