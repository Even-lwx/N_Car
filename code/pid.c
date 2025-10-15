#include "pid.h"
#include "zf_common_headfile.h"

// *************************** 宏定义 ***************************

// *************************** 全局变量定义 ***************************

// 角速度环PID控制器
PID_Controller gyro_pid = {
    .kp = -7.0f,
    .ki = -15.4f,
    .kd = 0.0f,
    .max_integral = 100.0f,
    .max_output = 8000.0f};

// 角度环PID控制器
PID_Controller angle_pid = {
    .kp = -0.4f,
    .ki = 0.0f,
    .kd = -0.0f,
    .max_integral = 20.0f,
    .max_output = 3000.0f // 角度环输出作为角速度环的目标角速度(度/秒)
};

// 速度环PID控制器
PID_Controller speed_pid = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_integral = 100.0f,
    .max_output = 1000.0f // 速度环输出作为角度环的目标角度偏移(度)
};

// 行进轮速度环PID控制器
PID_Controller drive_speed_pid = {
    .kp = -1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_integral = 100.0f,
    .max_output = 10000.0f // 行进轮速度环输出PWM值
};

// 转向PID控制器（不使用标准PID结构，仅使用P和D）
PID_Controller steer_pid = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_integral = 0.0f,
    .max_output = 0.0f};

// 转向PID参数
float steer_kp = 0.5f;            // 转向P系数（基于图像偏差）
float steer_kd = 0.01f;           // 转向D系数（基于陀螺仪gz）
float steer_output_limit = 30.0f; // 转向输出限幅（舵机角度限制）
uint32 steer_sample_start = 50;   // 图像采样起始行（从底部算起，越大越近）
uint32 steer_sample_end = 60;     // 图像采样结束行
uint32 steer_enable = 1;          // 转向环使能（0=禁用，1=启用）

// 行进轮速度环控制参数
uint32 drive_speed_enable = 1;        // 行进轮速度环使能（0=开环，1=闭环PID）
float drive_open_loop_output = 0.0f;  // 行进轮开环输出值（PWM值，-10000~10000）

// 目标值
float target_gyro_rate = 0.0f;    // 目标角速度
float target_angle = 0.0f;        // 目标角度
float target_speed = 0.0f;        // 目标速度
float target_drive_speed = 10.0f; // 行进轮目标速度

// 控制标志
volatile bool enable = false; // 使能标志，默认禁用（需在Cargo模式中启用）

// 控制变量
static uint32_t count = 0;                // 控制计数器
static float desired_angle = 0.0f;        // 期望角度（速度环输出）
static float angle_gyro_target = 0.0f;    // 目标角速度（角度环输出）
static float current_servo_angle = 90.0f; // 当前实际舵机角度（由转向PID更新）
static float current_image_error = 0.0f;  // 当前图像误差（由转向PID更新，用于转弯补偿）

// 一阶低通滤波器相关变量（仅对PID输出滤波）
float output_filter_coeff = 0.6f;   // 输出滤波系数 (0-1)，降低以增强平滑度
float filtered_motor_output = 0.0f; // 滤波后的电机输出
float drive_pwm_output = 0.0f;      // 行进轮PWM输出值

// 控制优化参数（导出到菜单）
float angle_deadzone = 5.0f;    // 角度死区
float angle_protection = 15.0f; // 角度保护阈值

// 变增益PID参数
float angle_gain_scale = 1.0f; // 角度环死区增益缩放（0-1），越小越平滑
float gyro_gain_scale = 1.0f;  // 角速度环死区增益缩放（0-1）

// PID计算函数
float pid_calculate(PID_Controller *pid, float target, float current)
{
    // 计算误差
    pid->error = target - current;

    // 积分项计算（带积分限幅）
    pid->integral += pid->error;
    pid->integral = constrain(pid->integral, -pid->max_integral, pid->max_integral);

    // 微分项计算
    pid->derivative = pid->error - pid->last_error;

    // PID输出计算
    pid->output = pid->kp * pid->error +
                  pid->ki * pid->integral +
                  pid->kd * pid->derivative;

    // 输出限幅
    pid->output = constrain(pid->output, -pid->max_output, pid->max_output);

    // 保存当前误差供下次使用
    pid->last_error = pid->error;

    return pid->output;
}
/**
 * @brief 角速度环控制（最内环）
 * @param angle_control 角度环的输出，作为角速度环的目标值
 */
void gyro_loop_control(int angle_control)
{
    // 将角度环输出作为角速度目标值
    float target_gyro = (float)angle_control;

    // 使用IMU中已经滤波后的陀螺仪数据
    float current_gyro_y = (float)imu_data.gyro_y;

    // 角速度环PID计算（使用IMU中已滤波的陀螺仪数据）
    float motor_output = pid_calculate(&gyro_pid, target_gyro, current_gyro_y);

    // 对PID输出进行一阶低通滤波，减少输出抖动
    // filtered_value = α * current_value + (1-α) * previous_filtered_value
    filtered_motor_output = output_filter_coeff * motor_output + (1.0f - output_filter_coeff) * filtered_motor_output;

    // 控制电机（motor_control_with_protection内部会检查enable标志）
    momentum_wheel_control((int16_t)filtered_motor_output);
}

/**
 * @brief 角度环控制（中间环）
 * @param speed_control 速度环的输出，作为角度环的目标偏移（暂时不用，通过desired_angle传递）
 */
void angle_loop_control(int speed_control)
{
    // 计算转弯补偿角度（使用实际舵机角度和当前图像误差）
    float current_speed = (float)encoder[1];
    float turn_compensation = turn_compensation_calculate(current_servo_angle, current_speed, current_image_error);

    // 使用IMU中已经滤波后的pitch角度（IMU中已对原始数据进行滤波再解算）
    float current_pitch = imu_data.pitch;

    // 目标角度 = 期望角度（速度环输出） + 转弯补偿
    float target_angle_with_comp = desired_angle + turn_compensation;

    // 角度环PID计算
    angle_gyro_target = pid_calculate(&angle_pid, target_angle_with_comp, current_pitch);
}

/**
 * @brief 速度环控制
 */
void speed_loop_control(void)
{
    // 速度环PID计算，输出作为角度偏移
    float speed_angle_offset = pid_calculate(&speed_pid, target_speed, encoder[0]);

    // 更新期望角度
    desired_angle = target_angle + speed_angle_offset;
}

/**
 * @brief 主控制函数
 */
void control(void)
{
    count++;
    // 传感器数据更新
    if (count % 2 == 0)
    {
        imu_update();
    }
    // 动量轮速度环周期采集编码器数据（20ms周期）
    if (count % 20 == 0)
    {
        motor_encoder_update_momentum();
    }
    // 行进轮编码器采集（5ms周期，与转弯补偿同步）
    if (count % 5 == 0)
    {
        motor_encoder_update_drive();
    }
    // printf("%f,%d\r\n", imu_data.pitch, imu_data.gyro_y);
    //  如果未启用控制，停止电机并返回
    if (!enable)
    {
        momentum_wheel_control(0);
        drive_wheel_control(0);
        return;
    }

    // 延迟停车更新（只有enable为true时才计时）
    delayed_stop_update();

    motor_protection_update();

    // 角度在±200范围内时清空积分项，重新开始积分
    // if (imu_data.pitch >= -200.0f && imu_data.pitch <= 200.0f)
    // {
    //     gyro_pid.integral = 0.0f;
    //     angle_pid.integral = 0.0f;
    //     speed_pid.integral = 0.0f;
    //     drive_speed_pid.integral = 0.0f;
    // }

    // 速度环控制（20ms周期，每20个1ms周期执行一次）
    if (count % 20 == 0)
    {
        speed_loop_control();
    }

    // 角度环控制（5ms周期，每5个1ms周期执行一次，内部包含转弯补偿）
    if (count % 5 == 0)
    {
        angle_loop_control(0); // 速度环输出通过desired_angle传递
    }

    // 角速度环控制（2ms周期，每次都执行）
    if (count % 2 == 0)
    {
        gyro_loop_control((int)angle_gyro_target);
    }
    // 行进轮速度环控制（20ms周期，每20个1ms周期执行一次）
    if (count % 20 == 0)
    {

        drive_speed_loop_control();
    }

    // 转向PID控制在主循环中调用（需要最新的图像数据）
    // 不在中断中调用，避免阻塞中断和重复调用

    // 防止计数器溢出
    if (count >= 1000)
    {
        count = 0;
    }
}

/**
 * @brief PID系统初始化
 */
void pid_init(void)
{
    // 初始化所有PID控制器状态
    gyro_pid.error = 0;
    gyro_pid.last_error = 0;
    gyro_pid.integral = 0;
    gyro_pid.output = 0;

    angle_pid.error = 0;
    angle_pid.last_error = 0;
    angle_pid.integral = 0;
    angle_pid.output = 0;

    speed_pid.error = 0;
    speed_pid.last_error = 0;
    speed_pid.integral = 0;
    speed_pid.output = 0;

    drive_speed_pid.error = 0;
    drive_speed_pid.last_error = 0;
    drive_speed_pid.integral = 0;
    drive_speed_pid.output = 0;

    steer_pid.error = 0;
    steer_pid.last_error = 0;
    steer_pid.integral = 0;
    steer_pid.output = 0;

    // 初始化滤波器状态
    filtered_motor_output = 0.0f;
}

/**
 * @brief PID系统重置
 */
void pid_reset(void)
{
    pid_init();

    // 重置目标值
    target_angle = 0.0f;
    target_speed = 0.0f;
    target_gyro_rate = 0.0f;
    target_drive_speed = 0.0f;
}

/**
 * @brief 设置目标速度
 */
void set_target_speed(float speed)
{
    target_speed = speed;
}

/**
 * @brief 设置目标角度
 */
void set_target_angle(float angle)
{
    target_angle = angle;
}

/**
 * @brief 设置行进轮目标速度
 */
void set_target_drive_speed(float speed)
{
    target_drive_speed = speed;
}

/**
 * @brief 设置角速度环PID参数
 */
void set_gyro_pid_params(float kp, float ki, float kd)
{
    gyro_pid.kp = kp;
    gyro_pid.ki = ki;
    gyro_pid.kd = kd;
}

/**
 * @brief 设置角度环PID参数
 */
void set_angle_pid_params(float kp, float ki, float kd)
{
    angle_pid.kp = kp;
    angle_pid.ki = ki;
    angle_pid.kd = kd;
}

/**
 * @brief 设置速度环PID参数
 */
void set_speed_pid_params(float kp, float ki, float kd)
{
    speed_pid.kp = kp;
    speed_pid.ki = ki;
    speed_pid.kd = kd;
}

/**
 * @brief 设置行进轮速度环PID参数
 */
void set_drive_speed_pid_params(float kp, float ki, float kd)
{
    drive_speed_pid.kp = kp;
    drive_speed_pid.ki = ki;
    drive_speed_pid.kd = kd;
}

/**
 * @brief 设置输出滤波系数
 * @param coeff 滤波系数 (0.0-1.0)，值越大滤波越强
 */
void set_output_filter_coeff(float coeff)
{
    if (coeff >= 0.0f && coeff <= 1.0f)
    {
        output_filter_coeff = coeff;
    }
}

/**
 * @brief 获取PID状态信息
 */
void get_pid_status(float *gyro_error, float *angle_error, float *speed_error,
                    float *gyro_output, float *angle_output, float *speed_output)
{
    if (gyro_error)
        *gyro_error = gyro_pid.error;
    if (angle_error)
        *angle_error = angle_pid.error;
    if (speed_error)
        *speed_error = speed_pid.error;
    if (gyro_output)
        *gyro_output = gyro_pid.output;
    if (angle_output)
        *angle_output = angle_pid.output;
    if (speed_output)
        *speed_output = speed_pid.output;
}

/**
 * @brief 行进轮速度环控制
 * @note 独立的速度环，使用encoder[1]作为速度反馈，输出PWM控制行进轮电机
 *       支持闭环PID和开环输出两种模式
 */
void drive_speed_loop_control(void)
{
    // 根据使能开关选择控制模式
    if (drive_speed_enable)
    {
        // 闭环PID模式：速度环PID计算
        drive_pwm_output = pid_calculate(&drive_speed_pid, target_drive_speed, (float)encoder[1]);
    }
    else
    {
        // 开环模式：直接使用设定的开环输出值
        drive_pwm_output = drive_open_loop_output;
    }

    // 控制行进轮电机
    drive_wheel_control((int16)drive_pwm_output);
}

/**
 * @brief 设置角度死区
 */
void set_angle_deadzone(float deadzone)
{
    angle_deadzone = deadzone;
}

/**
 * @brief 设置角度保护阈值
 */
void set_angle_protection(float protection)
{
    angle_protection = protection;
}

/**
 * @brief 获取角度死区
 */
float get_angle_deadzone(void)
{
    return angle_deadzone;
}

/**
 * @brief 获取角度保护阈值
 */
float get_angle_protection(void)
{
    return angle_protection;
}

/**
 * @brief 转向PID控制
 * @note P环基于图像中线偏差，D环基于陀螺仪gz（Z轴角速度）
 *       输出控制舵机打角
 */
void steer_pid_control(void)
{
    // 检查转向环是否启用
    if (!steer_enable)
    {
        // 转向环禁用时，舵机保持中点角度
        current_servo_angle = servo_motor_duty;
        current_image_error = 0.0f;  // 禁用时图像误差也为0
        servo_set_angle(servo_motor_duty);
        return;
    }

    // 1. 计算图像中线偏差（P环）
    float image_error = err_sum_average((uint8)steer_sample_start, (uint8)steer_sample_end);

    // 保存图像误差供转弯补偿使用
    current_image_error = image_error;

    // 2. 获取陀螺仪gz（Z轴角速度，偏航角速度）作为D环
    float gyro_gz = (float)imu_data.gyro_z;

    // 3. 计算转向输出：P * 图像偏差 + D * 陀螺仪gz
    float steer_output = steer_kp * image_error + steer_kd * gyro_gz;

    // 4. 输出限幅
    steer_output = constrain(steer_output, -steer_output_limit, steer_output_limit);

    // 5. 计算实际舵机角度并更新全局变量（用于转弯补偿）
    current_servo_angle = servo_center_angle + steer_output;

    // 6. 输出到舵机
    servo_set_angle(current_servo_angle);
}

/**
 * @brief 设置转向PID参数
 * @param kp    P系数（基于图像偏差）
 * @param kd    D系数（基于陀螺仪gz）
 * @param limit 输出限幅（舵机角度限制）
 */
void set_steer_pid_params(float kp, float kd, float limit)
{
    steer_kp = kp;
    steer_kd = kd;
    steer_output_limit = limit;
}
