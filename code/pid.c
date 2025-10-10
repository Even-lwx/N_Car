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

// 目标值
float target_gyro_rate = 0.0f;    // 目标角速度
float target_angle = 0.0f;        // 目标角度
float target_speed = 0.0f;        // 目标速度
float target_drive_speed = 10.0f; // 行进轮目标速度

// 控制标志
volatile bool enable = false; // 使能标志，默认禁用（需在Cargo模式中启用）

// 控制变量
static uint32_t count = 0;             // 控制计数器
static float desired_angle = 0.0f;     // 期望角度（速度环输出）
static float angle_gyro_target = 0.0f; // 目标角速度（角度环输出）

// 一阶低通滤波器相关变量（仅对PID输出滤波）
float output_filter_coeff = 0.6f;   // 输出滤波系数 (0-1)，降低以增强平滑度
float filtered_motor_output = 0.0f; // 滤波后的电机输出

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
    // 使用IMU中已经滤波后的pitch角度（IMU中已对原始数据进行滤波再解算）
    float current_pitch = imu_data.pitch;

    // 使用desired_angle作为目标角度（已由速度环更新），用IMU滤波后的pitch
    angle_gyro_target = pid_calculate(&angle_pid, desired_angle, current_pitch);
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
    // 速度环周期采集编码器数据
    if (count % 20 == 0)
    {
        motor_encoder_update();
    }
    // printf("%f,%d\r\n", imu_data.pitch, imu_data.gyro_y);
    //  如果未启用控制，停止电机并返回
    if (!enable)
    {
        momentum_wheel_control(0);
        drive_wheel_control(0);
        return;
    }
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

    // 角度环控制（5ms周期，每5个1ms周期执行一次）
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
 */
void drive_speed_loop_control(void)
{
    // 速度环PID计算
    float drive_pwm_output = pid_calculate(&drive_speed_pid, target_drive_speed, (float)encoder[1]);

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
