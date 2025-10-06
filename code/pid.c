#include "pid.h"
#include "zf_common_headfile.h"

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
bool enable = false; // 使能标志，默认禁用（需在Cargo模式中启用）

// 控制变量
static uint32_t count = 0;                      // 控制计数器
static float desired_angle = 0.0f;              // 期望角度（速度环输出）
static float angle_gyro_target = 0.0f;          // 目标角速度（角度环输出）
static uint32_t speed_integral_clear_count = 0; // 速度环积分清零计数器

// 低通滤波器相关变量

static float gyro_filter_coeff = 1.0f;     // 陀螺仪数据滤波系数 (0-1, 值越大滤波越强)
static float output_filter_coeff = 0.3f;   // 输出滤波系数
static float angle_filter_coeff = 0.4f;    // 角度环滤波系数 (0-1)
static float filtered_gyro_y = 0.0f;       // 滤波后的陀螺仪Y轴数据
static float filtered_motor_output = 0.0f; // 滤波后的电机输出
static float filtered_pitch = 0.0f;        // 滤波后的pitch角度

// 控制优化参数（导出到菜单）
float angle_deadzone = 5.0f;      // 角度死区
float angle_protection = 1500.0f; // 角度保护阈值

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

    // 角度保护：超出有效作用区间时停止电机
    if (fabs(imu_data.pitch) > angle_protection)
    {
        // 清空所有积分项，避免积分饱和
        angle_pid.integral = 0;
        gyro_pid.integral = 0;
        speed_pid.integral = 0;

        // 重置滤波器状态
        filtered_gyro_y = 0.0f;
        filtered_motor_output = 0.0f;

        momentum_wheel_control(0);
        return;
    }

    // 对陀螺仪数据进行低通滤波（一阶低通滤波器）
    // filtered_value = α * current_value + (1-α) * previous_filtered_value
    filtered_gyro_y = gyro_filter_coeff * imu_data.gyro_y + (1.0f - gyro_filter_coeff) * filtered_gyro_y;

    // 变增益控制：根据误差大小动态调整增益
    float gyro_error = target_gyro - filtered_gyro_y;
    float gain_factor = 1.0f;

    // 如果误差在死区附近，降低增益减少震荡
    if (fabs(gyro_error) < 100.0f) // 角速度死区阈值（可调）
    {
        // 线性插值：误差越小，增益越低
        gain_factor = gyro_gain_scale + (1.0f - gyro_gain_scale) * (fabs(gyro_error) / 100.0f);
    }

    // 临时调整PID参数
    float original_kp = gyro_pid.kp;
    float original_ki = gyro_pid.ki;
    gyro_pid.kp *= gain_factor;
    gyro_pid.ki *= gain_factor;

    // 角速度环PID计算（使用滤波后的陀螺仪数据）
    float motor_output = pid_calculate(&gyro_pid, target_gyro, filtered_gyro_y);

    // 恢复原始参数
    gyro_pid.kp = original_kp;
    gyro_pid.ki = original_ki;

    // 对PID输出进行低通滤波，减少输出抖动
    filtered_motor_output = output_filter_coeff * motor_output + (1.0f - output_filter_coeff) * filtered_motor_output;

    // 控制电机（使用滤波后的输出）
    momentum_wheel_control((int16_t)filtered_motor_output);
}

/**
 * @brief 角度环控制（中间环）
 * @param speed_control 速度环的输出，作为角度环的目标偏移（暂时不用，通过desired_angle传递）
 */
void angle_loop_control(int speed_control)
{
    // 对pitch角度进行一阶低通滤波
    filtered_pitch = angle_filter_coeff * imu_data.pitch + (1.0f - angle_filter_coeff) * filtered_pitch;

    // 角度死区处理与变增益控制
    float angle_error = desired_angle - filtered_pitch;
    float gain_factor = 1.0f;

    if (fabs(angle_error) < angle_deadzone)
    {
        // 死区内：衰减积分 + 降低增益
        angle_pid.integral *= 0.95f;
        // 线性插值：误差越小，增益越低
        gain_factor = angle_gain_scale + (1.0f - angle_gain_scale) * (fabs(angle_error) / angle_deadzone);
    }

    // 临时调整PID参数（角度环是PD，只调Kp和Kd）
    float original_kp = angle_pid.kp;
    float original_kd = angle_pid.kd;
    angle_pid.kp *= gain_factor;
    angle_pid.kd *= gain_factor;

    // 使用desired_angle作为目标角度（已由速度环更新），用滤波后的pitch
    angle_gyro_target = pid_calculate(&angle_pid, desired_angle, filtered_pitch);

    // 恢复原始参数
    angle_pid.kp = original_kp;
    angle_pid.kd = original_kd;
}

/**
 * @brief 速度环控制（最外环）
 */
void speed_loop_control(void)
{
    speed_integral_clear_count++;

    // 定时清零积分项（每1秒清零一次，10ms * 100 = 1000ms）
    if (speed_integral_clear_count >= 100)
    {
        speed_pid.integral = 0;
        speed_integral_clear_count = 0;
    }

    // 速度环PID计算，输出作为角度偏移
    float speed_angle_offset = pid_calculate(&speed_pid, target_speed, encoder[0]);

    // 更新期望角度
    desired_angle = target_angle + speed_angle_offset;
}

/**
 * @brief 主控制函数（多频率三环PID控制）
 */
void control(void)
{
    count++;

    // 传感器数据更新（始终执行，不受enable控制）
    motor_protection_update();
    imu_update();

    // 速度环周期采集编码器（20ms周期）
    if (count % 20 == 0)
    {
        motor_encoder_update();
    }

    // 如果未启用控制，传感器数据已更新，直接返回不执行PID
    if (!enable)
    {
        return;
    }

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

    // 行进轮速度环控制（20ms周期，每20个1ms周期执行一次）
    if (count % 20 == 0)
    {

        drive_speed_loop_control();
    }

    // 角速度环控制（1ms周期，每次都执行）
    gyro_loop_control((int)angle_gyro_target);

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
    filtered_gyro_y = 0.0f;
    filtered_motor_output = 0.0f;
    filtered_pitch = 0.0f;
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
 * @brief 设置陀螺仪数据滤波系数
 * @param coeff 滤波系数 (0.0-1.0)，值越大滤波越强
 */
void set_gyro_filter_coeff(float coeff)
{
    if (coeff >= 0.0f && coeff <= 1.0f)
    {
        gyro_filter_coeff = coeff;
    }
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
 * @brief 获取陀螺仪数据滤波系数
 */
float get_gyro_filter_coeff(void)
{
    return gyro_filter_coeff;
}

/**
 * @brief 获取输出滤波系数
 */
float get_output_filter_coeff(void)
{
    return output_filter_coeff;
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
