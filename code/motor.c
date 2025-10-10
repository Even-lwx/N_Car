// *************************** 头文件包含 ***************************
#include "motor.h"
#include "buzzer.h"
#include "imu.h"
#include "pid.h"
#include "zf_common_headfile.h"

// *************************** 全局变量定义 ***************************
int8 duty = 0;          // 当前占空比
bool dir = true;        // 计数方向
int16 encoder[2] = {0}; // 编码器值

// *************************** 外部变量引用 ***************************
extern volatile bool enable; // PID使能标志(定义在pid.c中)

// *************************** 电机保护相关变量 ***************************
// 保护触发原因枚举
typedef enum
{
    PROTECT_NONE = 0,       // 无保护
    PROTECT_STALL = 1,      // 堵转保护
    PROTECT_DIR_CHANGE = 2, // 方向切换过快保护
    PROTECT_ANGLE = 3       // 角度超限保护
} protect_reason_t;

// 电机保护状态结构体
typedef struct
{
    int16 last_pwm;                  // 上次PWM值
    int8 last_dir;                   // 上次方向 (1=正转, 0=反转, -1=未初始化)
    uint32 last_dir_change_count;    // 上次方向切换时的计数值
    uint32 stall_detect_count;       // 堵转检测计数器
    bool is_protected;               // 是否处于保护状态
    protect_reason_t protect_reason; // 保护触发原因
    uint32 protect_start_count;      // 保护开始时的计数值
} motor_protection_t;

static motor_protection_t motor_protect[2] = {
    {0, -1, 0, 0, false, PROTECT_NONE, 0}, // 动量轮
    {0, -1, 0, 0, false, PROTECT_NONE, 0}  // 行进轮
};

static uint32 protection_tick_count = 0; // 保护更新计数器（每次调用+1）

// *************************** 内部函数声明 ***************************
static void motor_control_with_protection(uint8 motor_id, int16 pwm_value, uint32 pwm_pin, uint32 dir_pin);
static void trigger_protection(uint8 motor_id, protect_reason_t reason);
static void check_angle_protection(void);
static void check_stall_protection(uint8 motor_id);
// 方向切换保护功能已禁用
// static void check_direction_change_protection(uint8 motor_id);

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
// 备注信息     控制动量轮电机的转速和方向，带堵转检测、方向切换保护、PWM渐变
//-------------------------------------------------------------------------------------------------------------------
void momentum_wheel_control(int16 pwm_value)
{
    motor_control_with_protection(0, pwm_value, MOMENTUM_WHEEL_PWM, MOMENTUM_WHEEL_DIR);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     行进轮电机控制函数
// 参数说明     pwm_value: 带符号的PWM数值，范围-10000到10000，正数正转，负数反转，0停止
// 返回参数     void
// 使用示例     drive_wheel_control(7500);   // 正转，PWM数值7500
//              drive_wheel_control(-2000);  // 反转，PWM数值2000
//              drive_wheel_control(0);      // 停止
// 备注信息     控制行进轮电机的转速和方向，带堵转检测、方向切换保护、PWM渐变
//-------------------------------------------------------------------------------------------------------------------
void drive_wheel_control(int16 pwm_value)
{
    motor_control_with_protection(1, pwm_value, DRIVE_WHEEL_PWM, DRIVE_WHEEL_DIR);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制内部函数（带保护）
// 参数说明     motor_id: 电机ID (0=动量轮, 1=行进轮)
//              pwm_value: 带符号的PWM数值
//              pwm_pin: PWM输出引脚
//              dir_pin: 方向控制引脚
// 返回参数     void
// 备注信息     内部函数，实现电机控制的保护逻辑
//-------------------------------------------------------------------------------------------------------------------
static void motor_control_with_protection(uint8 motor_id, int16 pwm_value, uint32 pwm_pin, uint32 dir_pin)
{
    if (motor_id >= 2)
        return;

    motor_protection_t *protect = &motor_protect[motor_id];

    // 如果使能标志为false，强制输出0（优先级最高）
    if (!enable)
    {
        pwm_set_duty(pwm_pin, 0);
        return;
    }

    // 如果处于保护状态，强制输出0
    if (protect->is_protected)
    {
        pwm_set_duty(pwm_pin, 0);
        return;
    }

    // 限制PWM数值范围
    if (pwm_value > MAX_PWM_VALUE)
        pwm_value = MAX_PWM_VALUE;
    if (pwm_value < -MAX_PWM_VALUE)
        pwm_value = -MAX_PWM_VALUE;

    // 正常控制
    if (pwm_value >= 0)
    {
        pwm_set_duty(pwm_pin, pwm_value);
        gpio_set_level(dir_pin, 1);
    }
    else
    {
        pwm_set_duty(pwm_pin, -pwm_value);
        gpio_set_level(dir_pin, 0);
    }

    // 更新状态（供保护检测使用）
    protect->last_pwm = pwm_value;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     触发电机保护
// 参数说明     motor_id: 电机ID (0=动量轮, 1=行进轮)
//              reason: 保护原因
// 返回参数     void
// 备注信息     触发保护时会禁用PID系统、停止电机并启动蜂鸣器报警
//-------------------------------------------------------------------------------------------------------------------
static void trigger_protection(uint8 motor_id, protect_reason_t reason)
{
    if (motor_id >= 2)
        return;

    motor_protection_t *protect = &motor_protect[motor_id];

    // 设置保护状态
    protect->is_protected = true;
    protect->protect_reason = reason;
    protect->protect_start_count = protection_tick_count;

    // 禁用PID控制系统（与倒地保护相同）
    enable = false;

    // 立即停止所有电机（统一保护：一个电机触发保护，所有电机都停止）
    pwm_set_duty(MOMENTUM_WHEEL_PWM, 0);
    pwm_set_duty(DRIVE_WHEEL_PWM, 0);

    // 启动蜂鸣器报警（响2声，每声100ms，间隔100ms）
    if (!buzzer_is_active())
    {
        buzzer_beep(BUZZER_BEEP_COUNT, BUZZER_BEEP_DURATION_MS, BUZZER_BEEP_INTERVAL_MS);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机保护更新函数（需要定时调用，建议1ms周期）
// 参数说明     void
// 返回参数     void
// 使用示例     motor_protection_update(); // 在control()函数中调用
// 备注信息     负责堵转检测、角度保护、方向切换保护等
//-------------------------------------------------------------------------------------------------------------------
void motor_protection_update(void)
{
    // 更新保护系统时间计数器
    protection_tick_count++;

    // 1. 角度保护检查（最高优先级，影响所有电机）
    check_angle_protection();

    // 如果角度保护触发，直接返回，不再进行其他检测
    if (motor_protect[0].is_protected || motor_protect[1].is_protected)
    {
        return;
    }

    // 2. 遍历每个电机进行独立保护检测
    for (uint8 i = 0; i < 2; i++)
    {
        // 如果该电机已处于保护状态，跳过检测（需手动解除保护）
        if (motor_protect[i].is_protected)
        {
            continue;
        }

        // 2.1 堵转保护检测
        check_stall_protection(i);

        // 2.2 方向切换保护检测
        // check_direction_change_protection(i);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度保护检测（内部函数）
// 参数说明     void
// 返回参数     void
// 备注信息     检测pitch角度是否超限，超限时触发保护并清空PID积分项
//-------------------------------------------------------------------------------------------------------------------
static void check_angle_protection(void)
{
    static bool angle_protect_triggered = false;

    // 检查角度是否超出保护阈值
    if (fabs(imu_data.pitch) > angle_protection)
    {
        // 防止重复触发
        if (!angle_protect_triggered)
        {
            // 触发角度保护
            trigger_protection(0, PROTECT_ANGLE);
            angle_protect_triggered = true;

            // 清空所有PID积分项，避免积分饱和
            extern PID_Controller angle_pid, gyro_pid, speed_pid;
            angle_pid.integral = 0;
            gyro_pid.integral = 0;
            speed_pid.integral = 0;

            // 重置输出滤波器状态
            extern float filtered_motor_output;
            filtered_motor_output = 0.0f;
        }
    }
    else
    {
        // 角度恢复正常，允许再次触发保护
        angle_protect_triggered = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     堵转保护检测（内部函数）
// 参数说明     motor_id: 电机ID (0=动量轮, 1=行进轮)
// 返回参数     void
// 备注信息     检测PWM高但编码器反馈低的情况，连续检测超过阈值时间则触发保护
//-------------------------------------------------------------------------------------------------------------------
static void check_stall_protection(uint8 motor_id)
{
    if (motor_id >= 2)
        return;

    motor_protection_t *protect = &motor_protect[motor_id];

    // 计算PWM和编码器的绝对值
    int16 abs_pwm = (protect->last_pwm >= 0) ? protect->last_pwm : -protect->last_pwm;
    int16 abs_encoder = (encoder[motor_id] >= 0) ? encoder[motor_id] : -encoder[motor_id];

    // 判断是否满足堵转条件：PWM高但编码器反馈低
    if (abs_pwm > STALL_DETECT_PWM_THRESHOLD && abs_encoder < STALL_DETECT_ENCODER_THRESHOLD)
    {
        // 累加堵转检测计数器
        protect->stall_detect_count++;

        // 连续检测到堵转超过阈值时间，触发保护
        if (protect->stall_detect_count >= STALL_DETECT_TIME_MS)
        {
            trigger_protection(motor_id, PROTECT_STALL);
            protect->stall_detect_count = 0; // 重置计数器
        }
    }
    else
    {
        // 未检测到堵转，清零计数器
        protect->stall_detect_count = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     方向切换保护检测（内部函数）- 已禁用
// 参数说明     motor_id: 电机ID (0=动量轮, 1=行进轮)
// 返回参数     void
// 备注信息     检测方向切换是否过快，过快则触发保护（功能已禁用）
//-------------------------------------------------------------------------------------------------------------------
/* 方向切换保护功能已禁用
static void check_direction_change_protection(uint8 motor_id)
{
    if (motor_id >= 2)
        return;

    motor_protection_t *protect = &motor_protect[motor_id];

    // 根据PWM符号确定当前方向 (1=正转, 0=反转)
    int8 current_dir = (protect->last_pwm >= 0) ? 1 : 0;

    // 检查是否发生方向切换（排除首次运行的情况）
    if (protect->last_dir != -1 && protect->last_dir != current_dir)
    {
        // 计算距离上次方向切换的时间间隔
        uint32 time_since_last_change = protection_tick_count - protect->last_dir_change_count;

        // 判断切换间隔是否过短
        if (time_since_last_change < DIR_CHANGE_MIN_INTERVAL_MS)
        {
            // 方向切换过快，触发保护
            trigger_protection(motor_id, PROTECT_DIR_CHANGE);
        }
        else
        {
            // 方向切换间隔正常，更新记录
            protect->last_dir_change_count = protection_tick_count;
            protect->last_dir = current_dir;
        }
    }
    else
    {
        // 首次运行或方向未改变，更新当前方向
        if (protect->last_dir == -1)
        {
            protect->last_dir = current_dir;
            protect->last_dir_change_count = protection_tick_count;
        }
    }
}
*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置电机保护状态
// 参数说明     void
// 返回参数     void
// 使用示例     motor_reset_protection(); // 手动解除保护
// 备注信息     用于手动解除所有电机的保护状态
//-------------------------------------------------------------------------------------------------------------------
void motor_reset_protection(void)
{
    for (uint8 i = 0; i < 2; i++)
    {
        motor_protect[i].is_protected = false;
        motor_protect[i].protect_reason = PROTECT_NONE;
        motor_protect[i].stall_detect_count = 0;
    }

    // 停止蜂鸣器
    buzzer_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询电机是否处于保护状态
// 参数说明     motor_id: 电机ID (0=动量轮, 1=行进轮)
// 返回参数     bool: true=处于保护状态, false=正常
// 使用示例     if(motor_is_stall_protected(0)) { ... }
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
bool motor_is_stall_protected(uint8 motor_id)
{
    if (motor_id >= 2)
        return false;
    return motor_protect[motor_id].is_protected;
}

