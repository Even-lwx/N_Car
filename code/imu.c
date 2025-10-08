// *************************** 头文件包含 ***************************
#include "imu.h"
#include "zf_common_headfile.h"

// *************************** 全局变量定义 ***************************
imu_data_t imu_data = {0}; // IMU数据结构体

// 陀螺仪零偏校准数据（原始数据）
int16 gyro_x_offset = 0;
int16 gyro_y_offset = 0;
int16 gyro_z_offset = 0;
uint32 machine_angle = 0; // 机械中值（去掉static，允许外部访问）

// 二阶Butterworth低通滤波器结构体
// 二阶Butterworth低通滤波器结构体（已禁用，不再使用）
/*
typedef struct
{
    float b0, b1, b2; // 分子系数
    float a1, a2;     // 分母系数（归一化，a0=1）
    float x1, x2;     // 输入历史值
    float y1, y2;     // 输出历史值
} Butterworth2ndFilter;

// 滤波器实例（仅对gyro_y滤波）
static Butterworth2ndFilter gyro_y_filter;
*/

// 一阶互补滤波参数（只保留必要变量）
static float angle_pitch_temp = 0.0f;
uint8 gyro_ration = 4;     // 陀螺仪比例系数
uint8 acc_ration = 4;      // 加速度计比例系数
float call_cycle = 0.002f; // 调用周期（单位：秒）

// *************************** 函数实现 ***************************

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化二阶Butterworth低通滤波器
// 参数说明     filter          滤波器结构体指针
//              cutoff_freq     截止频率 (Hz)
//              sample_freq     采样频率 (Hz)
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化二阶Butterworth低通滤波器（已禁用，不再使用）
// 参数说明     filter          滤波器结构体指针
//              cutoff_freq     截止频率 (Hz)
//              sample_freq     采样频率 (Hz)
// 返回参数     void
// 备注信息     不需要滤波，保留代码仅供参考
//-------------------------------------------------------------------------------------------------------------------
/*
static void butterworth_init(Butterworth2ndFilter *filter, float cutoff_freq, float sample_freq)
{
    // 计算归一化截止频率
    float omega_c = 2.0f * 3.14159265f * cutoff_freq / sample_freq;
    float cos_wc = cosf(omega_c);
    float sin_wc = sinf(omega_c);
    float alpha = sin_wc / (2.0f * 0.707f); // Q = 0.707 (Butterworth)

    // 计算系数
    float b0 = (1.0f - cos_wc) / 2.0f;
    float b1 = 1.0f - cos_wc;
    float b2 = (1.0f - cos_wc) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_wc;
    float a2 = 1.0f - alpha;

    // 归一化系数
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // 初始化历史值
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}
*/
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     二阶Butterworth低通滤波器处理（已禁用，不再使用）
// 参数说明     filter          滤波器结构体指针
//              input           输入值
// 返回参数     float           滤波后的输出值
// 备注信息     不需要滤波，保留代码仅供参考
//-------------------------------------------------------------------------------------------------------------------
/*
static float butterworth_filter(Butterworth2ndFilter *filter, float input)
{
    // 差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter->b0 * input +
                   filter->b1 * filter->x1 +
                   filter->b2 * filter->x2 -
                   filter->a1 * filter->y1 -
                   filter->a2 * filter->y2;

    // 更新历史值
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}
*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU初始化函数
// 参数说明     void
// 返回参数     uint8 初始化结果 1-成功 0-失败(和imu660rb_init()返回值相反)
// 使用示例     uint8 result = imu_init();
// 备注信息     初始化IMU660RB陀螺仪传感器
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{

    // 初始化IMU660RB
    while (1)
    {
        if (imu660rb_init())
        {
            imu_data.is_initialized = false;
            /*debug */
            // printf("\r\n IMU660RB init error."); // IMU660RB 初始化失败
            //  printf("return_state=%d", imu660rb_init());
        }
        else
        {
            imu_data.is_initialized = true;
            /*debug*/
            // printf("IMU660RB init success");
            break;
        }
    }

    // 初始化Butterworth滤波器（已禁用，不再使用）
    // butterworth_init(&gyro_y_filter, 10.0f, 1000.0f);

    return imu_data.is_initialized ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取IMU原始数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu_get_data();
// 备注信息     读取加速度计和陀螺仪的原始数据
//-------------------------------------------------------------------------------------------------------------------
void imu_get_data(void)
{
    if (!imu_data.is_initialized)
        return;

    // 获取加速度计数据（不滤波）
    imu660rb_get_acc();
    imu_data.acc_x = imu660rb_acc_x;
    imu_data.acc_y = imu660rb_acc_y;
    imu_data.acc_z = imu660rb_acc_z;

    // 获取陀螺仪数据并应用零偏
    imu660rb_get_gyro();
    imu_data.gyro_x = imu660rb_gyro_x + gyro_x_offset;
    imu_data.gyro_y = imu660rb_gyro_y + gyro_y_offset;
    imu_data.gyro_z = imu660rb_gyro_z + gyro_z_offset;

    // 死区处理（不滤波）
    if (imu_data.gyro_x > -5 && imu_data.gyro_x < 5)
        imu_data.gyro_x = 0;
    if (imu_data.gyro_y > -5 && imu_data.gyro_y < 5)
        imu_data.gyro_y = 0;
    if (imu_data.gyro_z > -5 && imu_data.gyro_z < 5)
        imu_data.gyro_z = 0;

    // gyro_y滤波处理（已禁用，不需要滤波）
    // float raw_gyro_y = (float)(imu660rb_gyro_y + gyro_y_offset);
    // if (raw_gyro_y > -5.0f && raw_gyro_y < 5.0f)
    //     raw_gyro_y = 0.0f;
    // imu_data.gyro_y = (int16)butterworth_filter(&gyro_y_filter, raw_gyro_y);

    // 输出原始数据和滤波后的数据
    // printf("%d,%d\r\n", imu_data.gyro_y, (int16)raw_gyro_y);

    imu_data.data_ready = true;
}

void imu_calculate_attitude(void)
{

    // 一阶互补滤波 pitch 角
    float gyro_temp = imu_data.gyro_y * gyro_ration;
    float acc_temp = (imu_data.acc_x - angle_pitch_temp) * acc_ration;
    angle_pitch_temp += ((gyro_temp + acc_temp) * call_cycle);
    imu_data.pitch = angle_pitch_temp + machine_angle;

    // roll/yaw角按原有方式处理或设为0
    imu_data.roll = 0.0f;
    imu_data.yaw = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU数据更新函数
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update(); // 在定时器中断中调用
// 备注信息     获取IMU数据并计算姿态角
//-------------------------------------------------------------------------------------------------------------------
void imu_update(void)
{
    imu_get_data();
    imu_calculate_attitude();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪校准函数
// 参数说明     sample_count    采样次数
// 返回参数     void
// 使用示例     imu_calibrate_gyro(2000);
// 备注信息     采样指定次数陀螺仪原始数据取平均值作为零偏，结果保存到flash
//-------------------------------------------------------------------------------------------------------------------
void imu_calibrate_gyro(uint16 sample_count)
{
    if (!imu_data.is_initialized)
        return;

    if (sample_count == 0)
        sample_count = 2000; // 默认2000次

    int32 gyro_x_sum = 0;
    int32 gyro_y_sum = 0;
    int32 gyro_z_sum = 0;

    // 采样期间临时设为0（保证读取未校正的原始数据）
    gyro_x_offset = 0;
    gyro_y_offset = 0;
    gyro_z_offset = 0;

    // 采样（利用中断中更新的数据，避免总线冲突）
    for (uint16 i = 0; i < sample_count; i++)
    {
        // 等待数据更新标志（由1ms中断设置）
        imu_data.data_ready = false;
        while (!imu_data.data_ready)
        {
            // 等待中断更新数据
        }

        // 累加未校正的陀螺仪数据
        gyro_x_sum += imu_data.gyro_x;
        gyro_y_sum += imu_data.gyro_y;
        gyro_z_sum += imu_data.gyro_z;
    }

    // 计算平均值作为零偏
    gyro_x_offset = -(int16)(gyro_x_sum / sample_count);
    gyro_y_offset = -(int16)(gyro_y_sum / sample_count);
    gyro_z_offset = -(int16)(gyro_z_sum / sample_count);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取横滚角
// 参数说明     void
// 返回参数     float 横滚角(度)
// 使用示例     float roll = imu_get_roll();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu_get_roll(void)
{
    return imu_data.roll;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取俯仰角
// 参数说明     void
// 返回参数     float 俯仰角(度)
// 使用示例     float pitch = imu_get_pitch();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu_get_pitch(void)
{
    return imu_data.pitch;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取偏航角
// 参数说明     void
// 返回参数     float 偏航角(度)
// 使用示例     float yaw = imu_get_yaw();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu_get_yaw(void)
{
    return imu_data.yaw;
}
