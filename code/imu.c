// *************************** 头文件包含 ***************************
#include "imu.h"
#include "zf_common_headfile.h"

// *************************** 全局变量定义 ***************************
imu_data_t imu_data = {0}; // IMU数据结构体

// 陀螺仪零偏校准数据（原始数据）
static int16 gyro_x_offset = -2;
static int16 gyro_y_offset = 11;
static int16 gyro_z_offset = 5;
uint32 machine_angle = 500;  // 机械中值（去掉static，允许外部访问）

// 一阶互补滤波参数（只保留必要变量）
static float angle_pitch_temp = 0.0f;
uint8 gyro_ration = 4;     // 陀螺仪比例系数
uint8 acc_ration = 4;      // 加速度计比例系数
float call_cycle = 0.001f; // 调用周期（单位：秒）

// *************************** 函数实现 ***************************

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

    // 获取加速度计数据
    imu660rb_get_acc();
    imu_data.acc_x = imu660rb_acc_x;
    imu_data.acc_y = imu660rb_acc_y;
    imu_data.acc_z = imu660rb_acc_z;

    // 获取陀螺仪数据
    imu660rb_get_gyro();
    imu_data.gyro_x = imu660rb_gyro_x + gyro_x_offset;
    imu_data.gyro_y = imu660rb_gyro_y + gyro_y_offset;
    imu_data.gyro_z = imu660rb_gyro_z + gyro_z_offset;
    if (imu_data.gyro_x > -5 && imu_data.gyro_x < 5)
        imu_data.gyro_x = 0;
    if (imu_data.gyro_y > -5 && imu_data.gyro_y < 5)
        imu_data.gyro_y = 0;
    if (imu_data.gyro_z > -5 && imu_data.gyro_z < 5)
        imu_data.gyro_z = 0;

    imu_data.data_ready = true;
}

void imu_calculate_attitude(void)
{

    // 一阶互补滤波 pitch 角
    float gyro_temp = imu_data.gyro_y * gyro_ration;
    float acc_temp = (imu_data.acc_x - angle_pitch_temp) * acc_ration;
    angle_pitch_temp += ((gyro_temp + acc_temp) * call_cycle);
    imu_data.pitch = angle_pitch_temp + machine_angle;

    // pitch/yaw角按原有方式处理或设为0
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
// 参数说明     sample_count    校准采样次数
// 返回参数     void
// 使用示例     imu_calibrate_gyro(200);
// 备注信息     静止状态下校准陀螺仪零偏
//-------------------------------------------------------------------------------------------------------------------
void imu_calibrate_gyro(uint16 sample_count)
{
    if (!imu_data.is_initialized)
        return;

    int32 gyro_x_sum = 0;
    int32 gyro_y_sum = 0;
    int32 gyro_z_sum = 0;

    for (uint16 i = 0; i < sample_count; i++)
    {
        // 获取陀螺仪原始数据
        imu660rb_get_gyro();
        gyro_x_sum += imu660rb_gyro_x;
        gyro_y_sum += imu660rb_gyro_y;
        gyro_z_sum += imu660rb_gyro_z;

        system_delay_ms(10);
    }

    // 计算平均值作为零偏（原始数据）
    gyro_x_offset = (int16)(gyro_x_sum / sample_count);
    gyro_y_offset = (int16)(gyro_y_sum / sample_count);
    gyro_z_offset = (int16)(gyro_z_sum / sample_count);
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
