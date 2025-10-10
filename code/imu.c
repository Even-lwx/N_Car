/*********************************************************************************************************************
 * 文件名称: imu.c
 * 功能说明: IMU惯性测量单元驱动（支持一阶互补滤波和EKF两种姿态解算算法）
 * 作    者: N_Car项目组
 * 日    期: 2025-01-09
 * 备    注: 基于逐飞TC264开源库，集成了IMU660RB传感器驱动和两种姿态解算算法
 ********************************************************************************************************************/

// *************************** 头文件包含 ***************************
#include "imu.h"
#include "zf_common_headfile.h"
#include "EKF/Attitude.h"      // EKF姿态解算库
#include "EKF/QuaternionEKF.h" // EKF四元数滤波库

// *************************** 宏定义 ***************************
#define DEG_TO_RAD 0.0174533f  // 度转弧度系数（π/180）
#define RAD_TO_DEG 57.2957795f // 弧度转度系数（180/π）

// *************************** 全局变量定义 ***************************
imu_data_t imu_data = {0}; // IMU数据结构体（加速度、陀螺仪、姿态角）

// ======================== 算法选择配置 ========================
// 功能: 切换IMU姿态解算算法
// 说明: 修改此值后需重新编译和烧录
//       0 = 一阶互补滤波（默认，计算快速，只输出pitch角）
//       1 = EKF扩展卡尔曼滤波（精度高，输出roll/pitch/yaw三轴）
uint8 imu_algorithm_select = 1; // 默认使用EKF算法（高精度）
// ================================================================

// *************************** 校准参数 *********************
int16 gyro_x_offset = 0; // 陀螺仪X轴零偏（原始数据）
int16 gyro_y_offset = 0; // 陀螺仪Y轴零偏（原始数据）
int16 gyro_z_offset = 0; // 陀螺仪Z轴零偏（原始数据）
float machine_angle = 0; // 机械中值角度偏移（度）

// *************************** 一阶互补滤波参数 ***************************
static float angle_pitch_temp = 0.0f; // Pitch角临时值（用于积分计算）
uint8 gyro_ration = 4;                // 陀螺仪权重系数（可调参数）
uint8 acc_ration = 4;                 // 加速度计权重系数（可调参数）
float call_cycle = 0.002f;            // 解算周期（单位：秒，2ms）

// *************************** 函数实现 ***************************

/*********************************************************************************************************************
 * @brief       IMU初始化函数
 * @param       void
 * @return      uint8           初始化结果（1=成功, 0=失败）
 * @note        初始化IMU660RB传感器，并根据选择的算法进行相应初始化
 * @example     uint8 result = imu_init();
 ********************************************************************************************************************/
uint8 imu_init(void)
{
    // ========== 初始化IMU660RB传感器 ==========
    while (1)
    {
        if (imu660rb_init())
        {
            // 初始化失败，继续尝试
            imu_data.is_initialized = false;
        }
        else
        {
            // 初始化成功
            imu_data.is_initialized = true;
            break;
        }
    }

    // ========== 根据算法选择进行初始化 ==========
    if (imu_algorithm_select == IMU_ALGORITHM_EKF)
    {
        // ---------- EKF算法初始化 ----------
        // 参数说明:
        //   process_noise1    : 四元数过程噪声（100）
        //   process_noise2    : 陀螺仪零偏过程噪声（0.00001）
        //   measure_noise     : 加速度计量测噪声（100000000）
        //   lambda            : 渐消因子（0.9996，防止滤波器发散）
        //   dt                : 更新周期（0.002s = 2ms，与实际调用周期一致）
        //   lpf               : 低通滤波系数（0=不使用）
        IMU_QuaternionEKF_Init(100, 0.00001, 100000000, 0.9996, 0.002f, 0);

        // 设置陀螺仪零偏初始值（从校准数据读取）
        gyroscopeOffset[0] = gyro_x_offset;
        gyroscopeOffset[1] = gyro_y_offset;
        gyroscopeOffset[2] = gyro_z_offset;
    }
    else
    {
        // ---------- 一阶互补滤波初始化 ----------
        // 清零临时变量（原有算法无需额外初始化）
        angle_pitch_temp = 0.0f;
    }

    return imu_data.is_initialized ? 1 : 0;
}

/*********************************************************************************************************************
 * @brief       获取IMU原始数据
 * @param       void
 * @return      void
 * @note        读取加速度计和陀螺仪的原始数据，并应用零偏校准和死区处理
 * @example     imu_get_data();
 ********************************************************************************************************************/
void imu_get_data(void)
{
    if (!imu_data.is_initialized)
        return;

    // ========== 读取加速度计数据 ==========
    imu660rb_get_acc();
    imu_data.acc_x = imu660rb_acc_x;
    imu_data.acc_y = imu660rb_acc_y;
    imu_data.acc_z = imu660rb_acc_z;

    // ========== 读取陀螺仪数据并应用零偏校准 ==========
    imu660rb_get_gyro();
    imu_data.gyro_x = imu660rb_gyro_x + gyro_x_offset;
    imu_data.gyro_y = imu660rb_gyro_y + gyro_y_offset;
    imu_data.gyro_z = imu660rb_gyro_z + gyro_z_offset;

    // ========== 陀螺仪死区处理（滤除小幅度噪声） ==========
    if (imu_data.gyro_x > -5 && imu_data.gyro_x < 5)
        imu_data.gyro_x = 0;
    if (imu_data.gyro_y > -5 && imu_data.gyro_y < 5)
        imu_data.gyro_y = 0;
    if (imu_data.gyro_z > -5 && imu_data.gyro_z < 5)
        imu_data.gyro_z = 0;

    // 标记数据就绪
    imu_data.data_ready = true;
}

/*********************************************************************************************************************
 * @brief       计算姿态角（一阶互补滤波算法）
 * @param       void
 * @return      void
 * @note        使用一阶互补滤波计算pitch角，保留原有算法（仅输出pitch）
 * @example     imu_calculate_attitude_complementary();
 ********************************************************************************************************************/
void imu_calculate_attitude_complementary(void)
{
    // ========== 一阶互补滤波计算pitch角 ==========
    // 公式: angle += (gyro_weight * gyro + acc_weight * acc_error) * dt
    float gyro_temp = imu_data.gyro_y * gyro_ration;                   // 陀螺仪积分项
    float acc_temp = (imu_data.acc_x - angle_pitch_temp) * acc_ration; // 加速度修正项
    angle_pitch_temp += ((gyro_temp + acc_temp) * call_cycle);         // 融合计算

    // 应用机械中值偏移
    imu_data.pitch = angle_pitch_temp + machine_angle;

    // roll/yaw角不由此算法提供
    imu_data.roll = 0.0f;
    imu_data.yaw = 0.0f;
}

/*********************************************************************************************************************
 * @brief       计算姿态角（EKF扩展卡尔曼滤波算法）
 * @param       void
 * @return      void
 * @note        使用四元数EKF计算三轴姿态角（roll/pitch/yaw）
 * @example     imu_calculate_attitude_ekf();
 ********************************************************************************************************************/
void imu_calculate_attitude_ekf(void)
{
    // ========== 单位转换：原始数据 → 物理单位 ==========
    // 陀螺仪: 原始值 → 弧度/秒
    float gx = imu660rb_gyro_transition(imu_data.gyro_x) * DEG_TO_RAD;
    float gy = imu660rb_gyro_transition(imu_data.gyro_y) * DEG_TO_RAD;
    float gz = imu660rb_gyro_transition(imu_data.gyro_z) * DEG_TO_RAD;

    // 加速度计: 原始值 → g
    float ax = imu660rb_acc_transition(imu_data.acc_x);
    float ay = imu660rb_acc_transition(imu_data.acc_y);
    float az = imu660rb_acc_transition(imu_data.acc_z);

    // ========== 陀螺仪死区处理（EKF专用） ==========
    const float gyro_deadzone = 0.008f; // 0.008 rad/s ≈ 0.46°/s
    if (fabsf(gx) < gyro_deadzone)
        gx = 0;
    if (fabsf(gy) < gyro_deadzone)
        gy = 0;
    if (fabsf(gz) < gyro_deadzone)
        gz = 0;

    // ========== 调用EKF更新函数 ==========
    IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az);

    // ========== 从EKF获取姿态角（度） ==========
    // 注意: 取反以匹配原有坐标系
    imu_data.pitch = -QEKF_INS.Pitch; // 俯仰角
    imu_data.roll = -QEKF_INS.Roll;   // 横滚角
    imu_data.yaw = QEKF_INS.Yaw;      // 偏航角

    // 应用机械中值偏移
    imu_data.pitch += machine_angle;
}

/*********************************************************************************************************************
 * @brief       计算姿态角（自动选择算法）
 * @param       void
 * @return      void
 * @note        根据imu_algorithm_select的值自动选择算法
 * @example     imu_calculate_attitude();
 ********************************************************************************************************************/
void imu_calculate_attitude(void)
{
    if (imu_algorithm_select == IMU_ALGORITHM_EKF)
    {
        imu_calculate_attitude_ekf(); // 使用EKF算法
    }
    else
    {
        imu_calculate_attitude_complementary(); // 使用互补滤波（默认）
    }
}

/*********************************************************************************************************************
 * @brief       IMU数据更新函数
 * @param       void
 * @return      void
 * @note        在定时器中断中周期性调用（2ms）
 * @example     imu_update(); // 在1ms中断中，每隔一次调用
 ********************************************************************************************************************/
void imu_update(void)
{
    imu_get_data();           // 读取传感器数据
    imu_calculate_attitude(); // 计算姿态角
}

/*********************************************************************************************************************
 * @brief       陀螺仪零偏校准函数
 * @param       sample_count    采样次数（0=使用默认值2000）
 * @return      void
 * @note        车体静止时调用，采样多次取平均值作为零偏
 * @example     imu_calibrate_gyro(2000);
 ********************************************************************************************************************/
void imu_calibrate_gyro(uint16 sample_count)
{
    if (!imu_data.is_initialized)
        return;

    // 使用默认采样次数
    if (sample_count == 0)
        sample_count = 2000;

    int32 gyro_x_sum = 0;
    int32 gyro_y_sum = 0;
    int32 gyro_z_sum = 0;

    // ========== 临时清零偏，以读取未校正的原始数据 ==========
    gyro_x_offset = 0;
    gyro_y_offset = 0;
    gyro_z_offset = 0;

    // ========== 采样循环 ==========
    for (uint16 i = 0; i < sample_count; i++)
    {
        // 等待数据更新（由1ms中断设置data_ready标志）
        imu_data.data_ready = false;
        while (!imu_data.data_ready)
        {
            // 等待中断更新数据
        }

        // 累加陀螺仪原始数据
        gyro_x_sum += imu_data.gyro_x;
        gyro_y_sum += imu_data.gyro_y;
        gyro_z_sum += imu_data.gyro_z;
    }

    // ========== 计算平均值作为零偏（取负值用于抵消） ==========
    gyro_x_offset = -(int16)(gyro_x_sum / sample_count);
    gyro_y_offset = -(int16)(gyro_y_sum / sample_count);
    gyro_z_offset = -(int16)(gyro_z_sum / sample_count);
}

/*********************************************************************************************************************
 * @brief       获取横滚角
 * @param       void
 * @return      float           横滚角（度）
 * @note        仅EKF算法有效，互补滤波返回0
 * @example     float roll = imu_get_roll();
 ********************************************************************************************************************/
float imu_get_roll(void)
{
    return imu_data.roll;
}

/*********************************************************************************************************************
 * @brief       获取俯仰角
 * @param       void
 * @return      float           俯仰角（度）
 * @note        两种算法均有效
 * @example     float pitch = imu_get_pitch();
 ********************************************************************************************************************/
float imu_get_pitch(void)
{
    return imu_data.pitch;
}

/*********************************************************************************************************************
 * @brief       获取偏航角
 * @param       void
 * @return      float           偏航角（度）
 * @note        仅EKF算法有效，互补滤波返回0
 * @example     float yaw = imu_get_yaw();
 ********************************************************************************************************************/
float imu_get_yaw(void)
{
    return imu_data.yaw;
}
