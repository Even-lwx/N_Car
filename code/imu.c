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
#include "EKF/Attitude.h"             // EKF姿态解算库
#include "EKF/QuaternionEKF.h"        // EKF四元数滤波库
#include "imu_calibration_improved.h" // 改进版加速度计校准库
#include <stdio.h>

// *************************** 宏定义 ***************************
#define DEG_TO_RAD 0.0174533f  // 度转弧度系数（π/180）
#define RAD_TO_DEG 57.2957795f // 弧度转度系数（180/π）

// *************************** 全局变量定义 ***************************
imu_data_t imu_data = {0}; // IMU数据结构体（加速度、陀螺仪、姿态角）
imu_dual_data_t imu_dual_data = {0}; // 双算法数据结构体（测试模式用）

// ======================== 算法选择配置 ========================
// 功能: 切换IMU姿态解算算法
// 说明: 修改此值后需重新编译和烧录
//       0 = 一阶互补滤波（默认，计算快速，输出roll和pitch角）
//       1 = EKF扩展卡尔曼滤波（精度高，输出roll/pitch/yaw三轴）
//       2 = 双算法同时运行（测试模式，两种算法结果分别输出）
uint8 imu_algorithm_select = 2; // 设置为测试模式
// ================================================================
// *************************** 校准参数 *********************
int16 gyro_x_offset = 0; // 陀螺仪X轴零偏（原始数据）
int16 gyro_y_offset = 0; // 陀螺仪Y轴零偏（原始数据）
int16 gyro_z_offset = 0; // 陀螺仪Z轴零偏（原始数据）
float machine_angle = 0; // 机械中值角度偏移（度）

// *************************** 一阶互补滤波参数 ***************************
static float angle_pitch_temp = 0.0f; // Pitch角临时值（用于积分计算）
static float angle_roll_temp = 0.0f;  // Roll角临时值（用于积分计算）
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
        IMU_QuaternionEKF_Init(0.1, 1e-1, 1e6, 0.9996, 0.002f, 0);

        // 设置陀螺仪零偏初始值（从校准数据读取）
        gyroscopeOffset[0] = gyro_x_offset;
        gyroscopeOffset[1] = gyro_y_offset;
        gyroscopeOffset[2] = gyro_z_offset;
    }
    else if (imu_algorithm_select == 2) // 双算法模式
    {
        // ---------- 双算法初始化（同时初始化互补滤波和EKF） ----------
        // 初始化互补滤波
        angle_pitch_temp = 0.0f;
        angle_roll_temp = 0.0f;
        
        // 初始化EKF
        IMU_QuaternionEKF_Init(0.1, 1e-1, 1e6, 0.9996, 0.002f, 0);
        gyroscopeOffset[0] = gyro_x_offset;
        gyroscopeOffset[1] = gyro_y_offset;
        gyroscopeOffset[2] = gyro_z_offset;
    }
    else
    {
        // ---------- 一阶互补滤波初始化 ----------
        // 清零临时变量（原有算法无需额外初始化）
        angle_pitch_temp = 0.0f;
        angle_roll_temp = 0.0f;
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
 * @brief       应用加速度计校准参数
 * @param       ax, ay, az  原始加速度值（g）
 * @param       ax_out, ay_out, az_out  校准后的加速度值（g）
 * @note        应用零偏和缩放因子校准：acc_calibrated = (acc_raw - bias) * scale
 * @example     apply_acc_calibration(ax, ay, az, &ax_cal, &ay_cal, &az_cal);
 ********************************************************************************************************************/
static inline void apply_acc_calibration(float ax, float ay, float az,
                                         float *ax_out, float *ay_out, float *az_out)
{
    // 检查是否已校准
    if (g_acc_calib_params.calibrated)
    {
        // 应用校准参数：(原始值 - 零偏) × 缩放因子
        *ax_out = (ax - g_acc_calib_params.bias_x) * g_acc_calib_params.scale_x;
        *ay_out = (ay - g_acc_calib_params.bias_y) * g_acc_calib_params.scale_y;
        *az_out = (az - g_acc_calib_params.bias_z) * g_acc_calib_params.scale_z;
    }
    else
    {
        // 未校准，直接使用原始值
        *ax_out = ax;
        *ay_out = ay;
        *az_out = az;
    }
}

/*********************************************************************************************************************
 * @brief       计算姿态角（一阶互补滤波算法）
 * @param       void
 * @return      void
 * @note        使用一阶互补滤波计算pitch和roll角
 * @example     imu_calculate_attitude_complementary();
 ********************************************************************************************************************/
void imu_calculate_attitude_complementary(void)
{
    // ========== 转换加速度计数据并应用校准 ==========
    float ax_raw = imu660rb_acc_transition(imu_data.acc_x);
    float ay_raw = imu660rb_acc_transition(imu_data.acc_y);
    float az_raw = imu660rb_acc_transition(imu_data.acc_z);

    float ax, ay, az;
    apply_acc_calibration(ax_raw, ay_raw, az_raw, &ax, &ay, &az);

    // ========== 一阶互补滤波计算pitch角 ==========
    // 公式: angle += (gyro_weight * gyro + acc_weight * acc_error) * dt
    float gyro_pitch = imu_data.gyro_y * gyro_ration;            // 陀螺仪积分项（Y轴陀螺仪对应pitch）
    float acc_pitch = (ax - angle_pitch_temp) * acc_ration;      // 加速度修正项（使用校准后的ax）
    angle_pitch_temp += ((gyro_pitch + acc_pitch) * call_cycle); // 融合计算

    // ========== 一阶互补滤波计算roll角 ==========
    float gyro_roll = imu_data.gyro_x * gyro_ration;          // 陀螺仪积分项（X轴陀螺仪对应roll）
    float acc_roll = (ay - angle_roll_temp) * acc_ration;     // 加速度修正项（使用校准后的ay）
    angle_roll_temp += ((gyro_roll + acc_roll) * call_cycle); // 融合计算

    // 应用机械中值偏移
    imu_data.pitch = angle_pitch_temp + machine_angle;
    imu_data.roll = angle_roll_temp;

    // yaw角不由此算法提供
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

    // 加速度计: 原始值 → g（未校准）
    float ax_raw = imu660rb_acc_transition(imu_data.acc_x);
    float ay_raw = imu660rb_acc_transition(imu_data.acc_y);
    float az_raw = imu660rb_acc_transition(imu_data.acc_z);

    // 应用加速度计校准参数
    float ax, ay, az;
    apply_acc_calibration(ax_raw, ay_raw, az_raw, &ax, &ay, &az);

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
 * @brief       计算姿态角（双算法测试模式）
 * @param       void
 * @return      void
 * @note        同时运行互补滤波和EKF算法，结果分别存储在 imu_dual_data 中，
 *              并通过 imu_host_send() 发送到上位机（默认空实现，用户可覆盖）
 * @example     imu_calculate_attitude_dual();
 ********************************************************************************************************************/
void imu_calculate_attitude_dual(void)
{
    // ----- 1) 互补滤波（计算但不覆盖全局 imu_data，结果写入 imu_dual_data.comp_*） -----
    // 使用与 imu_calculate_attitude_complementary 相同的处理流程
    float ax_raw = imu660rb_acc_transition(imu_data.acc_x);
    float ay_raw = imu660rb_acc_transition(imu_data.acc_y);
    float az_raw = imu660rb_acc_transition(imu_data.acc_z);

    float ax, ay, az;
    apply_acc_calibration(ax_raw, ay_raw, az_raw, &ax, &ay, &az);

    // 局部临时角度，避免修改全局互补滤波状态（但保留全局积分使互补滤波能连续运行）
    // 为了保持互补滤波连续性，这里仍然使用全局角度临时变量进行积分
    float gyro_pitch = imu_data.gyro_y * gyro_ration;
    float acc_pitch = (ax - angle_pitch_temp) * acc_ration;
    angle_pitch_temp += ((gyro_pitch + acc_pitch) * call_cycle);

    float gyro_roll = imu_data.gyro_x * gyro_ration;
    float acc_roll = (ay - angle_roll_temp) * acc_ration;
    angle_roll_temp += ((gyro_roll + acc_roll) * call_cycle);

    imu_dual_data.comp_pitch = angle_pitch_temp + machine_angle;
    imu_dual_data.comp_roll = angle_roll_temp;
    imu_dual_data.comp_yaw = 0.0f; // 互补滤波不输出yaw

    // ----- 2) EKF （复用现有EKF更新与读取） -----
    // EKF需要使用物理单位（弧度）+ 加速度（g）
    float gx = imu660rb_gyro_transition(imu_data.gyro_x) * DEG_TO_RAD;
    float gy = imu660rb_gyro_transition(imu_data.gyro_y) * DEG_TO_RAD;
    float gz = imu660rb_gyro_transition(imu_data.gyro_z) * DEG_TO_RAD;

    // 应用加速度计校准参数（与EKF函数一致）
    // 注意：上面已经计算过 ax/ay/az（以 g 为单位并校准）

    // 陀螺仪死区处理（与EKF保持一致）
    const float gyro_deadzone = 0.008f;
    if (fabsf(gx) < gyro_deadzone)
        gx = 0;
    if (fabsf(gy) < gyro_deadzone)
        gy = 0;
    if (fabsf(gz) < gyro_deadzone)
        gz = 0;

    // 调用EKF更新（内部会更新 QEKF_INS）
    IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az);

    // 从EKF读取结果（度），保持与 imu_calculate_attitude_ekf 相同的方向约定
    imu_dual_data.ekf_pitch = -QEKF_INS.Pitch + machine_angle;
    imu_dual_data.ekf_roll = -QEKF_INS.Roll;
    imu_dual_data.ekf_yaw = QEKF_INS.Yaw;

    // ----- 3) 将两套结果格式化并发送到上位机（如果用户实现了 imu_host_send） -----
    char buf[128];
    // 格式: COMP,roll,pitch;EKF,roll,pitch,yaw\n
    int len = snprintf(buf, sizeof(buf), "COMP,%.2f,%.2f;EKF,%.2f,%.2f,%.2f\n",
                       imu_dual_data.comp_roll, imu_dual_data.comp_pitch,
                       imu_dual_data.ekf_roll, imu_dual_data.ekf_pitch, imu_dual_data.ekf_yaw);
    if (len > 0)
    {
        imu_host_send(buf);
    }
}

// 默认的上位机发送函数（弱实现）。工程中可自行实现同名函数将数据通过串口/USB等发送。
// 提供一个空实现以保证链接通过。
void imu_host_send(const char *msg)
{
    (void)msg; // 默认不输出，用户可在工程中实现覆盖此函数
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
    else if (imu_algorithm_select == 2)
    {
        // 双算法测试模式：同时运行互补滤波和EKF，并发送结果到上位机
        imu_calculate_attitude_dual();
        // 为兼容性，保留 imu_data 中的 EKF 或互补滤波默认输出可按需选择；
        // 此处不修改 imu_data（上层可直接读取 imu_dual_data）
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
