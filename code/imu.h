/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          imu.h
 * 功能说明          IMU惯性测量单元驱动（支持互补滤波和EKF两种姿态解算算法）
 * 公司名称          成都逐飞科技有限公司 / N_Car项目组
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 * 2025-01-09       N_Car              增加EKF算法支持，优化代码结构
 ********************************************************************************************************************/

#ifndef IMU_H
#define IMU_H

#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"

// *************************** 宏定义 ***************************
#define IMU_UPDATE_FREQ (500) // IMU数据更新频率 (Hz)，实际为2ms周期=500Hz

// *** 加速度计校准算法配置 ***
#define ACC_CAL_MAX_SAMPLES  500   // 最大样本数（推荐300-500，越多精度越高）
#define ACC_CAL_MIN_SAMPLES  100   // 最小样本数（不建议低于100）

// *** 静止检测配置 ***
#define ACC_CAL_STILLNESS_THRESHOLD  0.05f  // 静止检测阈值（g），加速度变化小于此值认为静止
#define ACC_CAL_STILLNESS_SAMPLES    10     // 静止段内连续采样数（每段采集10个样本取平均）
#define ACC_CAL_MIN_STABLE_TIME_MS   500    // 每个方向最小稳定时间（毫秒）

// *************************** 枚举类型定义 ***************************

/**
 * @brief IMU姿态解算算法选择
 */
typedef enum
{
    IMU_ALGORITHM_COMPLEMENTARY = 0, // 一阶互补滤波（原算法，快速，仅pitch）
    IMU_ALGORITHM_EKF = 1            // 扩展卡尔曼滤波（高精度，roll/pitch/yaw）
} imu_algorithm_t;

// *************************** 结构体定义 ***************************

/**
 * @brief IMU数据结构体
 * @note  存储IMU的原始数据、姿态角和状态标志
 */
typedef struct
{
    // ---------- 原始传感器数据 ----------
    int16 acc_x;  // 加速度计X轴原始数据
    int16 acc_y;  // 加速度计Y轴原始数据
    int16 acc_z;  // 加速度计Z轴原始数据
    int16 gyro_x; // 陀螺仪X轴原始数据（已校准）
    int16 gyro_y; // 陀螺仪Y轴原始数据（已校准）
    int16 gyro_z; // 陀螺仪Z轴原始数据（已校准）

    // ---------- 姿态角（单位：度） ----------
    float roll;  // 横滚角 Roll  (°)，绕X轴旋转
    float pitch; // 俯仰角 Pitch (°)，绕Y轴旋转
    float yaw;   // 偏航角 Yaw   (°)，绕Z轴旋转

    // ---------- 状态标志 ----------
    bool is_initialized; // 初始化完成标志
    bool data_ready;     // 数据就绪标志（中断中设置）
} imu_data_t;

/**
 * @brief 双算法姿态数据结构体
 * @note  用于同时存储互补滤波和EKF两种算法的解算结果
 */
typedef struct
{
    // ---------- 互补滤波结果 ----------
    float comp_roll;   // 互补滤波 - 横滚角 (°)
    float comp_pitch;  // 互补滤波 - 俯仰角 (°)
    float comp_yaw;    // 互补滤波 - 偏航角 (°)

    // ---------- EKF滤波结果 ----------
    float ekf_roll;    // EKF - 横滚角 (°)
    float ekf_pitch;   // EKF - 俯仰角 (°)
    float ekf_yaw;     // EKF - 偏航角 (°)
} imu_dual_data_t;

// *************************** 全局变量声明 ***************************

/**
 * @brief IMU数据结构体全局实例
 * @note  保存IMU的所有传感器数据和姿态角
 */
extern imu_data_t imu_data;

/**
 * @brief 双算法姿态数据全局实例
 * @note  用于测试模式，同时保存两种算法的解算结果
 */
extern imu_dual_data_t imu_dual_data;

/**
 * @brief IMU算法选择变量
 * @note  0 = 一阶互补滤波（默认），1 = EKF扩展卡尔曼滤波，2 = 双算法同时运行（测试模式）
 *        修改此值后需重新编译
 */
extern uint8 imu_algorithm_select;

// ---------- IMU校准参数（可在菜单中调整） ----------
extern uint8 gyro_ration;   // 陀螺仪权重系数（互补滤波用）
extern uint8 acc_ration;    // 加速度计权重系数（互补滤波用）
extern float call_cycle;    // 解算周期，单位：秒（互补滤波用）
extern float machine_angle; // 机械中值角度偏移（度）

// ---------- 陀螺仪零偏校准数据（原始数据） ----------
extern int16 gyro_x_offset; // X轴零偏
extern int16 gyro_y_offset; // Y轴零偏
extern int16 gyro_z_offset; // Z轴零偏

// ---------- 加速度计校准数据 ----------
extern float acc_x_bias;   // X轴零偏（单位：g）
extern float acc_y_bias;   // Y轴零偏（单位：g）
extern float acc_z_bias;   // Z轴零偏（单位：g）
extern float acc_x_scale;  // X轴缩放因子
extern float acc_y_scale;  // Y轴缩放因子
extern float acc_z_scale;  // Z轴缩放因子
extern uint8 acc_calibrated; // 加速度计校准完成标志 (0=未校准, 1=已校准)

// *************************** 函数声明 ***************************

/**
 * @brief       IMU初始化函数
 * @return      uint8           初始化结果（1=成功, 0=失败）
 * @note        初始化IMU660RB传感器，并根据选择的算法进行初始化
 * @example     uint8 result = imu_init();
 */
uint8 imu_init(void);

/**
 * @brief       IMU数据更新函数
 * @note        在定时器中断中周期性调用（2ms周期）
 * @example     imu_update();
 */
void imu_update(void);

/**
 * @brief       获取IMU原始数据
 * @note        读取加速度计和陀螺仪的原始数据
 * @example     imu_get_data();
 */
void imu_get_data(void);

/**
 * @brief       陀螺仪零偏校准函数
 * @param       sample_count    采样次数（0=使用默认值2000）
 * @note        车体静止时调用，采样多次取平均值作为零偏
 * @example     imu_calibrate_gyro(2000);
 */
void imu_calibrate_gyro(uint16 sample_count);

/**
 * @brief       加速度计校准函数（改进版 - 带静止检测）
 * @param       sample_count    目标样本数（建议200-500）
 * @param       timeout_ms      总超时时间（毫秒）
 * @return      uint8           校准结果（1=成功, 0=失败）
 * @note        **必须在静止状态下校准，会自动过滤运动状态的数据**
 *              算法特性:
 *              - 自动静止检测（加速度变化 < 0.05g）
 *              - 高频连续采样（500Hz数据更新，每个静止段采10个样本平均）
 *              - 六方向覆盖度检测（自动检查是否覆盖 ±X/±Y/±Z）
 *              - 椭球拟合算法解出6个参数（bias和scale）
 *
 *              使用方法:
 *              1. 启动校准后，缓慢旋转IMU到6个主要方向
 *              2. 在每个方向上**静止2-3秒**（系统自动检测静止并采样）
 *              3. 等待提示完成或检查方向覆盖度
 * @example     uint8 result = imu_calibrate_acc(300, 60000);
 */
uint8 imu_calibrate_acc(uint16 sample_count, uint32 timeout_ms);

/**
 * @brief       获取横滚角
 * @return      float           横滚角（度）
 * @note        仅EKF算法有效，互补滤波返回0
 * @example     float roll = imu_get_roll();
 */
float imu_get_roll(void);

/**
 * @brief       获取俯仰角
 * @return      float           俯仰角（度）
 * @note        两种算法均有效
 * @example     float pitch = imu_get_pitch();
 */
float imu_get_pitch(void);

/**
 * @brief       获取偏航角
 * @return      float           偏航角（度）
 * @note        仅EKF算法有效，互补滤波返回0
 * @example     float yaw = imu_get_yaw();
 */
float imu_get_yaw(void);

/**
 * @brief       双算法同时解算姿态角（测试模式）
 * @param       void
 * @return      void
 * @note        同时运行互补滤波和EKF两种算法，结果存储在imu_dual_data中
 *              用于对比测试两种算法的性能差异
 * @example     imu_calculate_attitude_dual();
 */
void imu_calculate_attitude_dual(void);

/**
 * @brief       上位机发送回调
 * @param       msg  要发送的字符串（以"\n"结尾）
 * @note        默认库提供空实现，工程中可实现同名函数将数据通过串口或其他方式发出
 */
void imu_host_send(const char *msg);

#endif // IMU_H
