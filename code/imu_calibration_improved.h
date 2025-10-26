/*********************************************************************************************************************
 * 文件名称: imu_calibration_improved.h
 * 功能说明: 改进版加速度计校准算法（带静止检测、高频采样、方向覆盖检测）
 * 作    者: N_Car项目组
 * 日    期: 2025-01-25
 * 备    注: 独立于原imu.c的校准模块，可选择性集成
 ********************************************************************************************************************/

#ifndef IMU_CALIBRATION_IMPROVED_H
#define IMU_CALIBRATION_IMPROVED_H

#include "zf_common_typedef.h"

// *************************** 配置宏定义 ***************************

// *** 采样配置 ***
#define ACC_CAL_IMPROVED_MAX_SAMPLES  500   // 最大样本组数（推荐300-500）
#define ACC_CAL_IMPROVED_MIN_SAMPLES  100   // 最小样本组数（不建议低于100）

// *** 静止检测配置 ***
#define ACC_CAL_STILLNESS_THRESHOLD   0.05f // 静止检测阈值（g），加速度变化小于此值认为静止
#define ACC_CAL_STILLNESS_SAMPLES     10    // 静止段内连续采样数（每段采集10个样本取平均）
#define ACC_CAL_MIN_STABLE_TIME_MS    500   // 每个方向最小稳定时间（毫秒）

// *** 方向覆盖检测配置 ***
#define ACC_CAL_DIRECTION_TOLERANCE   0.3f  // 方向判断容差（g），主轴需接近1.0±0.3g
#define ACC_CAL_MIN_COVERAGE_COUNT    4     // 最少方向覆盖数（建议至少4个方向）

// *************************** 结构体定义 ***************************

/**
 * @brief 加速度计校准参数结构体
 */
typedef struct
{
    float bias_x;       // X轴零偏（g）
    float bias_y;       // Y轴零偏（g）
    float bias_z;       // Z轴零偏（g）
    float scale_x;      // X轴缩放因子
    float scale_y;      // Y轴缩放因子
    float scale_z;      // Z轴缩放因子
    uint8 calibrated;   // 校准完成标志 (0=未校准, 1=已校准)
} acc_calibration_params_t;

/**
 * @brief 方向覆盖统计结构体
 */
typedef struct
{
    uint8 covered_px;   // +X方向已覆盖 (0=未覆盖, 1=已覆盖)
    uint8 covered_nx;   // -X方向已覆盖
    uint8 covered_py;   // +Y方向已覆盖
    uint8 covered_ny;   // -Y方向已覆盖
    uint8 covered_pz;   // +Z方向已覆盖
    uint8 covered_nz;   // -Z方向已覆盖
    uint8 count;        // 已覆盖方向总数
} acc_direction_coverage_t;

/**
 * @brief 手动校准状态结构体
 * @note  用于菜单系统实时显示校准进度
 */
typedef struct
{
    uint8 is_active;                  // 校准是否进行中 (0=未启动, 1=进行中)
    uint16 sample_count;              // 已采集样本数
    uint16 target_samples;            // 目标样本数
    float HTH_data[36];               // H'H累加器 (6×6矩阵)
    float HTb_data[6];                // H'b累加器 (6×1向量)
    acc_direction_coverage_t coverage; // 方向覆盖统计
} manual_calibration_state_t;

// *************************** 全局变量声明 ***************************

/**
 * @brief 加速度计校准参数全局实例
 * @note  外部模块可通过此变量获取校准结果
 */
extern acc_calibration_params_t g_acc_calib_params;

/**
 * @brief 手动校准状态全局实例
 * @note  外部模块（如菜单系统）可通过此变量访问校准进度信息
 *        主要用于显示：已采集样本数、方向覆盖情况等
 */
extern manual_calibration_state_t g_manual_calib_state;

// *************************** 函数声明 ***************************

/**
 * @brief       启动手动校准模式
 * @return      uint8           校准结果（1=成功启动, 0=失败）
 * @note        手动确认模式：
 *              1. 调用此函数启动校准
 *              2. 将IMU旋转到一个方向并保持静止
 *              3. 按下确认按钮调用 imu_calibrate_acc_confirm_sample()
 *              4. 重复步骤2-3直到覆盖6个方向
 *              5. 调用 imu_calibrate_acc_manual_finish() 完成校准
 * @example     uint8 result = imu_calibrate_acc_manual();
 */
uint8 imu_calibrate_acc_manual(void);

/**
 * @brief       手动校准的按钮确认回调函数
 * @param       void
 * @return      void
 * @note        在按钮中断或按键扫描中调用此函数
 *              系统会采集当前方向的数据（10个样本快速平均）
 * @example     在按键中断中调用: imu_calibrate_acc_confirm_sample();
 */
void imu_calibrate_acc_confirm_sample(void);

/**
 * @brief       完成手动校准并计算参数
 * @param       void
 * @return      uint8           校准结果（1=成功, 0=失败）
 * @note        采集足够样本后调用此函数完成校准
 *              系统会求解椭球拟合参数并输出结果
 * @example     uint8 result = imu_calibrate_acc_manual_finish();
 */
uint8 imu_calibrate_acc_manual_finish(void);

/**
 * @brief       重置加速度计校准参数为默认值
 * @return      void
 * @note        恢复到未校准状态（bias=0, scale=1）
 * @example     imu_reset_acc_calibration();
 */
void imu_reset_acc_calibration(void);

/**
 * @brief       启动局部手动校准模式（Z轴向上范围）
 * @return      uint8           校准结果（1=成功启动, 0=失败）
 * @note        局部校准模式（适用于直立平衡车等Z轴向上场景）：
 *              1. 调用此函数启动校准
 *              2. 在Z轴向上的范围内（±30度倾角范围）旋转IMU到不同姿态并保持静止
 *              3. 按下确认按钮调用 imu_calibrate_acc_confirm_sample()
 *              4. 重复步骤2-3，建议采集15-20个样本（覆盖不同前后左右倾角）
 *              5. 调用 imu_calibrate_acc_manual_finish_local() 完成校准
 * @example     uint8 result = imu_calibrate_acc_manual_local();
 */
uint8 imu_calibrate_acc_manual_local(void);

/**
 * @brief       完成局部手动校准并计算参数（Z轴向上范围）
 * @param       void
 * @return      uint8           校准结果（1=成功, 0=失败）
 * @note        采集足够样本后调用此函数完成局部校准
 *              使用简化的2自由度椭球拟合（假设Z轴接近1g）
 *              只校准X、Y轴的bias和scale，Z轴使用简化模型
 * @example     uint8 result = imu_calibrate_acc_manual_finish_local();
 */
uint8 imu_calibrate_acc_manual_finish_local(void);

#endif // IMU_CALIBRATION_IMPROVED_H
