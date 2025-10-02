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
 * 文件名称          imu
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

#ifndef IMU_H
#define IMU_H

#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"

// *************************** IMU相关宏定义 ***************************
#define IMU_UPDATE_FREQ (100) // IMU数据更新频率 Hz

// IMU数据结构体
typedef struct
{
    // 原始数据
    int16 acc_x;  // 加速度计X轴原始数据
    int16 acc_y;  // 加速度计Y轴原始数据
    int16 acc_z;  // 加速度计Z轴原始数据
    int16 gyro_x; // 陀螺仪X轴原始数据
    int16 gyro_y; // 陀螺仪Y轴原始数据
    int16 gyro_z; // 陀螺仪Z轴原始数据

    // 去掉物理量转换，直接使用原始数据

    // 姿态角
    float roll;  // 横滚角 (°)
    float pitch; // 俯仰角 (°)
    float yaw;   // 偏航角 (°)

    // 状态标志
    bool is_initialized; // 初始化状态
    bool data_ready;     // 数据就绪标志
} imu_data_t;

// *************************** 全局变量声明 ***************************
extern imu_data_t imu_data; // IMU数据结构体

// IMU校准参数（可在菜单中调整）
extern uint8 gyro_ration; // 陀螺仪比例系数
extern uint8 acc_ration;  // 加速度计比例系数
extern float call_cycle;  // 调用周期（单位：秒）

// *************************** 函数声明 ***************************
uint8 imu_init(void);    // IMU初始化函数
void imu_update(void);   // IMU数据更新函数
void imu_get_data(void); // 获取IMU原始数据

void imu_calibrate_gyro(uint16 sample_count); // 陀螺仪校准函数
float imu_get_roll(void);                     // 获取横滚角
float imu_get_pitch(void);                    // 获取俯仰角
float imu_get_yaw(void);                      // 获取偏航角

#endif
