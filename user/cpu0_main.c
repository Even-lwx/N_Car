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
 * 文件名称          cpu0_main
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
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// **************************** 代码区域 ****************************
void all_init(void)
{
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口
    // 从 Flash 加载保存的参数（必须在 menu_init 之前）
    Param_Load_All();
    Menu_Init(); // 初始化菜单系统
    // 蜂鸣器初始化（最早初始化，用于系统启动提示和保护报警）
    buzzer_init(); // 初始化蜂鸣器

    // 初始化各个模块
    ips114_init();              // 初始化IPS114液晶屏
    imu_init();                 // 初始化IMU陀螺仪
    servo_init();               // 初始化舵机
    motor_init();               // 初始化电机和编码器
    pid_init();                 // 初始化简化PID控制系统
    pit_ms_init(CCU60_CH0, 1);  // 1ms定时器用于PID控制
    pit_ms_init(CCU60_CH1, 20); // 20ms定时器用于按键扫描（长按检测）
    buzzer_beep(1, 100, 100);
}
int core0_main(void)
{
    all_init(); // 初始化所有模块

    cpu_wait_event_ready(); // 等待所有核心初始化完毕

    while (1)
    {

        // 显示菜单
        menu_update();
        //momentum_wheel_control(3000);
        //drive_wheel_control(3000);
        // 减少CPU占用
        system_delay_ms(10);
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
