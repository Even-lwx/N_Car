/*********************************************************************
 * 文件: buzzer.h
 * 蜂鸣器控制库头文件
 * 功能：提供蜂鸣器的初始化、鸣叫、更新等功能
 *********************************************************************/

#ifndef BUZZER_H
#define BUZZER_H

#include "zf_common_headfile.h"

// *************************** 蜂鸣器引脚定义 ***************************
#define BUZZER_PIN (P33_10) // 蜂鸣器引脚，根据实际硬件修改

// *************************** 蜂鸣器模式枚举 ***************************
typedef enum {
    BUZZER_MODE_ONCE = 0,      // 单次鸣叫模式
    BUZZER_MODE_REPEAT = 1,    // 重复鸣叫模式
    BUZZER_MODE_CONTINUOUS = 2 // 连续鸣叫模式
} buzzer_mode_t;

// *************************** 函数声明 ***************************

/**
 * @brief 蜂鸣器初始化
 * @note 初始化蜂鸣器引脚为输出模式，默认关闭
 */
void buzzer_init(void);

/**
 * @brief 蜂鸣器更新函数（需要定时调用，建议20ms周期）
 * @note 在定时器中断中调用，负责处理蜂鸣器的自动鸣叫逻辑
 */
void buzzer_update(void);

/**
 * @brief 启动蜂鸣器鸣叫
 * @param count 鸣叫次数（0表示连续鸣叫）
 * @param duration_ms 每次鸣叫持续时间（毫秒）
 * @param interval_ms 鸣叫间隔时间（毫秒）
 * @note 调用后蜂鸣器会自动按照指定参数鸣叫，需要配合buzzer_update()使用
 * @example buzzer_beep(3, 100, 100);  // 响3声，每声100ms，间隔100ms
 */
void buzzer_beep(uint8 count, uint16 duration_ms, uint16 interval_ms);

/**
 * @brief 停止蜂鸣器鸣叫
 * @note 立即停止当前的鸣叫，蜂鸣器关闭
 */
void buzzer_stop(void);

/**
 * @brief 手动开启蜂鸣器
 * @note 直接开启蜂鸣器，不使用自动鸣叫逻辑
 */
void buzzer_on(void);

/**
 * @brief 手动关闭蜂鸣器
 * @note 直接关闭蜂鸣器
 */
void buzzer_off(void);

/**
 * @brief 翻转蜂鸣器状态
 * @note 翻转蜂鸣器当前状态（开变关，关变开）
 */
void buzzer_toggle(void);

/**
 * @brief 查询蜂鸣器是否正在鸣叫
 * @return true=正在鸣叫, false=已停止
 */
bool buzzer_is_active(void);

#endif // BUZZER_H
