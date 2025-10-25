/*********************************************************************
 * 文件: buzzer.c
 * 蜂鸣器控制库实现
 * 功能：提供蜂鸣器的初始化、鸣叫、更新等功能
 *********************************************************************/

#include "buzzer.h"
#include "zf_common_headfile.h"

// *************************** 内部变量 ***************************
static volatile uint8 buzzer_active = 0;           // 蜂鸣器是否激活（正在自动鸣叫）(0=否, 1=是)
static volatile uint8 buzzer_beep_count = 0;       // 当前已完成的鸣叫次数
static volatile uint8 buzzer_beep_total = 0;       // 总共需要鸣叫的次数（0表示连续鸣叫）
static volatile uint16 buzzer_beep_duration = 100; // 每次鸣叫持续时间（毫秒）
static volatile uint16 buzzer_beep_interval = 100; // 鸣叫间隔时间（毫秒）
static volatile uint32 buzzer_last_toggle_time = 0; // 上次状态切换的时间戳
static volatile uint8 buzzer_state = 0;            // 当前蜂鸣器物理状态（0=关，1=开）
static volatile uint32 buzzer_time_ms = 0;        // 蜂鸣器内部时间计数器（毫秒）

// 待初始化参数缓冲区（用于主程序传递参数给中断）
static volatile uint8 buzzer_pending_init = 0;     // 待初始化标志（0=无，1=有新参数）
static volatile uint8 buzzer_pending_count;        // 待设置的鸣叫次数
static volatile uint16 buzzer_pending_duration;    // 待设置的持续时间
static volatile uint16 buzzer_pending_interval;    // 待设置的间隔时间

// *************************** 函数实现 ***************************

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_init();
// 备注信息     初始化蜂鸣器引脚为输出模式，默认关闭
//-------------------------------------------------------------------------------------------------------------------
void buzzer_init(void)
{
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL); // 初始化蜂鸣器引脚为输出，默认关闭
    buzzer_active = 0;
    buzzer_state = 0;
    buzzer_beep_count = 0;
    buzzer_beep_total = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器更新函数（需要定时调用，建议20ms周期）
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_update(); // 在20ms定时器中断中调用
// 备注信息     负责处理蜂鸣器的自动鸣叫逻辑和参数初始化
//-------------------------------------------------------------------------------------------------------------------
void buzzer_update(void)
{
    buzzer_time_ms += 20; // 更新内部时间（按20ms周期递增）

    // 检查是否有待初始化的参数
    if (buzzer_pending_init)
    {
        buzzer_pending_init = 0; // 清除标志（单字节写入原子）

        // 应用新参数（在中断上下文中，天然线程安全）
        buzzer_active = 1;
        buzzer_beep_total = buzzer_pending_count;
        buzzer_beep_count = 0;
        buzzer_beep_duration = buzzer_pending_duration;
        buzzer_beep_interval = buzzer_pending_interval;

        // 立即开始第一次鸣叫（主程序已经开启了GPIO）
        buzzer_state = 1;
        buzzer_last_toggle_time = buzzer_time_ms;

        return; // 本次更新完成，下次再处理鸣叫逻辑
    }

    if (!buzzer_active)
        return;

    uint32 elapsed = buzzer_time_ms - buzzer_last_toggle_time;

    // 根据当前状态决定下一步动作
    if (buzzer_state) // 蜂鸣器正在响
    {
        if (elapsed >= buzzer_beep_duration)
        {
            // 关闭蜂鸣器
            gpio_set_level(BUZZER_PIN, GPIO_LOW);
            buzzer_state = 0;
            buzzer_last_toggle_time = buzzer_time_ms;
            buzzer_beep_count++;

            // 检查是否完成所有鸣叫
            if (buzzer_beep_total > 0 && buzzer_beep_count >= buzzer_beep_total)
            {
                buzzer_active = 0;
            }
        }
    }
    else // 蜂鸣器关闭中（间隔期）
    {
        // 连续鸣叫模式或还有剩余次数
        uint8 should_continue = (buzzer_beep_total == 0) || (buzzer_beep_count < buzzer_beep_total);

        if (elapsed >= buzzer_beep_interval && should_continue)
        {
            // 开启蜂鸣器
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            buzzer_state = 1;
            buzzer_last_toggle_time = buzzer_time_ms;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动蜂鸣器鸣叫
// 参数说明     count: 鸣叫次数（0表示连续鸣叫）
//              duration_ms: 每次鸣叫持续时间（毫秒）
//              interval_ms: 鸣叫间隔时间（毫秒）
// 返回参数     void
// 使用示例     buzzer_beep(3, 100, 100);  // 响3声，每声100ms，间隔100ms
//              buzzer_beep(0, 200, 200);  // 连续鸣叫，每次200ms，间隔200ms
// 备注信息     使用标志位延迟初始化机制，避免使用全局中断禁用
//              参数将在下一次 buzzer_update() 中断中应用（最多延迟20ms）
//-------------------------------------------------------------------------------------------------------------------
void buzzer_beep(uint8 count, uint16 duration_ms, uint16 interval_ms)
{
    // 主程序上下文：设置待初始化参数
    buzzer_pending_count = count;
    buzzer_pending_duration = duration_ms;
    buzzer_pending_interval = interval_ms;

    // 最后设置标志位（单字节写入原子，触发中断处理）
    buzzer_pending_init = 1;

    // 立即开始第一次鸣叫（提供即时反馈）
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止蜂鸣器鸣叫
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_stop();
// 备注信息     立即停止当前的鸣叫，蜂鸣器关闭
//              单字节变量写入是原子的，无需中断保护
//-------------------------------------------------------------------------------------------------------------------
void buzzer_stop(void)
{
    buzzer_active = 0;  // 单字节写入原子
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    buzzer_state = 0;   // 单字节写入原子
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动开启蜂鸣器
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_on();
// 备注信息     直接开启蜂鸣器，不使用自动鸣叫逻辑（会停止自动鸣叫）
//              单字节变量写入是原子的，无需中断保护
//-------------------------------------------------------------------------------------------------------------------
void buzzer_on(void)
{
    buzzer_active = 0;  // 停止自动鸣叫（单字节写入原子）
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);
    buzzer_state = 1;   // 单字节写入原子
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动关闭蜂鸣器
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_off();
// 备注信息     直接关闭蜂鸣器
//              单字节变量写入是原子的，无需中断保护
//-------------------------------------------------------------------------------------------------------------------
void buzzer_off(void)
{
    buzzer_active = 0;  // 停止自动鸣叫（单字节写入原子）
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    buzzer_state = 0;   // 单字节写入原子
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     翻转蜂鸣器状态
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_toggle();
// 备注信息     翻转蜂鸣器当前状态（开变关，关变开），会停止自动鸣叫
//              单字节变量写入是原子的，无需中断保护
//-------------------------------------------------------------------------------------------------------------------
void buzzer_toggle(void)
{
    buzzer_active = 0;  // 停止自动鸣叫（单字节写入原子）
    gpio_toggle_level(BUZZER_PIN);
    buzzer_state = !buzzer_state;  // 单字节写入原子
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询蜂鸣器是否正在鸣叫
// 参数说明     void
// 返回参数     uint8: 1=正在鸣叫, 0=已停止
// 使用示例     if(buzzer_is_active()) { ... }
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 buzzer_is_active(void)
{
    return buzzer_active;
}
