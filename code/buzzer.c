/*********************************************************************
 * 文件: buzzer.c
 * 蜂鸣器控制库实现
 * 功能：提供蜂鸣器的初始化、鸣叫、更新等功能
 *********************************************************************/

#include "buzzer.h"
#include "zf_common_headfile.h"

// *************************** 内部变量 ***************************
static bool buzzer_active = false;          // 蜂鸣器是否激活（正在自动鸣叫）
static uint8 buzzer_beep_count = 0;         // 当前已完成的鸣叫次数
static uint8 buzzer_beep_total = 0;         // 总共需要鸣叫的次数（0表示连续鸣叫）
static uint16 buzzer_beep_duration = 100;   // 每次鸣叫持续时间（毫秒）
static uint16 buzzer_beep_interval = 100;   // 鸣叫间隔时间（毫秒）
static uint32 buzzer_last_toggle_time = 0; // 上次状态切换的时间戳
static bool buzzer_state = false;           // 当前蜂鸣器物理状态（true=开，false=关）
static uint32 buzzer_time_ms = 0;           // 蜂鸣器内部时间计数器（毫秒）

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
    buzzer_active = false;
    buzzer_state = false;
    buzzer_beep_count = 0;
    buzzer_beep_total = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器更新函数（需要定时调用，建议20ms周期）
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_update(); // 在20ms定时器中断中调用
// 备注信息     负责处理蜂鸣器的自动鸣叫逻辑
//-------------------------------------------------------------------------------------------------------------------
void buzzer_update(void)
{
    buzzer_time_ms += 20; // 更新内部时间（按20ms周期递增）

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
            buzzer_state = false;
            buzzer_last_toggle_time = buzzer_time_ms;
            buzzer_beep_count++;

            // 检查是否完成所有鸣叫
            if (buzzer_beep_total > 0 && buzzer_beep_count >= buzzer_beep_total)
            {
                buzzer_active = false;
            }
        }
    }
    else // 蜂鸣器关闭中（间隔期）
    {
        // 连续鸣叫模式或还有剩余次数
        bool should_continue = (buzzer_beep_total == 0) || (buzzer_beep_count < buzzer_beep_total);

        if (elapsed >= buzzer_beep_interval && should_continue)
        {
            // 开启蜂鸣器
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            buzzer_state = true;
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
// 备注信息     调用后蜂鸣器会自动按照指定参数鸣叫，需要配合buzzer_update()使用
//-------------------------------------------------------------------------------------------------------------------
void buzzer_beep(uint8 count, uint16 duration_ms, uint16 interval_ms)
{
    buzzer_active = true;
    buzzer_beep_total = count;
    buzzer_beep_count = 0;
    buzzer_beep_duration = duration_ms;
    buzzer_beep_interval = interval_ms;
    buzzer_state = false;
    buzzer_last_toggle_time = buzzer_time_ms;

    // 立即开始第一次鸣叫
    if (count > 0 || count == 0)
    {
        gpio_set_level(BUZZER_PIN, GPIO_HIGH);
        buzzer_state = true;
        buzzer_last_toggle_time = buzzer_time_ms;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止蜂鸣器鸣叫
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_stop();
// 备注信息     立即停止当前的鸣叫，蜂鸣器关闭
//-------------------------------------------------------------------------------------------------------------------
void buzzer_stop(void)
{
    buzzer_active = false;
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    buzzer_state = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动开启蜂鸣器
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_on();
// 备注信息     直接开启蜂鸣器，不使用自动鸣叫逻辑（会停止自动鸣叫）
//-------------------------------------------------------------------------------------------------------------------
void buzzer_on(void)
{
    buzzer_active = false; // 停止自动鸣叫
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);
    buzzer_state = true;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动关闭蜂鸣器
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_off();
// 备注信息     直接关闭蜂鸣器
//-------------------------------------------------------------------------------------------------------------------
void buzzer_off(void)
{
    buzzer_active = false; // 停止自动鸣叫
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    buzzer_state = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     翻转蜂鸣器状态
// 参数说明     void
// 返回参数     void
// 使用示例     buzzer_toggle();
// 备注信息     翻转蜂鸣器当前状态（开变关，关变开），会停止自动鸣叫
//-------------------------------------------------------------------------------------------------------------------
void buzzer_toggle(void)
{
    buzzer_active = false; // 停止自动鸣叫
    gpio_toggle_level(BUZZER_PIN);
    buzzer_state = !buzzer_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询蜂鸣器是否正在鸣叫
// 参数说明     void
// 返回参数     bool: true=正在鸣叫, false=已停止
// 使用示例     if(buzzer_is_active()) { ... }
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
bool buzzer_is_active(void)
{
    return buzzer_active;
}
