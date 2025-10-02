/*********************************************************************
 * 文件: menu_clean.c
 * 精简的菜单系统实现（用于 IPS114 显示屏）
 * 从 IPS200 菜单系统改编，已移除所有业务逻辑，仅保留通用框架
 ********************************************************************/

#include "menu.h"
#include "zf_common_headfile.h"
#include "pid.h"
/**************** 函数声明 ****************/
void ips_clear(void);
void show_string(uint16 x, uint16 y, const char *str);
void show_int(uint16 x, uint16 y, int value, uint8 num);
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);

/**************** 全局变量 ****************/

Page *Now_Menu = NULL;         // 当前菜单页面指针
static uint8 need_refresh = 1; // 屏幕刷新标志

/**************** 按键扫描相关函数 ****************/

/**
 * @brief 初始化按键（GPIO 配置）
 */
void Key_Init(void)
{
    // 初始化4个按键为上拉输入模式
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/**
 * @brief 按键扫描（含长按检测）
 * @return 按键代码（KEY_NONE, KEY_UP, KEY_DOWN, KEY_OK, KEY_BACK）
 * @note 长按会自动连续触发，用于实现快速翻页或快速调参
 */
uint8 Key_Scan(void)
{
    static uint8 press_cnt = 0;       // 按键按下计数器
    static uint8 last_key = KEY_NONE; // 上次检测到的按键
    uint8 key_value = KEY_NONE;
    uint8 current_key = KEY_NONE;

    // 检测当前哪个按键被按下（不延时，在中断中已有固定20ms周期）
    if (gpio_get_level(KEY1_PIN) == GPIO_LOW)
        current_key = KEY_UP;
    else if (gpio_get_level(KEY2_PIN) == GPIO_LOW)
        current_key = KEY_DOWN;
    else if (gpio_get_level(KEY3_PIN) == GPIO_LOW)
        current_key = KEY_OK;
    else if (gpio_get_level(KEY4_PIN) == GPIO_LOW)
        current_key = KEY_BACK;

    // 按键处理逻辑
    if (current_key != KEY_NONE) // 有按键按下
    {
        if (current_key == last_key) // 同一个按键持续按下
        {
            press_cnt++;

            // 第1次检测到：立即触发（短按响应）
            if (press_cnt == 1)
            {
                key_value = current_key;
            }
            // 达到长按阈值后：开始连续触发
            else if (press_cnt >= LONG_PRESS_CNT)
            {
                // 每隔REPEAT_INTERVAL次触发一次（实现自动连续操作）
                if ((press_cnt - LONG_PRESS_CNT) % REPEAT_INTERVAL == 0)
                {
                    key_value = current_key;
                }
            }
        }
        else // 不同的按键
        {
            press_cnt = 1;
            last_key = current_key;
            key_value = current_key; // 立即触发新按键
        }
    }
    else // 没有按键按下
    {
        press_cnt = 0;
        last_key = KEY_NONE;
    }

    return key_value;
}

/**************** 菜单参数与页面配置区 ****************/
/*
 * 使用说明：
 * 1. 先定义参数变量
 * 2. 再定义对应的步进数组（调参用）
 * 3. 然后配置 CustomData 数组
 * 4. 最后定义 Page 页面结构
 *
 * 注意：
 * - Debug 监控页面的参数不需要步进数组（只读显示）
 * - 步进数组的长度可以灵活配置（1~N个步进值）
 */

//============================================================
// 1. 示例菜单 - Example Menu
//============================================================
// 1.1 参数变量定义
float example_float1 = 1.5f;
float example_float2 = 2.0f;
int16 example_int16_1 = 100;
int16 example_int16_2 = 200;
uint32 example_uint32 = 1000;

// 1.2 步进数组定义
float float1_step[] = {0.01f, 0.1f, 1.0f}; // 3 个步进档位
float float2_step[] = {0.1f, 0.5f};        // 2 个步进档位
int16 int16_1_step[] = {1, 10, 100};       // 3 个步进档位
int16 int16_2_step[] = {5, 50};            // 2 个步进档位
uint32 uint32_step[] = {1, 10, 100, 1000}; // 4 个步进档位

// 1.3 参数配置数组
CustomData example_data[] = {
    // 变量地址          类型              显示名称         步进数组        步进数  索引  整数位  小数位
    {&example_float1, data_float_show, "Float Param 1", float1_step, 3, 0, 4, 4},
    {&example_float2, data_float_show, "Float Param 2", float2_step, 2, 0, 4, 4},
    {&example_int16_1, data_int16_show, "Int16 Param 1", int16_1_step, 3, 0, 4, 4},
    {&example_int16_2, data_int16_show, "Int16 Param 2", int16_2_step, 2, 0, 4, 4},
    {&example_uint32, data_uint32_show, "Uint32 Param", uint32_step, 4, 0, 4, 4},
};

// 1.4 页面定义
Page page_example = {
    .name = "Example Menu",
    .data = example_data,
    .len = 5,
    .stage = Menu,
    .back = NULL, // Menu_Init 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
};

//============================================================
// 2. 电机控制菜单 - Motor Control Menu
//============================================================
// 2.1 参数变量定义
float motor_speed = 0.0f;
int16 motor_duty = 0;

// 2.2 步进数组定义
float motor_speed_step[] = {0.1f, 1.0f, 10.0f}; // 3 个步进档位
int16 motor_duty_step[] = {10, 100, 1000};      // 3 个步进档位

// 2.3 参数配置数组
CustomData motor_data[] = {
    // 变量地址          类型              显示名称         步进数组           步进数  索引  整数位  小数位
    {&motor_speed, data_float_show, "Motor Speed", motor_speed_step, 3, 0, 4, 4},
    {&motor_duty, data_int16_show, "Motor Duty", motor_duty_step, 3, 0, 4, 4},
};

// 2.4 页面定义
Page page_motor = {
    .name = "Motor Control",
    .data = motor_data,
    .len = 2,
    .stage = Menu,
    .back = NULL, // Menu_Init 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
};

//============================================================
// 3. PID参数菜单 - PID Parameters
//============================================================
// 说明：调整三个串级PID控制器的参数
// - 角速度环PID (gyro_pid)  : 最内环，直接控制电机输出
// - 角度环PID   (angle_pid) : 中间环，输出作为角速度环目标
// - 速度环PID   (speed_pid) : 最外环，输出作为角度环目标偏移

// 3.1 步进数组定义（统一的PID参数步进）
float pid_kp_step[] = {0.01f, 0.1f, 1.0f};      // Kp步进
float pid_ki_step[] = {0.01f, 0.1f, 1.0f};      // Ki步进
float pid_kd_step[] = {0.001f, 0.01f, 0.1f};    // Kd步进
float pid_limit_step[] = {1.0f, 10.0f, 100.0f}; // 限幅步进

// 3.2 参数配置数组
CustomData pid_data[] = {
    // -------- 角速度环PID --------
    // 变量地址                 类型              显示名称         步进数组        步进数  索引  整数位  小数位
    {&gyro_pid.kp, data_float_show, "Gyro Kp", pid_kp_step, 3, 0, 4, 3},
    {&gyro_pid.ki, data_float_show, "Gyro Ki", pid_ki_step, 3, 0, 4, 3},
    {&gyro_pid.kd, data_float_show, "Gyro Kd", pid_kd_step, 3, 0, 4, 4},
    {&gyro_pid.max_integral, data_float_show, "Gyro MaxI", pid_limit_step, 3, 0, 5, 1},
    {&gyro_pid.max_output, data_float_show, "Gyro MaxOut", pid_limit_step, 3, 0, 5, 1},

    // -------- 角度环PID --------
    {&angle_pid.kp, data_float_show, "Angle Kp", pid_kp_step, 3, 0, 4, 3},
    {&angle_pid.ki, data_float_show, "Angle Ki", pid_ki_step, 3, 0, 4, 3},
    {&angle_pid.kd, data_float_show, "Angle Kd", pid_kd_step, 3, 0, 4, 4},
    {&angle_pid.max_integral, data_float_show, "Angle MaxI", pid_limit_step, 3, 0, 5, 1},
    {&angle_pid.max_output, data_float_show, "Angle MaxOut", pid_limit_step, 3, 0, 5, 1},

    // -------- 速度环PID --------
    {&speed_pid.kp, data_float_show, "Speed Kp", pid_kp_step, 3, 0, 4, 3},
    {&speed_pid.ki, data_float_show, "Speed Ki", pid_ki_step, 3, 0, 4, 3},
    {&speed_pid.kd, data_float_show, "Speed Kd", pid_kd_step, 3, 0, 4, 4},
    {&speed_pid.max_integral, data_float_show, "Speed MaxI", pid_limit_step, 3, 0, 5, 1},
    {&speed_pid.max_output, data_float_show, "Speed MaxOut", pid_limit_step, 3, 0, 5, 1},
};

// 3.3 页面定义
Page page_pid = {
    .name = "PID Params",
    .data = pid_data,
    .len = 15, // 3个PID控制器 × 5个参数 = 15
    .stage = Menu,
    .back = NULL, // Menu_Init 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
};

//============================================================
// 4. 调试监控页面 - Debug Monitor (只读显示)
//============================================================
// 4.1 参数配置数组（复用现有变量，无需步进数组）
CustomData debug_data[] = {
    // 变量地址          类型              显示名称      步进数组  步进数  索引  整数位  小数位
    {&motor_speed, data_float_show, "Speed", NULL, 0, 0, 3, 2},
    {&motor_duty, data_int16_show, "Duty", NULL, 0, 0, 3, 2},
    {&example_float1, data_float_show, "Param", NULL, 0, 0, 3, 2},
    // 可在此处添加更多传感器或状态数据
};

// 4.2 页面定义
Page page_debug = {
    .name = "Debug Monitor",
    .data = debug_data,
    .len = 3,
    .stage = Menu,
    .back = NULL, // Menu_Init 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
};

//============================================================
// 5. 主菜单 - Main Menu
//============================================================
// 说明：
// - 上电自动从 Flash 加载参数
// - 退出调参模式时自动保存到 Flash
//============================================================
Page main_page = {
    .name = "Main Menu",
    .data = NULL, // 主菜单无参数
    .len = 4,     // 4 个子菜单
    .stage = Menu,
    .back = NULL, // 主菜单无父菜单
    .enter = {&page_example, &page_motor, &page_pid, &page_debug},
    .content = {NULL},
    .order = 0,
};

//============================================================
// 6. 添加新菜单的模板（取消注释后使用）
//============================================================
/*
// 6.1 参数变量定义
float   new_param1 = 0.0f;
int16   new_param2 = 0;

// 6.2 步进数组定义
float   new_param1_step[] = {0.1f, 1.0f};
int16   new_param2_step[] = {1, 10, 100};

// 6.3 参数配置数组
CustomData new_menu_data[] = {
    {&new_param1,  data_float_show, "New Param 1", new_param1_step, 2, 0, 4, 4},
    {&new_param2,  data_int16_show, "New Param 2", new_param2_step, 3, 0, 4, 4},
};

// 6.4 页面定义
Page page_new_menu = {
    .name   = "New Menu",
    .data   = new_menu_data,
    .len    = 2,
    .stage  = Menu,
    .back   = NULL,     // Menu_Init 中设置
    .enter  = {NULL},
    .content = {NULL},
    .order  = 0,
};

// 6.5 别忘了在 Menu_Init() 中添加：
// page_new_menu.back = &main_page;

// 6.6 别忘了在 main_page.enter[] 中添加并更新 main_page.len：
// .enter = {&page_example, &page_motor, &page_pid, &page_debug, &page_new_menu},
// .len = 5;
*/

/**************** 参数保存/加载功能实现 ****************/
#include "param_save.h"

/**************** 菜单控制函数 ****************/

/**
 * @brief 初始化菜单系统
 */
void Menu_Init(void)
{
    // 先初始化按键
    Key_Init();

    // 设置页面之间的关系（父页面指针）
    page_example.back = &main_page;
    page_motor.back = &main_page;
    page_pid.back = &main_page;
    page_debug.back = &main_page;

    // 设置当前菜单为主页面
    Now_Menu = &main_page;
}

/**
 * @brief 确认/进入按键操作
 */
void Menu_Enter(void)
{
    if (Now_Menu->stage == Menu)
    {
        // 如果存在子菜单则进入子菜单
        if (Now_Menu->enter[Now_Menu->order] != NULL)
        {
            ips_clear(); // 立即清屏
            Now_Menu = Now_Menu->enter[Now_Menu->order];
            Now_Menu->order = 0;
            need_refresh = 1; // 进入子菜单时请求刷新屏幕
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 在调参模式下，按确认键切换步进大小
        Now_Menu->data[Now_Menu->order].step_num++;
        if (Now_Menu->data[Now_Menu->order].step_num >= Now_Menu->data[Now_Menu->order].step_len)
        {
            Now_Menu->data[Now_Menu->order].step_num = 0;
        }
    }
}

/**
 * @brief 返回键操作
 */
void Menu_Back(void)
{
    if (Now_Menu->stage == Menu)
    {
        // 返回到父菜单
        if (Now_Menu->back != NULL)
        {
            ips_clear(); // 立即清屏
            Now_Menu = Now_Menu->back;
            need_refresh = 1; // 返回父菜单时请求刷新屏幕
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 退出调参模式，返回到 Menu 阶段
        // 自动保存参数到 Flash
        Param_Save_All();

        ips_clear(); // 立即清屏
        Now_Menu->stage = Menu;
        need_refresh = 1; // 退出调参模式时请求刷新屏幕
    }
    else if (Now_Menu->stage == Funtion)
    {
        // 退出功能页面，返回到 Menu 阶段
        ips_clear(); // 立即清屏
        Now_Menu->stage = Menu;
        need_refresh = 1; // 退出功能模式时请求刷新屏幕
    }
}

/**
 * @brief 上键操作
 */
void Menu_Up(void)
{
    if (Now_Menu->stage == Menu)
    {
        // 在菜单中向上移动选择
        if (Now_Menu->order > 0)
        {
            Now_Menu->order--;
        }
        else
        {
            // 循环到末尾
            if (Now_Menu->len > 0)
            {
                Now_Menu->order = Now_Menu->len - 1;
            }
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 在调参模式下，增加参数值
        switch (Now_Menu->data[Now_Menu->order].type)
        {
        case data_float_show:
            *(float *)Now_Menu->data[Now_Menu->order].address +=
                ((float *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_int16_show:
            *(int16 *)Now_Menu->data[Now_Menu->order].address +=
                ((int16 *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_int_show:
            *(int *)Now_Menu->data[Now_Menu->order].address +=
                ((int *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_uint32_show:
            *(uint32 *)Now_Menu->data[Now_Menu->order].address +=
                ((uint32 *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;
        }
    }
}

/**
 * @brief 下键操作
 */
void Menu_Down(void)
{
    if (Now_Menu->stage == Menu)
    {
        // 在菜单中向下移动选择
        if (Now_Menu->len > 0 && Now_Menu->order < Now_Menu->len - 1)
        {
            Now_Menu->order++;
        }
        else
        {
            // 循环到开头
            Now_Menu->order = 0;
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 在调参模式下，减少参数值
        switch (Now_Menu->data[Now_Menu->order].type)
        {
        case data_float_show:
            *(float *)Now_Menu->data[Now_Menu->order].address -=
                ((float *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_int16_show:
            *(int16 *)Now_Menu->data[Now_Menu->order].address -=
                ((int16 *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_int_show:
            *(int *)Now_Menu->data[Now_Menu->order].address -=
                ((int *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;

        case data_uint32_show:
            *(uint32 *)Now_Menu->data[Now_Menu->order].address -=
                ((uint32 *)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
            break;
        }
    }
}

/**
 * @brief 左键操作（进入参数调节模式）
 */
void Menu_Left(void)
{
    if (Now_Menu->stage == Menu && Now_Menu->len > 0)
    {
        // 进入参数调节模式
        ips_clear(); // 立即清屏
        Now_Menu->stage = Change;
        need_refresh = 1; // 进入调参模式时请求刷新屏幕
    }
}

/**
 * @brief 右键操作（执行功能）
 */
void Menu_Right(void)
{
    if (Now_Menu->stage == Menu)
    {
        // 如果配置了功能函数则执行
        if (Now_Menu->content.function != NULL)
        {
            Now_Menu->stage = Funtion;
            Now_Menu->content.function();
        }
    }
}

/**************** 显示函数封装 ****************/

/**
 * @brief 显示字符串（封装）
 */
void show_string(uint16 x, uint16 y, const char *str)
{
    ips114_show_string(x * 8, y * 8, str);
}

/**
 * @brief 显示整数
 */
void show_int(uint16 x, uint16 y, int32 value, uint8 num)
{
    ips114_show_int(x * 8, y * 8, value, num);
}

/**
 * @brief 显示浮点数
 */
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum)
{
    ips114_show_float(x * 8, y * 8, value, num, pointnum);
}

/**
 * @brief 清屏
 */
void ips_clear(void)
{
    ips114_clear();
}

/**
 * @brief 清除单行文本（用空格覆盖）
 */
void ips_clear_line(uint16 x, uint16 y, uint8 len)
{
    for (uint8 i = 0; i < len; i++)
    {
        show_string(x + i, y, " ");
    }
}

/**
 * @brief 菜单显示（主渲染函数）
 */
void Menu_Show(void)
{
    static uint8 last_stage = 0xFF; // 记录页面类型变化
    uint8 full_redraw = 0;          // 全量重绘标志

    // 判断是否需要刷新屏幕
    if (need_refresh)
    {
        ips_clear();
        need_refresh = 0;
        full_redraw = 1;
        last_stage = Now_Menu->stage;
    }

    // 在 Menu 模式下显示页面标题
    if (Now_Menu->stage == Menu)
    {
        show_string(1, 0, Now_Menu->name);
    }

    if (Now_Menu->stage == Menu)
    {
        // 显示菜单项（参数列表或子菜单）
        if (Now_Menu->data != NULL)
        {
            // 参数菜单
            for (uint8 i = 0; i < Now_Menu->len && i < 6; i++)
            {
                // 绘制光标或清除光标位置
                if (i == Now_Menu->order)
                {
                    show_string(0, i * 2 + 2, ">");
                }
                else
                {
                    show_string(0, i * 2 + 2, " "); // 清除其他位置的光标
                }

                // 显示参数名称
                show_string(1, i * 2 + 2, Now_Menu->data[i].name);

                // 显示参数值
                switch (Now_Menu->data[i].type)
                {
                case data_float_show:
                    show_float(15, i * 2 + 2, *(float *)Now_Menu->data[i].address,
                               Now_Menu->data[i].digit_int, Now_Menu->data[i].digit_point);
                    break;

                case data_int16_show:
                    show_int(15, i * 2 + 2, *(int16 *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int);
                    break;

                case data_int_show:
                    show_int(15, i * 2 + 2, *(int *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int);
                    break;

                case data_uint32_show:
                    show_int(15, i * 2 + 2, *(uint32 *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int);
                    break;
                }
            }
        }
        else
        {
            // 子菜单列表
            for (uint8 i = 0; i < Now_Menu->len && i < 6; i++)
            {
                // 绘制光标或清除光标位置
                if (i == Now_Menu->order)
                {
                    show_string(0, i * 2 + 2, ">");
                }
                else
                {
                    show_string(0, i * 2 + 2, " "); // 清除其他位置的光标
                }

                if (Now_Menu->enter[i] != NULL)
                {
                    show_string(1, i * 2 + 2, Now_Menu->enter[i]->name);
                }
            }
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 调参模式 - 始终全量重绘当前参数界面
        if (Now_Menu->data != NULL && Now_Menu->len > 0)
        {
            CustomData *param = &Now_Menu->data[Now_Menu->order];

            // 检查是否有step数组（判断是否可调参）
            if (param->step == NULL || param->step_len == 0)
            {
                // 无step数组，说明是只读参数，不允许进入调参模式
                // 自动返回Menu模式
                Now_Menu->stage = Menu;
                need_refresh = 1;
                return; // 直接返回，下次循环重新绘制
            }

            // 显示调参模式标题（Y=0，与菜单标题共用位置）
            show_string(1, 0, "ADJUST MODE");
            show_string(1, 0, "ADJUST MODE");

            // 显示参数名称（Y=2）
            show_string(1, 2, param->name);

            // 显示当前参数值（Y=6）
            show_string(1, 6, "Value:");
            switch (param->type)
            {
            case data_float_show:
                show_float(8, 6, *(float *)param->address,
                           param->digit_int, param->digit_point);
                break;

            case data_int16_show:
                show_int(8, 6, *(int16 *)param->address, param->digit_int);
                break;

            case data_int_show:
                show_int(8, 6, *(int *)param->address, param->digit_int);
                break;

            case data_uint32_show:
                show_int(8, 6, *(uint32 *)param->address, param->digit_int);
                break;
            }

            // 显示当前步进大小（Y=8）
            show_string(1, 8, "Step: ");
            switch (param->type)
            {
            case data_float_show:
                show_float(7, 8, ((float *)param->step)[param->step_num], 3, 3);
                break;

            case data_int16_show:
                show_int(7, 8, ((int16 *)param->step)[param->step_num], 5);
                break;

            case data_int_show:
                show_int(7, 8, ((int *)param->step)[param->step_num], 5);
                break;

            case data_uint32_show:
                show_int(7, 8, ((uint32 *)param->step)[param->step_num], 5);
                break;
            }

            // 显示操作提示（Y=10）
            show_string(1, 10, "--------------------");
            show_string(1, 12, "UP/DN:Value");
            show_string(1, 14, "OK:Step  BACK:Exit");
        }
    }
}

/**
 * @brief 按键操作分发函数
 * 将物理按键代码映射为菜单操作
 */
void Key_operation(uint8 key)
{
    switch (key)
    {
    case KEY_UP: // KEY1 - 上
        Menu_Up();
        break;

    case KEY_DOWN: // KEY2 - 下
        Menu_Down();
        break;

    case KEY_OK: // KEY3 - 确认/进入
        if (Now_Menu->stage == Menu)
        {
            // 如果当前菜单有参数数据，则进入调参模式
            if (Now_Menu->data != NULL && Now_Menu->len > 0)
            {
                Menu_Left(); // 进入参数调节模式
            }
            // 否则进入子菜单
            else
            {
                Menu_Enter(); // 进入子菜单
            }
        }
        else if (Now_Menu->stage == Change)
        {
            Menu_Enter(); // 切换步进大小
        }
        break;

    case KEY_BACK: // KEY4 - 返回
        Menu_Back();
        break;

    default:
        break;
    }
}

/**
 * @brief 菜单更新函数（主循环调用）
 * @note 集成按键扫描、操作处理和菜单显示
 */
void menu_update(void)
{
    // 扫描按键
    uint8 key = Key_Scan();

    // 如果有按键按下，处理按键操作
    if (key != KEY_NONE)
    {
        Key_operation(key);
    }

    // 显示菜单
    Menu_Show();
}
