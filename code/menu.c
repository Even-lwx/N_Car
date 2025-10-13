/*********************************************************************
 * 文件: menu.c
 * 菜单系统内核实现（用于 IPS114 显示屏）
 * 说明：本文件为菜单系统内核，提供通用框架功能
 *       用户配置请修改 menu_config.c 文件
 ********************************************************************/

#include "menu.h"
#include "zf_common_headfile.h"

/**************** 函数声明 ****************/
void ips_clear(void);
void show_string(uint16 x, uint16 y, const char *str);
void show_int(uint16 x, uint16 y, int value, uint8 num);
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);

/**************** 页面前置声明 ****************/
extern Page main_page;
extern Page page_cargo; // Cargo运行模式页面（用于特殊退出处理）

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

/**************** 菜单控制函数 ****************/

/**
 * @brief 初始化菜单系统
 */
void Menu_Init(void)
{
    // 先初始化按键
    Key_Init();

    // 调用用户菜单配置初始化（设置页面关系）
    Menu_Config_Init();

    // 上电自动从Flash加载参数
    Param_Load_All();

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
            Now_Menu = Now_Menu->enter[Now_Menu->order];
            Now_Menu->order = 0;
            Now_Menu->scroll_offset = 0; // 重置滚动偏移

            ips_clear();
            need_refresh = 1;

            // 如果进入的是功能页面，立即执行功能函数
            if (Now_Menu->stage == Funtion && Now_Menu->content.function != NULL)
            {
                // 执行功能函数（显示界面、初始化等）
                Now_Menu->content.function();

                // 不再进入死循环，而是直接返回
                // 由主循环根据 enable 标志位来决定是否刷新菜单
                // 如果是 cargo 模式，enable 会被设置为 true，主循环会跳过菜单刷新
            }
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
            Now_Menu = Now_Menu->back;
            Now_Menu->scroll_offset = 0;  // 重置滚动偏移量，避免显示错乱

            ips_clear();
            need_refresh = 1;
        }
        else
        {
            // 如果已经在主菜单（back为NULL），按返回键让光标指向Cargo项
            // 这是一个快捷选择功能，还需要按OK键才能真正进入
            if (Now_Menu == &main_page)
            {
                // 将光标移动到第0项（Cargo）
                Now_Menu->order = 0;
                Now_Menu->scroll_offset = 0;
                // 不需要刷新整个屏幕，光标位置自动更新
            }
        }
    }
    else if (Now_Menu->stage == Change)
    {
        // 退出调参模式，返回到 Menu 阶段
        // 自动保存参数到 Flash
        Param_Save_All();

        Now_Menu->stage = Menu;

        ips_clear();
        need_refresh = 1;
    }
    else if (Now_Menu->stage == Funtion)
    {
        // 退出功能页面，返回到父菜单

        // cargo模式的特殊退出处理
        if (Now_Menu == &page_cargo)
        {
            // 禁用PID控制并停止电机
            enable = false;
            momentum_wheel_control(0);
            drive_wheel_control(0);
            motor_reset_protection();
        }

        if (Now_Menu->back != NULL)
        {
            Now_Menu = Now_Menu->back;
            Now_Menu->order = 0;          // 重置光标到第一项
            Now_Menu->scroll_offset = 0;  // 重置滚动偏移量

            ips_clear();
            need_refresh = 1;
        }
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

            // 如果当前选中项在可见区域上方，向上滚动
            if (Now_Menu->order < Now_Menu->scroll_offset)
            {
                Now_Menu->scroll_offset = Now_Menu->order;
                need_refresh = 1; // 需要刷新屏幕
            }
        }
        else
        {
            // 循环到末尾
            if (Now_Menu->len > 0)
            {
                Now_Menu->order = Now_Menu->len - 1;

// 滚动到显示最后一项
#define MAX_VISIBLE_ITEMS 6
                if (Now_Menu->len > MAX_VISIBLE_ITEMS)
                {
                    Now_Menu->scroll_offset = Now_Menu->len - MAX_VISIBLE_ITEMS;
                }
                else
                {
                    Now_Menu->scroll_offset = 0;
                }
                need_refresh = 1; // 需要刷新屏幕
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
            // 如果是舵机角度参数，实时更新舵机
            if (Now_Menu->data[Now_Menu->order].address == &servo_motor_duty)
            {
                servo_set_angle(servo_motor_duty);
            }
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

// 如果当前选中项在可见区域下方，向下滚动
#define MAX_VISIBLE_ITEMS 6
            if (Now_Menu->order >= Now_Menu->scroll_offset + MAX_VISIBLE_ITEMS)
            {
                Now_Menu->scroll_offset = Now_Menu->order - MAX_VISIBLE_ITEMS + 1;
                need_refresh = 1; // 需要刷新屏幕
            }
        }
        else
        {
            // 循环到开头
            Now_Menu->order = 0;
            Now_Menu->scroll_offset = 0;
            need_refresh = 1; // 需要刷新屏幕
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
            // 如果是舵机角度参数，实时更新舵机
            if (Now_Menu->data[Now_Menu->order].address == &servo_motor_duty)
            {
                servo_set_angle(servo_motor_duty);
            }
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
 * @brief 显示彩色字符串
 */
void show_string_color(uint16 x, uint16 y, const char *str, uint16 color)
{
    ips114_set_color(color, RGB565_BLACK);
    ips114_show_string(x * 8, y * 8, str);
}

/**
 * @brief 显示彩色整数
 */
void show_int_color(uint16 x, uint16 y, int32 value, uint8 num, uint16 color)
{
    ips114_set_color(color, RGB565_BLACK);
    ips114_show_int(x * 8, y * 8, value, num);
}

/**
 * @brief 显示彩色浮点数
 */
void show_float_color(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum, uint16 color)
{
    ips114_set_color(color, RGB565_BLACK);
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
    // 安全检查：确保Now_Menu有效
    if (Now_Menu == NULL)
    {
        Now_Menu = &main_page;
        need_refresh = 1;
        return;
    }

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

    // 在 Menu 模式下显示页面标题（红色）
    if (Now_Menu->stage == Menu)
    {
        if (Now_Menu->name != NULL)
        {
            show_string_color(1, 0, Now_Menu->name, RGB565_RED);
            ips114_set_color(RGB565_WHITE, RGB565_BLACK); // 恢复默认白色
        }
    }

    if (Now_Menu->stage == Menu)
    {
        // 显示菜单项（参数列表或子菜单）
        if (Now_Menu->data != NULL)
        {
// 参数菜单 - 支持滚动显示
#define MAX_VISIBLE_ITEMS 6 // 屏幕最多显示6个菜单项

            uint8 visible_count = 0; // 可见项计数
            for (uint8 i = Now_Menu->scroll_offset; i < Now_Menu->len && visible_count < MAX_VISIBLE_ITEMS; i++)
            {
                uint8 display_line = visible_count * 2 + 2; // 计算显示行号

                // 判断是否为选中行
                uint8 is_selected = (i == Now_Menu->order);

                // 绘制光标
                if (is_selected)
                {
                    show_string_color(0, display_line, ">", RGB565_GREEN);
                }
                else
                {
                    show_string(0, display_line, " "); // 清除其他位置的光标
                }

                // 显示参数名称（选中时为绿色高亮，未选中为白色）
                show_string_color(1, display_line, Now_Menu->data[i].name,
                                  is_selected ? RGB565_GREEN : RGB565_WHITE);

                // 显示参数值（选中时为绿色高亮，未选中为白色）
                uint16 value_color = is_selected ? RGB565_GREEN : RGB565_WHITE;
                switch (Now_Menu->data[i].type)
                {
                case data_float_show:
                    show_float_color(15, display_line, *(float *)Now_Menu->data[i].address,
                               Now_Menu->data[i].digit_int, Now_Menu->data[i].digit_point, value_color);
                    break;

                case data_int16_show:
                    show_int_color(15, display_line, *(int16 *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int, value_color);
                    break;

                case data_int_show:
                    show_int_color(15, display_line, *(int *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int, value_color);
                    break;

                case data_uint32_show:
                    show_int_color(15, display_line, *(uint32 *)Now_Menu->data[i].address,
                             Now_Menu->data[i].digit_int, value_color);
                    break;
                }

                // 恢复默认白色
                ips114_set_color(RGB565_WHITE, RGB565_BLACK);

                visible_count++;
            }
        }
        else
        {
// 子菜单列表 - 支持滚动显示
#define MAX_VISIBLE_ITEMS 6 // 屏幕最多显示6个菜单项

            uint8 visible_count = 0; // 可见项计数
            for (uint8 i = Now_Menu->scroll_offset; i < Now_Menu->len && visible_count < MAX_VISIBLE_ITEMS; i++)
            {
                uint8 display_line = visible_count * 2 + 2; // 计算显示行号

                // 判断是否为选中行
                uint8 is_selected = (i == Now_Menu->order);

                // 绘制光标
                if (is_selected)
                {
                    show_string_color(0, display_line, ">", RGB565_GREEN);
                }
                else
                {
                    show_string(0, display_line, " "); // 清除其他位置的光标
                }

                if (Now_Menu->enter[i] != NULL)
                {
                    // 选中时为绿色高亮，未选中为白色
                    show_string_color(1, display_line, Now_Menu->enter[i]->name,
                                      is_selected ? RGB565_GREEN : RGB565_WHITE);
                }

                // 恢复默认白色
                ips114_set_color(RGB565_WHITE, RGB565_BLACK);

                visible_count++;
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

            // 显示调参模式标题（Y=0，与菜单标题共用位置）（红色）
            show_string_color(1, 0, "ADJUST MODE", RGB565_RED);

            // 显示参数名称（Y=2）（白色）
            show_string_color(1, 2, param->name, RGB565_WHITE);

            // 显示当前参数值（Y=6）
            show_string(1, 6, "Value:");
            switch (param->type)
            {
            case data_float_show:
                show_float_color(8, 6, *(float *)param->address,
                           param->digit_int, param->digit_point, RGB565_WHITE);
                break;

            case data_int16_show:
                show_int_color(8, 6, *(int16 *)param->address, param->digit_int, RGB565_WHITE);
                break;

            case data_int_show:
                show_int_color(8, 6, *(int *)param->address, param->digit_int, RGB565_WHITE);
                break;

            case data_uint32_show:
                show_int_color(8, 6, *(uint32 *)param->address, param->digit_int, RGB565_WHITE);
                break;
            }

            // 显示当前步进大小（Y=8）
            show_string(1, 8, "Step: ");
            switch (param->type)
            {
            case data_float_show:
                show_float_color(7, 8, ((float *)param->step)[param->step_num], 3, 3, RGB565_WHITE);
                break;

            case data_int16_show:
                show_int_color(7, 8, ((int16 *)param->step)[param->step_num], 5, RGB565_WHITE);
                break;

            case data_int_show:
                show_int_color(7, 8, ((int *)param->step)[param->step_num], 5, RGB565_WHITE);
                break;

            case data_uint32_show:
                show_int_color(7, 8, ((uint32 *)param->step)[param->step_num], 5, RGB565_WHITE);
                break;
            }

            // 显示操作提示（Y=10）
            show_string(1, 10, "--------------------");
            show_string(1, 12, "UP/DN:Value");
            show_string(1, 14, "OK:Step  BACK:Exit");

            // 恢复默认白色
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
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
    // 安全检查
    if (Now_Menu == NULL)
    {
        Now_Menu = &main_page;
        ips_clear();
        need_refresh = 1;
    }

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
