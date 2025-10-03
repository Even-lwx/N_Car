#/************************************************************************
 * 版权所有 (C), 2024 , ASTA ASC.
 * 文件名:        menu.h
 * 作者: CANG_HAI  版本: 2.00 (已清理)   日期: 2025/10/02
 * 描述:          精简后的菜单框架头文件
 *                适配 IPS114 显示屏（240x135 像素）
 *                已移除所有业务逻辑，仅保留框架代码
 *************************************************************************/
#ifndef _MENU_H_
#define _MENU_H_

#include "zf_common_typedef.h"
#include "zf_device_ips114.h"
#include "zf_driver_gpio.h"

/**************** GPIO 引脚定义 ****************/
#define KEY1_PIN P20_6 // KEY1 - 上（UP）
#define KEY2_PIN P20_7 // KEY2 - 下（DOWN）
#define KEY3_PIN P11_2 // KEY3 - 确认（OK / Enter）
#define KEY4_PIN P11_3 // KEY4 - 返回（BACK / Return）

/**************** 按键代码定义 ****************/
#define KEY_NONE 0
#define KEY_UP 1
#define KEY_DOWN 2
#define KEY_OK 3
#define KEY_BACK 4

/**************** 长按配置 ****************/
#define LONG_PRESS_CNT 30 // 长按阈值：连续检测15次认为长按（约300ms）
#define REPEAT_INTERVAL 3 // 长按后每3次循环触发一次（约60ms间隔）

/**************** 字体配置 ****************/
#define FONT_W 8 // 字符宽度
#define FONT_H 8 // 字符高度

/**************** 枚举类型 ****************/

/**
 * @brief 页面类型枚举
 */
typedef enum
{
    Menu,    // 菜单页面
    Change,  // 参数调整页面
    Funtion, // 功能执行页面
    Debug,   // 调试/监视页面（实时传感器或状态数据显示）
} Page_Type;

/**
 * @brief 数据类型枚举
 */
typedef enum
{
    data_float_show = 0,  // 浮点数
    data_int16_show = 1,  // 16位有符号整数
    data_uint32_show = 2, // 32位无符号整数
    data_int_show = 3,    // 一般整数
} Data_Type;

/**************** 结构体定义 ****************/

/**
 * @brief 自定义参数数据结构
 */
typedef struct
{
    void *address;     // 数据地址
    Data_Type type;    // 数据类型
    char name[20];     // 参数名称（字符串）
    void *step;        // 步进数组指针
    uint8 step_len;    // 步进数组长度
    uint8 step_num;    // 当前步进索引
    uint8 digit_int;   // 整数位数显示
    uint8 digit_point; // 小数位数显示
} CustomData;

/**
 * @brief 页面结构体（前置声明）
 */
typedef struct Page Page;

/**
 * @brief 页面结构体定义
 */
struct Page
{
    char name[20];    // 页面名称
    CustomData *data; // 指向参数数组的指针
    uint8 len;        // 参数数组长度
    Page_Type stage;  // 页面类型
    Page *back;       // 返回的父页面指针
    Page *enter[4];   // 子页面指针（最多4个）
    union
    {
        void (*function)(void); // 功能函数指针
    } content;
    uint8 order;        // 当前选中索引
    uint8 scroll_offset; // 滚动偏移量（首个显示项的索引）
};

/**************** 全局变量声明 ****************/
extern Page *Now_Menu; // 当前菜单指针

extern int16 add_mode[5];    // 通用步进数组（示例）
extern float add_float[5];   // 浮点类型步进数组
extern int16 add_int16[5];   // int16 步进数组
extern uint32 add_uint32[5]; // uint32 步进数组
extern int add_int[5];       // int 步进数组

/**************** 函数声明 ****************/

// 按键扫描相关函数
void Key_Init(void);  // 按键初始化
uint8 Key_Scan(void); // 按键扫描（含消抖）

// 菜单操作相关函数
void Menu_Init(void);          // 菜单初始化
void Menu_Enter(void);         // 确认/进入按键处理
void Menu_Back(void);          // 返回按键处理
void Menu_Up(void);            // 向上选择
void Menu_Down(void);          // 向下选择
void Menu_Left(void);          // 左键处理（进入调整/减少）
void Menu_Right(void);         // 右键处理（执行/增加）
void Menu_Show(void);          // 菜单渲染显示
void Key_operation(uint8 key); // 按键分发处理函数
void menu_update(void);        // 菜单更新函数（主循环调用）

// 显示封装函数
void show_string(uint16 x, uint16 y, const char *str);
void show_int(uint16 x, uint16 y, int32 value, uint8 num);
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);
void show_uint(uint16 x, uint16 y, uint32 value, uint8 num);

// 工具/辅助函数
void Screen_Clear(void);                                 // 清屏
void Screen_SetColor(uint16 pen_color, uint16 bg_color); // 设置前景/背景色

#endif // _MENU_H_
