/*********************************************************************
 * 文件: menu_config.h
 * 用户菜单配置头文件
 * 说明：用户只需修改此文件来添加/修改菜单，无需修改 menu.c 内核
 ********************************************************************/

#ifndef _MENU_CONFIG_H_
#define _MENU_CONFIG_H_

#include "menu.h"

/**************** 用户菜单配置说明 ****************/
/*
 * 添加新菜单的步骤：
 *
 * 1. 定义参数变量（如果需要）
 *    例如：float my_param = 1.0f;
 *
 * 2. 定义步进数组（用于调参）
 *    例如：float my_param_step[] = {0.1f, 1.0f, 10.0f};
 *
 * 3. 定义参数配置数组 CustomData[]
 *    例如：CustomData my_data[] = {
 *              {&my_param, data_float_show, "My Param", my_param_step, 3, 0, 4, 2},
 *          };
 *
 * 4. 定义页面 Page
 *    例如：Page page_my_menu = {
 *              .name = "My Menu",
 *              .data = my_data,
 *              .len = 1,
 *              .stage = Menu,
 *              .back = NULL,  // 在 Menu_Config_Init() 中设置
 *              .enter = {NULL},
 *              .content = {NULL},
 *              .order = 0,
 *              .scroll_offset = 0,
 *          };
 *
 * 5. 在 Menu_Config_Init() 中设置页面关系
 * 6. 在主菜单的 .enter[] 中添加页面指针
 *
 * 注意：
 * - 参数地址（第1个参数）不能修改，否则Flash数据会丢失
 * - 显示名称（第3个参数）可以随意修改
 * - 功能页面的 .stage = Funtion, .content.function = 函数指针
 */

/**************** 外部引用的主菜单声明 ****************/
extern Page main_page;

/**************** 用户菜单配置函数 ****************/
void Menu_Config_Init(void);  // 用户菜单配置初始化

#endif // _MENU_CONFIG_H_
