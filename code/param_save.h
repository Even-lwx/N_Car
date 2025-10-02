/*********************************************************************
  File: param_save.h
  参数保存/加载功能头文件 - 通用版
  自动遍历菜单系统，保存所有参数到Flash
  添加新参数无需修改此文件！
*********************************************************************/

#ifndef _PARAM_SAVE_H_
#define _PARAM_SAVE_H_

#include "zf_common_typedef.h"
#include "menu.h"

/**************** Flash存储配置 ****************/
#define PARAM_FLASH_SECTOR      0           // 使用扇区0
#define PARAM_FLASH_PAGE        0           // 使用页0
#define MAX_PARAM_COUNT         100         // 最大支持100个参数

/**************** 参数存储结构 ****************/
typedef struct {
    uint32 param_count;                 // 实际参数数量
    uint32 data[MAX_PARAM_COUNT];       // 参数数据（统一按uint32存储）
} FlashParamData;

/**************** 函数声明 ****************/

/**
 * @brief 自动保存所有菜单参数到Flash
 * @return 1=成功, 0=失败
 */
uint8 Param_Save_All(void);

/**
 * @brief 自动从Flash加载所有菜单参数
 * @return 1=成功, 0=失败
 */
uint8 Param_Load_All(void);

/**
 * @brief 恢复出厂设置（需手动实现默认值）
 * @return 1=成功, 0=失败
 */
uint8 Param_Reset_Default(void);

#endif // _PARAM_SAVE_H_
