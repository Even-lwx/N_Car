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
#define PARAM_MAGIC_NUMBER      0x12345678  // 魔术字，用于校验数据有效性

/**************** 参数项结构 ****************/
typedef struct {
    uint32 name_hash;   // 参数名称的哈希值（用于标识参数）
    uint32 value;       // 参数值
} ParamItem;

/**************** 参数存储结构 ****************/
typedef struct {
    uint32 magic;                       // 魔术字
    uint32 param_count;                 // 实际参数数量
    ParamItem items[MAX_PARAM_COUNT];   // 参数项数组
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
