/*********************************************************************
  File: param_save.c
  参数保存/加载功能实现 - 通用版
  自动遍历菜单系统，保存/加载所有参数
  添加新菜单参数无需修改此文件！
*********************************************************************/

#include "zf_common_headfile.h"
#include "param_save.h"
#include "menu.h"

/**************** 外部变量声明 ****************/
extern Page *Now_Menu;
extern Page main_page;
extern Page page_servo;
extern Page page_gyro_pid;
extern Page page_angle_pid;
extern Page page_speed_pid;
extern Page page_drive_speed_pid;
extern Page page_imu_params;
extern Page page_delayed_stop; // 延迟停车参数页面
extern Page page_turn_comp;    // 转弯补偿参数页面
// 添加新页面时在这里声明

/**************** 内部变量 ****************/
// 需要保存的页面列表（在这里添加所有包含参数的页面）
static Page *param_pages[] = {
    &page_servo,
    &page_gyro_pid,
    &page_angle_pid,
    &page_speed_pid,
    &page_drive_speed_pid,
    &page_imu_params,
    &page_delayed_stop, // 延迟停车参数
    &page_turn_comp,    // 转弯补偿参数
    // 添加新页面时在这里添加指针
    NULL // 结束标记
};

/**************** 内部函数 ****************/

/**
 * @brief 简单的字符串哈希函数
 * @param str 字符串
 * @return 哈希值
 */
static uint32 string_hash(const char *str)
{
    uint32 hash = 5381;
    while (*str)
    {
        hash = ((hash << 5) + hash) + (*str++); // hash * 33 + c
    }
    return hash;
}

/**
 * @brief 遍历所有页面并收集参数数据
 * @param items 输出缓冲区（ParamItem数组）
 * @return 参数总数
 */
static uint32 Collect_All_Params(ParamItem *items)
{
    uint32 param_index = 0;

    // 遍历所有需要保存的页面
    for (uint8 page_idx = 0; param_pages[page_idx] != NULL; page_idx++)
    {
        Page *page = param_pages[page_idx];

        // 如果页面有参数数据
        if (page->data != NULL && page->len > 0)
        {
            // 遍历该页面的所有参数
            for (uint8 i = 0; i < page->len; i++)
            {
                if (param_index >= MAX_PARAM_COUNT)
                {
                    // 超出最大参数数量
                    return param_index;
                }

                CustomData *param = &page->data[i];

                // 生成唯一标识：页面名称 + 参数名称
                char unique_name[50];
                snprintf(unique_name, sizeof(unique_name), "%s.%s", page->name, param->name);
                items[param_index].name_hash = string_hash(unique_name);

                // 根据类型读取参数值并转换为uint32存储
                switch (param->type)
                {
                case data_float_show:
                {
                    float value = *(float *)param->address;
                    // 将float按位复制到uint32
                    items[param_index].value = *(uint32 *)&value;
                    break;
                }

                case data_int16_show:
                {
                    int16 value = *(int16 *)param->address;
                    // 扩展到uint32
                    items[param_index].value = (uint32)value;
                    break;
                }

                case data_int_show:
                {
                    int value = *(int *)param->address;
                    items[param_index].value = (uint32)value;
                    break;
                }

                case data_uint32_show:
                {
                    uint32 value = *(uint32 *)param->address;
                    items[param_index].value = value;
                    break;
                }
                }

                param_index++;
            }
        }
    }

    return param_index;
}

/**
 * @brief 将参数数据写回到变量地址（通过哈希匹配）
 * @param items 参数项数组
 * @param param_count 参数数量
 * @return 1=成功, 0=失败
 */
static uint8 Restore_All_Params(ParamItem *items, uint32 param_count)
{
    // 遍历所有需要保存的页面
    for (uint8 page_idx = 0; param_pages[page_idx] != NULL; page_idx++)
    {
        Page *page = param_pages[page_idx];

        // 如果页面有参数数据
        if (page->data != NULL && page->len > 0)
        {
            // 遍历该页面的所有参数
            for (uint8 i = 0; i < page->len; i++)
            {
                CustomData *param = &page->data[i];

                // 生成唯一标识
                char unique_name[50];
                snprintf(unique_name, sizeof(unique_name), "%s.%s", page->name, param->name);
                uint32 target_hash = string_hash(unique_name);

                // 在Flash数据中查找匹配的参数
                for (uint32 j = 0; j < param_count; j++)
                {
                    if (items[j].name_hash == target_hash)
                    {
                        // 找到匹配项，根据类型将uint32数据还原到变量
                        switch (param->type)
                        {
                        case data_float_show:
                        {
                            uint32 raw_data = items[j].value;
                            // 将uint32按位复制回float
                            *(float *)param->address = *(float *)&raw_data;
                            break;
                        }

                        case data_int16_show:
                        {
                            *(int16 *)param->address = (int16)items[j].value;
                            break;
                        }

                        case data_int_show:
                        {
                            *(int *)param->address = (int)items[j].value;
                            break;
                        }

                        case data_uint32_show:
                        {
                            *(uint32 *)param->address = items[j].value;
                            break;
                        }
                        }
                        break; // 找到后跳出内层循环
                    }
                }
            }
        }
    }

    return 1;
}

/**
 * @brief 自动保存所有菜单参数到Flash
 * @return 1=成功, 0=失败
 */
uint8 Param_Save_All(void)
{
    FlashParamData save_data;
    uint32 flash_buffer[sizeof(FlashParamData) / sizeof(uint32) + 1];

    // 1. 设置魔术字
    save_data.magic = PARAM_MAGIC_NUMBER;

    // 2. 自动收集所有参数
    save_data.param_count = Collect_All_Params(save_data.items);

    if (save_data.param_count == 0)
    {
        return 0; // 没有参数可保存
    }

    // 3. 复制到Flash缓冲区
    memcpy(flash_buffer, &save_data, sizeof(FlashParamData));

    // 4. 擦除Flash页
    flash_erase_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);

    // 5. 写入Flash
    flash_write_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE, flash_buffer,
                     sizeof(FlashParamData) / sizeof(uint32) + 1);

    // 静默保存，无屏幕提示

    return 1;
}

/**
 * @brief 从Flash自动加载所有菜单参数
 * @return 1=成功, 0=失败（Flash为空或版本不匹配）
 */
uint8 Param_Load_All(void)
{
    FlashParamData load_data;
    uint32 flash_buffer[sizeof(FlashParamData) / sizeof(uint32) + 1];

    // 1. 检查Flash是否有数据
    if (flash_check(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE) == 0)
    {
        // Flash为空，返回失败
        return 0;
    }

    // 2. 从Flash读取数据
    flash_read_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE, flash_buffer,
                    sizeof(FlashParamData) / sizeof(uint32) + 1);

    // 3. 复制到结构体
    memcpy(&load_data, flash_buffer, sizeof(FlashParamData));

    // 4. 校验魔术字
    if (load_data.magic != PARAM_MAGIC_NUMBER)
    {
        // 数据无效
        return 0;
    }

    // 5. 自动恢复所有参数（通过哈希匹配）
    Restore_All_Params(load_data.items, load_data.param_count);

    // 静默加载，无屏幕提示

    return 1;
}

/**
 * @brief 恢复出厂设置（擦除Flash，重新保存当前值）
 * @return 1=成功, 0=失败
 * @note 本函数会擦除Flash并保存当前参数值作为默认值
 *       如果需要恢复特定默认值，请在调用前手动设置参数
 */
uint8 Param_Reset_Default(void)
{
    // 擦除Flash
    flash_erase_page(PARAM_FLASH_SECTOR, PARAM_FLASH_PAGE);

    // 保存当前值到Flash（作为默认值）
    return Param_Save_All();
}
