/*********************************************************************
 * 文件: image.h
 * 图像处理头文件
 * 说明：包含赛道图像处理相关的宏定义、变量声明和函数声明
 *       提供边界提取、元素识别（十字路口、环岛、坡道等）功能
 ********************************************************************/

#ifndef _IMAGE_H
#define _IMAGE_H

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

//============================================================
// 宏定义
//============================================================

// 图像尺寸宏定义（使用MT9V03X摄像头的分辨率）
#define IMAGE_WIDTH  MT9V03X_W  // 图像宽度 188
#define IMAGE_HEIGHT MT9V03X_H  // 图像高度 120

// 转弯标准范围宏定义
#define TURN_STANDARD_START turn_start  // 转弯检测起始行
#define TURN_STANDARD_END turn_end      // 转弯检测结束行

//============================================================
// 全局变量声明
//============================================================

// -------------------- 图像处理基本参数 --------------------
extern int threshold;           // 全局二值化阈值
extern uint8 image_proess;      // 图像处理标志
extern int turn_start;          // 转弯检测起始行
extern int turn_end;            // 转弯检测结束行

// -------------------- 图像数据 --------------------
extern uint8 image_copy[IMAGE_HEIGHT][IMAGE_WIDTH];  // 图像副本数组
extern volatile int Left_Line[MT9V03X_H];            // 左边界数组
extern volatile int Right_Line[MT9V03X_H];           // 右边界数组
extern const uint8 Road_Standard_Wide[MT9V03X_H];    // 赛道标准宽度数组

// -------------------- 赛道元素检测标志 --------------------
extern volatile int Cross_Flag;         // 十字路口检测标志
extern volatile int Island_State;       // 环岛状态标志
extern volatile int Ramp_Flag;          // 坡道检测标志
extern int Ramp_offset;                 // 坡道偏移量
extern volatile int circle_flag;        // 环岛标志（0=无环岛，非0=有环岛）
extern volatile int right_circle_flag;  // 右环岛标志（0/1/2/3不同状态）
extern volatile int Zebra_Stripes_Flag; // 斑马线标志位

// -------------------- 边界搜索相关 --------------------
extern volatile int Search_Stop_Line;           // 边界搜索停止行
extern int Longest_White_Column_Left[2];        // 左侧最长白列：[0]长度，[1]列号
extern int Longest_White_Column_Right[2];       // 右侧最长白列：[0]长度，[1]列号

// -------------------- 编码器相关变量 --------------------
extern int Encoder_Left;  // 左编码器累计值
extern int encoder_sum;   // 编码器总和

//============================================================
// 函数声明
//============================================================

// -------------------- 赛道元素检测函数 --------------------
/**
 * @brief 最长白列检测
 * @note 用于检测起跑线或停止线
 */
void Longest_White_Column(void);

/**
 * @brief 十字路口检测
 * @note 检测并设置Cross_Flag标志
 */
void Cross_Detect(void);

/**
 * @brief 斑马线检测
 * @return 检测结果（0=无斑马线，非0=有斑马线）
 */
int Zebra_Detect(void);

/**
 * @brief 坡道检测
 * @note 检测并设置Ramp_Flag标志
 */
void Ramp_Detect(void);

// -------------------- 边界处理函数 --------------------
/**
 * @brief 显示赛道边界
 * @note 在屏幕上绘制左右边界线
 */
void Show_Boundry(void);

/**
 * @brief 向上搜索边界点
 * @param start 起始行
 * @param end 结束行
 */
void Find_Up_Point(int start, int end);

/**
 * @brief 向下搜索边界点
 * @param start 起始行
 * @param end 结束行
 */
void Find_Down_Point(int start, int end);

/**
 * @brief 延长左边界
 * @param start 起始行
 * @param end 结束行
 */
void Lengthen_Left_Boundry(int start, int end);

/**
 * @brief 延长右边界
 * @param start 起始行
 * @param end 结束行
 */
void Lengthen_Right_Boundry(int start, int end);

/**
 * @brief 添加左边界线段
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 */
void Left_Add_Line(int x1, int y1, int x2, int y2);

/**
 * @brief 添加右边界线段
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 */
void Right_Add_Line(int x1, int y1, int x2, int y2);

/**
 * @brief 绘制直线
 * @param startX 起点X坐标
 * @param startY 起点Y坐标
 * @param endX 终点X坐标
 * @param endY 终点Y坐标
 */
void Draw_Line(int startX, int startY, int endX, int endY);

// -------------------- 图像分析函数 --------------------
/**
 * @brief 计算图像中线偏差的平均值
 * @param start_point 起始行（从图像底部算起）
 * @param end_point 结束行
 * @return 中线偏差平均值（正值表示偏右，负值表示偏左）
 * @note 用于转向PID控制的P环输入
 */
float err_sum_average(uint8 start_point, uint8 end_point);

/**
 * @brief 检测图像是否出界
 * @param binaryImage 二值化图像数组
 * @return 出界标志（0=未出界，1=出界）
 */
uint8 image_out_of_bounds(uint8 binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]);

// -------------------- 总图像处理函数 --------------------
/**
 * @brief 图像处理主函数
 * @note 集成边界提取和元素识别的主函数
 *       包括：边界搜索、元素检测、边界延长等完整处理流程
 */
void image_process(void);

#endif
