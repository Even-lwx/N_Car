/*********************************************************************
 * 文件: image.c
 * 图像处理实现文件
 * 说明：实现赛道图像处理的核心功能
 *       包括边界提取、元素识别（十字路口、环岛、坡道、斑马线等）
 *       使用大津法自动阈值和双边巡线算法
 ********************************************************************/

#include "image.h"
#include "zf_common_headfile.h"

//============================================================
// 宏定义
//============================================================

#define IMG_BLACK 0     // 黑色像素值
#define IMG_WHITE 255   // 白色像素值

//============================================================
// 全局变量定义
//============================================================

// -------------------- 元素检测标志 --------------------
volatile int circle_flag = 0;       // 环岛标志
volatile int right_circle_flag = 0; // 右环岛标志（0/1/2/3不同状态）
volatile int Island_State = 0;      // 环岛状态
volatile int Cross_Flag = 0;        // 十字路口检测标志
volatile int Ramp_Flag = 0;         // 坡道检测标志
int Ramp_offset;                    // 坡道偏移量
volatile int Zebra_Stripes_Flag;    // 斑马线标志位

// -------------------- 编码器相关 --------------------
int Encoder_Left = 0;  // 左编码器累计值
int encoder_sum = 0;   // 编码器总和

// -------------------- 图像数据数组 --------------------
uint8 image_copy[IMAGE_HEIGHT][IMAGE_WIDTH];  // 图像副本数组

extern const uint8 Image_Flags[][9][8];       // 外部图像标志数组
extern uint8_t binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]; // 二值化图像数组

volatile int Left_Line[MT9V03X_H];            // 左边界数组
volatile int Right_Line[MT9V03X_H];           // 右边界数组
volatile int Mid_Line[MT9V03X_H];             // 中线数组
volatile int Road_Wide[MT9V03X_H];            // 赛道宽度数组
volatile int White_Column[MT9V03X_W];         // 白列统计数组

// -------------------- 边界搜索相关 --------------------
volatile int Search_Stop_Line;                // 边界搜索停止行
volatile int Boundry_Start_Left;              // 左边界起始行
volatile int Boundry_Start_Right;             // 右边界起始行
volatile int Left_Lost_Time;                  // 左边界丢失次数
volatile int Right_Lost_Time;                 // 右边界丢失次数
volatile int Both_Lost_Time;                  // 双边界丢失次数

int Longest_White_Column_Left[2];             // 左侧最长白列：[0]长度，[1]列号
int Longest_White_Column_Right[2];            // 右侧最长白列：[0]长度，[1]列号

int Left_Lost_Flag[MT9V03X_H];                // 左边界丢失标志数组
int Right_Lost_Flag[MT9V03X_H];               // 右边界丢失标志数组

// -------------------- 拐点检测 --------------------
volatile int Left_Down_Find = 0;   // 左下拐点位置
volatile int Left_Up_Find = 0;     // 左上拐点位置
volatile int Right_Down_Find = 0;  // 右下拐点位置
volatile int Right_Up_Find = 0;    // 右上拐点位置

// -------------------- 赛道标准宽度查找表 --------------------
const uint8 Road_Standard_Wide[MT9V03X_H] =
{
    41, 42, 43, 45, 46, 47, 49, 49, 51, 53,
    53, 55, 55, 57, 58, 59, 61, 62, 63, 64,
    65, 67, 68, 69, 70, 72, 73, 74, 76, 76,
    78, 79, 80, 82, 82, 84, 86, 86, 88, 88,
    90, 91, 92, 94, 95, 96, 97, 98, 100, 100,
    102, 103, 105, 105, 107, 108, 109, 111, 112, 113,
    114, 116, 117, 118, 119, 120, 122, 123, 124, 126,
    126, 128, 129, 130, 132, 132, 134, 134, 136, 138,
    138, 140, 140, 142, 144, 144, 146, 146, 148, 149,
    150, 151, 152, 154, 155, 156, 157, 158, 159, 161,
    162, 163, 164, 165, 166, 167, 169, 170, 171, 172,
    173, 175, 175, 177, 177, 179, 180, 181, 184, 184
};

// -------------------- 图像处理基本参数 --------------------
extern volatile int Island_State;  // 环岛状态（外部引用）
extern volatile int Ramp_Flag;     // 坡道标志（外部引用）

int turn_start = 50;  // 转弯检测起始行
int turn_end = 53;    // 转弯检测结束行

int threshold;         // 全局二值化阈值
uint8 image_proess = 0; // 图像处理完成标志

//============================================================
// 函数实现
//============================================================

/**
 * @brief 最长白列检测并提取边界
 * @note 通过扫描每列的白色像素，找到最长的白色列作为起跑线或停止线的参考
 *       同时从最长白列位置开始，向左右扫描边界
 */
void Longest_White_Column()
{
    int i, j;
    int start_column = 35;
    int end_column = MT9V03X_W - 35;
    int left_border = 0, right_border = 0;

    // 初始化变量
    Longest_White_Column_Left[0] = 0;
    Longest_White_Column_Left[1] = 0;
    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = 0;
    Right_Lost_Time = 0;
    Left_Lost_Time = 0;
    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;

    // 初始化边界数组和丢失标志
    for (i = 0; i <= MT9V03X_H - 1; i++)
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W - 1;
    }
    for (i = 0; i <= MT9V03X_W - 1; i++)
    {
        White_Column[i] = 0;
    }

    // 环岛状态下调整扫描范围
    if (circle_flag)
    {
        if (right_circle_flag == 2)
        {
            start_column = 60;
            end_column = MT9V03X_W - 20;
        }
    }

    // 统计每列的白色像素数量
    for (j = start_column; j <= end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if (binaryImage[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }

    // 找左侧最长白列
    Longest_White_Column_Left[0] = 0;
    for (i = start_column; i <= end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])
        {
            Longest_White_Column_Left[0] = White_Column[i];
            Longest_White_Column_Left[1] = i;
        }
    }

    // 找右侧最长白列
    Longest_White_Column_Right[0] = 0;
    for (i = end_column; i >= Longest_White_Column_Left[1]; i--)
    {
        if (Longest_White_Column_Right[0] < White_Column[i])
        {
            Longest_White_Column_Right[0] = White_Column[i];
            Longest_White_Column_Right[1] = i;
        }
    }

    // 确定搜索停止行（取左右最长白列中的较大值）
    Search_Stop_Line = (Longest_White_Column_Left[0] > Longest_White_Column_Right[0]) ? Longest_White_Column_Left[0] : Longest_White_Column_Right[0];

    // 从最长白列位置开始，向上扫描边界
    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--)
    {
        // 扫描右边界
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j + 1] == IMG_BLACK && binaryImage[i][j + 2] == IMG_BLACK)
            {
                right_border = j;
                Right_Lost_Flag[i] = 0;
                break;
            }
            else if (j >= MT9V03X_W - 1 - 2)
            {
                right_border = j;
                Right_Lost_Flag[i] = 1;
                break;
            }
        }

        // 扫描左边界
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)
        {
            if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j - 1] == IMG_BLACK && binaryImage[i][j - 2] == IMG_BLACK)
            {
                left_border = j;
                Left_Lost_Flag[i] = 0;
                break;
            }
            else if (j <= 2)
            {
                left_border = j;
                Left_Lost_Flag[i] = 1;
                break;
            }
        }
        Left_Line[i] = left_border;
        Right_Line[i] = right_border;
    }

    // 统计边界丢失次数和边界起始行
    for (i = MT9V03X_H - 1; i >= 0; i--)
    {
        if (Left_Lost_Flag[i] == 1)
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)
            Both_Lost_Time++;
        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] != 1)
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i] = Right_Line[i] - Left_Line[i];
    }
}

/**
 * @brief 在二值化图像上显示边界
 * @note 将左边界、中线、右边界绘制到二值化图像上
 */
void Show_Boundry(void)
{
    int16 i;
    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--)
    {
        binaryImage[i][Left_Line[i] + 1] = IMG_BLACK;
        binaryImage[i][(Left_Line[i] + Right_Line[i]) >> 1] = IMG_BLACK;
        binaryImage[i][Right_Line[i] - 1] = IMG_BLACK;
    }
}

/**
 * @brief 添加左边界线段
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 * @note 使用线性插值在两点间连接左边界
 */
void Left_Add_Line(int x1, int y1, int x2, int y2)
{
    int i, max, a1, a2;
    int hx;

    // 边界检查
    if (x1 >= MT9V03X_W - 1)
        x1 = MT9V03X_W - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= MT9V03X_H - 1)
        y1 = MT9V03X_H - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= MT9V03X_W - 1)
        x2 = MT9V03X_W - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= MT9V03X_H - 1)
        y2 = MT9V03X_H - 1;
    else if (y2 <= 0)
        y2 = 0;

    // 确保a1 < a2
    a1 = y1;
    a2 = y2;
    if (a1 > a2)
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    // 线性插值连接边界
    for (i = a1; i <= a2; i++)
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        Left_Line[i] = hx;
    }
}

/**
 * @brief 添加右边界线段
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 * @note 使用线性插值在两点间连接右边界
 */
void Right_Add_Line(int x1, int y1, int x2, int y2)
{
    int i, max, a1, a2;
    int hx;

    // 边界检查
    if (x1 >= MT9V03X_W - 1)
        x1 = MT9V03X_W - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= MT9V03X_H - 1)
        y1 = MT9V03X_H - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= MT9V03X_W - 1)
        x2 = MT9V03X_W - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= MT9V03X_H - 1)
        y2 = MT9V03X_H - 1;
    else if (y2 <= 0)
        y2 = 0;

    // 确保a1 < a2
    a1 = y1;
    a2 = y2;
    if (a1 > a2)
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    // 线性插值连接边界
    for (i = a1; i <= a2; i++)
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        Right_Line[i] = hx;
    }
}

/**
 * @brief 向下搜索拐点
 * @param start 起始行
 * @param end 结束行
 * @note 检测边界突变点（下拐点），用于十字路口识别
 */
void Find_Down_Point(int start, int end)
{
    int i, t;
    Right_Down_Find = 0;
    Left_Down_Find = 0;

    // 确保start > end
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    // 边界检查
    if (start >= MT9V03X_H - 1 - 5)
        start = MT9V03X_H - 1 - 5;
    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5)
        end = 5;

    // 搜索左右下拐点
    for (i = start; i >= end; i--)
    {
        // 左下拐点检测：上方平稳，下方向内突变
        if (Left_Down_Find == 0 &&
            abs(Left_Line[i] - Left_Line[i + 1]) <= 5 &&
            abs(Left_Line[i + 1] - Left_Line[i + 2]) <= 5 &&
            abs(Left_Line[i + 2] - Left_Line[i + 3]) <= 5 &&
            (Left_Line[i] - Left_Line[i - 2]) >= 8 &&
            (Left_Line[i] - Left_Line[i - 3]) >= 15 &&
            (Left_Line[i] - Left_Line[i - 4]) >= 15)
        {
            Left_Down_Find = i;
        }

        // 右下拐点检测：上方平稳，下方向内突变
        if (Right_Down_Find == 0 &&
            abs(Right_Line[i] - Right_Line[i + 1]) <= 5 &&
            abs(Right_Line[i + 1] - Right_Line[i + 2]) <= 5 &&
            abs(Right_Line[i + 2] - Right_Line[i + 3]) <= 5 &&
            (Right_Line[i] - Right_Line[i - 2]) <= -8 &&
            (Right_Line[i] - Right_Line[i - 3]) <= -15 &&
            (Right_Line[i] - Right_Line[i - 4]) <= -15)
        {
            Right_Down_Find = i;
        }

        if (Left_Down_Find != 0 && Right_Down_Find != 0)
        {
            break;
        }
    }
}

/**
 * @brief 向上搜索拐点
 * @param start 起始行
 * @param end 结束行
 * @note 检测边界突变点（上拐点），用于十字路口识别
 */
void Find_Up_Point(int start, int end)
{
    int i, t;
    Left_Up_Find = 0;
    Right_Up_Find = 0;

    // 确保start > end
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    // 边界检查
    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5)
        end = 5;
    if (start >= MT9V03X_H - 1 - 5)
        start = MT9V03X_H - 1 - 5;

    // 搜索左右上拐点
    for (i = start; i >= end; i--)
    {
        // 左上拐点检测：下方平稳，上方向内突变
        if (Left_Up_Find == 0 &&
            abs(Left_Line[i] - Left_Line[i - 1]) <= 5 &&
            abs(Left_Line[i - 1] - Left_Line[i - 2]) <= 5 &&
            abs(Left_Line[i - 2] - Left_Line[i - 3]) <= 5 &&
            (Left_Line[i] - Left_Line[i + 2]) >= 8 &&
            (Left_Line[i] - Left_Line[i + 3]) >= 15 &&
            (Left_Line[i] - Left_Line[i + 4]) >= 15)
        {
            Left_Up_Find = i;
        }

        // 右上拐点检测：下方平稳，上方向内突变
        if (Right_Up_Find == 0 &&
            abs(Right_Line[i] - Right_Line[i - 1]) <= 5 &&
            abs(Right_Line[i - 1] - Right_Line[i - 2]) <= 5 &&
            abs(Right_Line[i - 2] - Right_Line[i - 3]) <= 5 &&
            (Right_Line[i] - Right_Line[i + 2]) <= -8 &&
            (Right_Line[i] - Right_Line[i + 3]) <= -15 &&
            (Right_Line[i] - Right_Line[i + 4]) <= -15)
        {
            Right_Up_Find = i;
        }

        if (Left_Up_Find != 0 && Right_Up_Find != 0)
        {
            break;
        }
    }

    // 如果左右上拐点位置差距过大，认为检测无效
    if (abs(Right_Up_Find - Left_Up_Find) >= 30)
    {
        Right_Up_Find = 0;
        Left_Up_Find = 0;
    }
}

/**
 * @brief 延长左边界
 * @param start 起始行
 * @param end 结束行
 * @note 根据起始点前方的斜率延长边界
 */
void Lengthen_Left_Boundry(int start, int end)
{
    int i, t;
    float k = 0;

    // 边界检查
    if (start >= MT9V03X_H - 1)
        start = MT9V03X_H - 1;
    else if (start <= 0)
        start = 0;
    if (end >= MT9V03X_H - 1)
        end = MT9V03X_H - 1;
    else if (end <= 0)
        end = 0;

    // 确保end > start
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    // 如果起始点太靠前，直接连线
    if (start <= 5)
    {
        Left_Add_Line(Left_Line[start], start, Left_Line[end], end);
    }
    // 否则根据斜率延长
    else
    {
        k = (float)(Left_Line[start] - Left_Line[start - 4]) / 5.0;
        for (i = start; i <= end; i++)
        {
            Left_Line[i] = (int)(i - start) * k + Left_Line[start];
            if (Left_Line[i] >= MT9V03X_W - 1)
            {
                Left_Line[i] = MT9V03X_W - 1;
            }
            else if (Left_Line[i] <= 0)
            {
                Left_Line[i] = 0;
            }
        }
    }
}

/**
 * @brief 延长右边界
 * @param start 起始行
 * @param end 结束行
 * @note 根据起始点前方的斜率延长边界
 */
void Lengthen_Right_Boundry(int start, int end)
{
    int i, t;
    float k = 0;

    // 边界检查
    if (start >= MT9V03X_H - 1)
        start = MT9V03X_H - 1;
    else if (start <= 0)
        start = 0;
    if (end >= MT9V03X_H - 1)
        end = MT9V03X_H - 1;
    else if (end <= 0)
        end = 0;

    // 确保end > start
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    // 如果起始点太靠前，直接连线
    if (start <= 5)
    {
        Right_Add_Line(Right_Line[start], start, Right_Line[end], end);
    }
    // 否则根据斜率延长
    else
    {
        k = (float)(Right_Line[start] - Right_Line[start - 4]) / 5.0;
        for (i = start; i <= end; i++)
        {
            Right_Line[i] = (int)(i - start) * k + Right_Line[start];
            if (Right_Line[i] >= MT9V03X_W - 1)
            {
                Right_Line[i] = MT9V03X_W - 1;
            }
            else if (Right_Line[i] <= 0)
            {
                Right_Line[i] = 0;
            }
        }
    }
}

/**
 * @brief 十字路口检测
 * @note 通过检测上下拐点判断是否为十字路口，并修补边界
 */
void Cross_Detect()
{
    int down_search_start = 0;
    Cross_Flag = 0;

    // 环岛和坡道状态下不检测十字路口
    if (Island_State == 0 && Ramp_Flag == 0)
    {
        Left_Up_Find = 0;
        Right_Up_Find = 0;

        // 如果双边界丢失较多，尝试搜索上拐点
        if (Both_Lost_Time >= 10)
        {
            Find_Up_Point(MT9V03X_H - 1, 0);
            if (Left_Up_Find == 0 && Right_Up_Find == 0)
            {
                return;
            }
        }

        // 如果找到左右上拐点，判定为十字路口
        if (Left_Up_Find != 0 && Right_Up_Find != 0)
        {
            Cross_Flag = 1;
            down_search_start = Left_Up_Find > Right_Up_Find ? Left_Up_Find : Right_Up_Find;

            // 搜索下拐点
            Find_Down_Point(MT9V03X_H - 5, down_search_start + 2);

            // 确保下拐点在上拐点下方
            if (Left_Down_Find <= Left_Up_Find)
            {
                Left_Down_Find = 0;
            }
            if (Right_Down_Find <= Right_Up_Find)
            {
                Right_Down_Find = 0;
            }

            // 根据找到的拐点情况修补边界
            if (Left_Down_Find != 0 && Right_Down_Find != 0)
            {
                Left_Add_Line(Left_Line[Left_Up_Find], Left_Up_Find, Left_Line[Left_Down_Find], Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find], Right_Up_Find, Right_Line[Right_Down_Find], Right_Down_Find);
            }
            else if (Left_Down_Find == 0 && Right_Down_Find != 0)
            {
                Lengthen_Left_Boundry(Left_Up_Find - 1, MT9V03X_H - 1);
                Right_Add_Line(Right_Line[Right_Up_Find], Right_Up_Find, Right_Line[Right_Down_Find], Right_Down_Find);
            }
            else if (Left_Down_Find != 0 && Right_Down_Find == 0)
            {
                Left_Add_Line(Left_Line[Left_Up_Find], Left_Up_Find, Left_Line[Left_Down_Find], Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find - 1, MT9V03X_H - 1);
            }
            else if (Left_Down_Find == 0 && Right_Down_Find == 0)
            {
                Lengthen_Left_Boundry(Left_Up_Find - 1, MT9V03X_H - 1);
                Lengthen_Right_Boundry(Right_Up_Find - 1, MT9V03X_H - 1);
            }
        }
        else
        {
            Cross_Flag = 0;
        }
    }
}

/**
 * @brief 绘制直线到二值化图像上
 * @param startX 起点X坐标
 * @param startY 起点Y坐标
 * @param endX 终点X坐标
 * @param endY 终点Y坐标
 * @note 用于在图像上绘制辅助线
 */
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i, x, y;
    int start = 0, end = 0;

    // 边界检查
    if (startX >= MT9V03X_W - 1)
        startX = MT9V03X_W - 1;
    else if (startX <= 0)
        startX = 0;
    if (startY >= MT9V03X_H - 1)
        startY = MT9V03X_H - 1;
    else if (startY <= 0)
        startY = 0;
    if (endX >= MT9V03X_W - 1)
        endX = MT9V03X_W - 1;
    else if (endX <= 0)
        endX = 0;
    if (endY >= MT9V03X_H - 1)
        endY = MT9V03X_H - 1;
    else if (endY <= 0)
        endY = 0;

    // 竖直线
    if (startX == endX)
    {
        if (startY > endY)
        {
            start = endY;
            end = startY;
        }
        for (i = start; i <= end; i++)
        {
            if (i <= 1)
                i = 1;
            binaryImage[i][startX] = IMG_BLACK;
            binaryImage[i - 1][startX] = IMG_BLACK;
        }
    }
    // 水平线
    else if (startY == endY)
    {
        if (startX > endX)
        {
            start = endX;
            end = startX;
        }
        for (i = start; i <= end; i++)
        {
            if (startY <= 1)
                startY = 1;
            binaryImage[startY][i] = IMG_BLACK;
            binaryImage[startY - 1][i] = IMG_BLACK;
        }
    }
    // 斜线
    else
    {
        // 按Y方向绘制
        if (startY > endY)
        {
            start = endY;
            end = startY;
        }
        else
        {
            start = startY;
            end = endY;
        }
        for (i = start; i <= end; i++)
        {
            x = (int)(startX + (endX - startX) * (i - startY) / (endY - startY));
            if (x >= MT9V03X_W - 1)
                x = MT9V03X_W - 1;
            else if (x <= 1)
                x = 1;
            binaryImage[i][x] = IMG_BLACK;
            binaryImage[i][x - 1] = IMG_BLACK;
        }

        // 按X方向绘制
        if (startX > endX)
        {
            start = endX;
            end = startX;
        }
        else
        {
            start = startX;
            end = endX;
        }
        for (i = start; i <= end; i++)
        {
            y = (int)(startY + (endY - startY) * (i - startX) / (endX - startX));
            if (y >= MT9V03X_H - 1)
                y = MT9V03X_H - 1;
            else if (y <= 0)
                y = 0;
            binaryImage[y][i] = IMG_BLACK;
        }
    }
}

/**
 * @brief 斑马线检测
 * @return 检测结果（0=无斑马线，1=有斑马线）
 * @note 通过统计图像底部白黑跳变次数判断是否为斑马线
 */
int Zebra_Detect(void)
{
    uint8 zebra_count = 0;

    // 检测条件：最长白列在合理范围内，且搜索停止行足够高
    if (Longest_White_Column_Left[1] > 20 && Longest_White_Column_Left[1] < IMAGE_WIDTH - 20 &&
        Longest_White_Column_Right[1] > 20 && Longest_White_Column_Right[1] < IMAGE_WIDTH - 20 &&
        Search_Stop_Line >= 110)

        // 扫描图像底部几行
        for (int i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - 3; i--)
        {
            for (int j = 20; j <= IMAGE_WIDTH - 1 - 20; j++)
            {
                // 统计白黑跳变次数
                if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j + 1] == IMG_BLACK && binaryImage[i][j + 2] == IMG_BLACK)
                {
                    zebra_count++;
                }
            }

            // 跳变次数超过阈值，判定为斑马线
            if (zebra_count >= 10)
            {
                return 1;
            }
        }

    return 0;
}

/**
 * @brief 坡道检测
 * @note 通过赛道宽度变化和编码器变化判断坡道状态
 */
void Ramp_Detect(void)
{
    static int Encoder_Sum_Last, Encoder_Sum;
    int i = 0;
    int count = 0;

    // 十字路口或环岛状态下不检测坡道
    if (Cross_Flag != 0 || Island_State != 0)
    {
        return;
    }

    // 检测赛道宽度是否超过标准宽度（坡道特征）
    if (Search_Stop_Line >= 66)
    {
        for (i = MT9V03X_H - 1; i > MT9V03X_H - Search_Stop_Line; i--)
        {
            if (Road_Wide[i] > Road_Standard_Wide[i] + Ramp_offset)
            {
                count++;
            }
        }
    }

    // 如果宽度异常行数超过阈值，进入坡道检测状态机
    if (count >= 10)
    {
        Ramp_Flag = 1;
        int err;
        Encoder_Sum = Encoder_Left + Encoder_Left;
        err = Encoder_Sum - Encoder_Sum_Last;
        Encoder_Sum_Last = Encoder_Sum;

        // 状态1：检测上坡（编码器突然减小）
        if (Ramp_Flag == 1)
        {
            if (err < -100)
            {
                Ramp_Flag = 2;
            }
        }
        // 状态2：检测下坡（编码器突然增大）
        else if (Ramp_Flag == 2)
        {
            if (err > 100)
            {
                Ramp_Flag = 3;
            }
        }
        // 状态3：等待通过坡道
        else if (Ramp_Flag == 3)
        {
            if (encoder_sum > 30000)
            {
                Ramp_Flag = 0;
            }
        }
    }
    else
    {
        Ramp_Flag = 0;
    }
}

/**
 * @brief 计算图像中线偏差的平均值
 * @param start_point 起始行（从图像底部算起）
 * @param end_point 结束行
 * @return 中线偏差平均值（正值表示偏右，负值表示偏左）
 * @note 用于转向PID控制的P环输入
 */
float err_sum_average(uint8 start_point, uint8 end_point)
{
    // 确保start_point < end_point
    if (end_point < start_point)
    {
        uint8 t = end_point;
        end_point = start_point;
        start_point = t;
    }

    // 边界检查
    if (start_point < MT9V03X_H - Search_Stop_Line)
        start_point = MT9V03X_H - Search_Stop_Line - 1;
    if (end_point < MT9V03X_H - Search_Stop_Line)
        end_point = MT9V03X_H - Search_Stop_Line - 2;

    // 计算偏差累加值
    float err = 0;
    for (int i = start_point; i <= end_point; i++)
    {
        err += (MT9V03X_W / 2 - ((Left_Line[i] + Right_Line[i]) >> 1));
    }

    // 计算平均偏差
    err = err / (end_point - start_point + 1);
    return err;
}

/**
 * @brief 检测图像是否出界
 * @param binaryImage 二值化图像数组
 * @return 出界标志（0=未出界，1=出界）
 * @note 通过检测图像底部中央区域的平均灰度判断
 */
uint8 image_out_of_bounds(uint8 binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    // 如果检测到斑马线，不判定为出界
    if (Zebra_Detect())
    {
        return 0;
    }

    // 统计图像底部中央区域的灰度值
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            sum += image_copy[IMAGE_HEIGHT - 1 - j][IMAGE_WIDTH / 2 - 5 + i];
        }
    }

    // 计算平均灰度值
    int average = sum / 20;

    // 如果平均灰度较低（较黑），判定为出界
    if (average < 110)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 图像处理主函数
 * @note 集成边界提取和元素识别的主函数
 *       包括：图像复制、大津法二值化、边界搜索、元素检测等
 */
void image_process(void)
{
    // 0. 复制图像数据并进行大津法二值化
    memcpy(image_copy, mt9v03x_image, MT9V03X_H * MT9V03X_W);
    threshold = otsu_get_threshold((uint8 *)image_copy, MT9V03X_W, MT9V03X_H);
    applyThreshold(image_copy, binaryImage, threshold);

    // 1. 双边巡线 - 提取左右边界
    Longest_White_Column();

    // 2. 赛道元素检测
    Cross_Detect();  // 十字路口检测
    Ramp_Detect();   // 坡道检测

    // 3. 设置图像处理完成标志
    image_proess = 1;
}
