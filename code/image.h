#ifndef _IMAGE_H
#define _IMAGE_H

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

// 图像尺寸宏定义（使用MT9V03X摄像头的分辨率）
#define IMAGE_WIDTH  MT9V03X_W  // 图像宽度 188
#define IMAGE_HEIGHT MT9V03X_H  // 图像高度 120

#define TURN_STANDARD_START turn_start // ǰհ
#define TURN_STANDARD_END turn_end	   // ǰհ

extern int turn_start;
extern int turn_end;
extern volatile int Cross_Flag ;// ʮ��
extern volatile int Island_State; // ����״̬��־
extern volatile int Ramp_Flag;    // �µ�
extern int Ramp_offset;//�µ����ȵ���

// 环岛检测相关变量
extern volatile int circle_flag;       // 环岛标志（0=无环岛，非0=有环岛）
extern volatile int right_circle_flag; // 右环岛标志（0/1/2/3不同状态）

// 编码器相关变量
extern int Encoder_Left;  // 左编码器累计值
extern int encoder_sum;   // 编码器总和

// 图像处理相关
extern uint8 image_copy[IMAGE_HEIGHT][IMAGE_WIDTH]; // 图像副本数组


extern volatile int Left_Line[MT9V03X_H];  // ���������
extern volatile int Right_Line[MT9V03X_H]; // �ұ�������
extern const uint8 Road_Standard_Wide[MT9V03X_H];
extern volatile int Search_Stop_Line;
extern int Longest_White_Column_Left[2];  // �����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
extern int Longest_White_Column_Right[2]; // �����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
extern volatile int Zebra_Stripes_Flag;   // �����߱�־λ
void Longest_White_Column(void);
/*Ԫ��*/
void Cross_Detect(void);
int Zebra_Detect(void);
void Ramp_Detect(void);




void Show_Boundry(void);
void Find_Up_Point(int start, int end);
void Find_Down_Point(int start, int end);
void Lengthen_Left_Boundry(int start, int end);
void Lengthen_Right_Boundry(int start, int end);
void Left_Add_Line(int x1, int y1, int x2, int y2);
void Right_Add_Line(int x1, int y1, int x2, int y2);
void Draw_Line(int startX, int startY, int endX, int endY);
float err_sum_average(uint8 start_point, uint8 end_point);
uint8 image_out_of_bounds(uint8 binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]);

// 总图像处理函数
void image_process(void);  // 集成边界提取和元素识别的主函数

#endif
