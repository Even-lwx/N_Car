#include "zf_common_headfile.h"
#include "image.h"

#define IMG_BLACK 0   // 0x00�Ǻ�
#define IMG_WHITE 255 // 0xffΪ��

// 环岛检测变量定义
volatile int circle_flag = 0;       // 环岛标志（0=无环岛，非0=有环岛）
volatile int right_circle_flag = 0; // 右环岛标志（0/1/2/3不同状态）

// 环岛状态变量定义
volatile int Island_State = 0; // 环岛状态标志

// 编码器变量定义
int Encoder_Left = 0;  // 左编码器累计值
int encoder_sum = 0;   // 编码器总和

// 图像副本数组定义
uint8 image_copy[IMAGE_HEIGHT][IMAGE_WIDTH]; // 图像副本数组

// ���û�������
extern const uint8 Image_Flags[][9][8];                // ����ͼ�ϵ����ֱ��
extern uint8_t binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]; // ��ֵ�����ԭ����
volatile int Left_Line[MT9V03X_H];                     // ���������
volatile int Right_Line[MT9V03X_H];                    // �ұ�������
volatile int Mid_Line[MT9V03X_H];                      // ��������
volatile int Road_Wide[MT9V03X_H];                     // ��������
volatile int White_Column[MT9V03X_W];                  // ÿ�а��г���
volatile int Search_Stop_Line;                         // ������ֹ��,ֻ��¼���ȣ���Ҫ������Ҫ����Ұ�߶ȼ�ȥ��ֵ
volatile int Boundry_Start_Left;                       // ���ұ߽���ʼ��
volatile int Boundry_Start_Right;                      // ��һ���Ƕ��ߵ�,����߽���ʼ��
volatile int Left_Lost_Time;                           // �߽綪����
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;                // ����ͬʱ������
int Longest_White_Column_Left[2];           // �����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
int Longest_White_Column_Right[2];          // �����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
int Left_Lost_Flag[MT9V03X_H];              // �������飬������1��û������0
int Right_Lost_Flag[MT9V03X_H];             // ��Standard_�������飬������1��û������0
const uint8 Road_Standard_Wide[MT9V03X_H] = // ÿ�е�������
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
        173, 175, 175, 177, 177, 179, 180, 181, 184, 184};
/*����Ԫ�� */

// ʮ��
volatile int Cross_Flag = 0;
volatile int Left_Down_Find = 0; // ʮ��ʹ�ã��ҵ�����������û�ҵ�����0
volatile int Left_Up_Find = 0;   // �ĸ��յ��־
volatile int Right_Down_Find = 0;
volatile int Right_Up_Find = 0;

// ��������
extern volatile int Island_State; // ����״̬��־
extern volatile int Ramp_Flag;    // �µ�

// �µ�
volatile int Ramp_Flag = 0; // �µ���־

int turn_start = 50;
int turn_end = 53;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ˫�����Ѳ��
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      �����Ѳ�ߣ�Ѱ�ҳ�ʼ�߽磬���ߣ�����еȻ���Ԫ�أ�������ȡ��Щ��������������ʶ��
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column() // �����Ѳ��
{
    int i, j;
    int start_column = 35;                 // ����е�������ʼ��
    int end_column = MT9V03X_W - 35;       // ����е�������ֹ��
    int left_border = 0, right_border = 0; // ��ʱ�洢����λ��
    Longest_White_Column_Left[0] = 0;      // �����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Left[1] = 0;      // �����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right[0] = 0;     // �����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right[1] = 0;     // �����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Right_Lost_Time = 0;                   // �߽綪����
    Left_Lost_Time = 0;
    Boundry_Start_Left = 0; // ��һ���Ƕ��ߵ�,����߽���ʼ��
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0; // ����ͬʱ������

    /*��������*/
    for (i = 0; i <= MT9V03X_H - 1; i++) // �������
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W - 1;
    }
    for (i = 0; i <= MT9V03X_W - 1; i++) // �������
    {
        White_Column[i] = 0;
    }

    // ������Ҫ������з�Χ�����޶�������3״̬�Ҳ����Ͻǵ㣬�����޸���������
    // ����2״̬��Ҫ�ı������Ѱ�ҷ�Χ
    if (circle_flag) // �һ�
    {
        if (right_circle_flag == 2)
        {
            start_column = 60;
            end_column = MT9V03X_W - 20;
        }
    }

    // �����ң��������ϣ�����ȫͼ��¼��Χ�ڵ�ÿһ�а׵�����
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

    // ����������������
    Longest_White_Column_Left[0] = 0;
    for (i = start_column; i <= end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i]) // �������һ��
        {
            Longest_White_Column_Left[0] = White_Column[i]; // ��0���ǰ��г���
            Longest_White_Column_Left[1] = i;               // ��1�����±꣬��i��
        }
    }
    // ���ҵ���������������
    Longest_White_Column_Right[0] = 0;                           // ��0���ǰ��г���
    for (i = end_column; i >= Longest_White_Column_Left[1]; i--) // ��������ע���������ҵ���������λ�þͿ���ͣ��
    {
        if (Longest_White_Column_Right[0] < White_Column[i]) // �������һ��
        {
            Longest_White_Column_Right[0] = White_Column[i]; // ��0���ǰ��г���
            Longest_White_Column_Right[1] = i;               // ��1�����±꣬��j��
        }
    }

    Search_Stop_Line = (Longest_White_Column_Left[0] > Longest_White_Column_Right[0]) ? Longest_White_Column_Left[0] : Longest_White_Column_Right[0]; // �ǳ���Ҫ��������ֹ�д洢

    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--) // ����Ѳ��
    {
        /*���ұ߽� */
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j + 1] == IMG_BLACK && binaryImage[i][j + 2] == IMG_BLACK) // �׺ںڣ����ұ߽�
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; // �Ҷ������飬������1����������0
                break;
            }
            else if (j >= MT9V03X_W - 1 - 2) // û�ҵ��ұ߽磬����Ļ���Ҹ�ֵ���ұ߽�
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; // �Ҷ������飬������1����������0
                break;
            }
        }
        /*����߽� */
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--) // �����ɨ��
        {
            if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j - 1] == IMG_BLACK && binaryImage[i][j - 2] == IMG_BLACK) // �׺ںڣ�����߽�
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; // �������飬������1����������0
                break;
            }
            else if (j <= 2)
            {
                left_border = j;       // �ҵ�ͷ��û�ҵ��ߣ��Ͱ���Ļ�����ҵ����߽�
                Left_Lost_Flag[i] = 1; // �������飬������1����������0
                break;
            }
        }
        Left_Line[i] = left_border;   // �����������
        Right_Line[i] = right_border; // �ұ���������
    }

    for (i = MT9V03X_H - 1; i >= 0; i--) // �������ݳ�������
    {
        if (Left_Lost_Flag[i] == 1) // ���߶�����
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1) // ˫�߶�����
            Both_Lost_Time++;
        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] != 1) // ��¼��һ���Ƕ��ߵ㣬�߽���ʼ��
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i] = Right_Line[i] - Left_Line[i];
			//	printf("%d,%d\r\n",i, Road_Wide[i]);
    }

    // debugʹ�ã���Ļ��ʾ��ز���
    //    ips200_showint16(0,0, Longest_White_Column_Right[0]);//��0���ǰ��г���
    //    ips200_showint16(0,1, Longest_White_Column_Right[1]);//��1�����±꣬��j��)
    //    ips200_showint16(0,2, Longest_White_Column_Left[0]);//��0���ǰ��г���
    //    ips200_showint16(0,3, Longest_White_Column_Left[1]);//��1�����±꣬��j��)
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �߽���ʾ������ͼ������ʾ����Ļ�ϣ�
  @param     null
  @return    null
  Sample     ֱ�ӵ���
  @note      ��ʾ�����ұ߽磬���ߣ�
                                           ��������²�Ҫ�ã���Ϊֱ����ԭͼ��д���˱߽���Ϣ
                                           ���Ԫ���ж���ɸ��ŵģ�����ʱ�����
-------------------------------------------------------------------------------------------------------------------*/
void Show_Boundry(void)
{
    int16 i;
    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--) // �����������ɨ��
    {
        binaryImage[i][Left_Line[i] + 1] = IMG_BLACK;
        binaryImage[i][(Left_Line[i] + Right_Line[i]) >> 1] = IMG_BLACK;
        binaryImage[i][Right_Line[i] - 1] = IMG_BLACK;
    }

    // ����Ļ�������ߴ���ʾ���ߣ����ڵ�������ͷ
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����
  @param     ���ߵ���㣬�յ�
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      ����ֱ���Ǳ߽磬������ǿ��Ŷȸߵ�,��Ҫ�Ҳ�
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1, int y1, int x2, int y2) // ����,�����Ǳ߽�
{
    int i, max, a1, a2;
    int hx;
    if (x1 >= MT9V03X_W - 1) // ��ʼ��λ��У�����ų�����Խ��Ŀ���
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
    a1 = y1;
    a2 = y2;
    if (a1 > a2) // ���껥��
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++) // ����б�ʲ��߼���
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        Left_Line[i] = hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �Ҳ���
  @param     ���ߵ���㣬�յ�
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      ����ֱ���Ǳ߽磬������ǿ��Ŷȸߵģ���Ҫ�Ҳ�
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1, int y1, int x2, int y2) // �Ҳ���,�����Ǳ߽�
{
    int i, max, a1, a2;
    int hx;
    if (x1 >= MT9V03X_W - 1) // ��ʼ��λ��У�����ų�����Խ��Ŀ���
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
    a1 = y1;
    a2 = y2;
    if (a1 > a2) // ���껥��
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++) // ����б�ʲ��߼���
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        Right_Line[i] = hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start, int end)
{
    int i, t;
    Right_Down_Find = 0;
    Left_Down_Find = 0;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (start >= MT9V03X_H - 1 - 5) // ����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start = MT9V03X_H - 1 - 5;
    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5)
        end = 5;
    for (i = start; i >= end; i--)
    {
        if (Left_Down_Find == 0 &&                       // ֻ�ҵ�һ�����������ĵ�
            abs(Left_Line[i] - Left_Line[i + 1]) <= 5 && // �ǵ����ֵ���Ը���
            abs(Left_Line[i + 1] - Left_Line[i + 2]) <= 5 &&
            abs(Left_Line[i + 2] - Left_Line[i + 3]) <= 5 &&
            (Left_Line[i] - Left_Line[i - 2]) >= 8 &&
            (Left_Line[i] - Left_Line[i - 3]) >= 15 &&
            (Left_Line[i] - Left_Line[i - 4]) >= 15)
        {
            Left_Down_Find = i; // ��ȡ��������
        }
        if (Right_Down_Find == 0 &&                        // ֻ�ҵ�һ�����������ĵ�
            abs(Right_Line[i] - Right_Line[i + 1]) <= 5 && // �ǵ����ֵ���Ը���
            abs(Right_Line[i + 1] - Right_Line[i + 2]) <= 5 &&
            abs(Right_Line[i + 2] - Right_Line[i + 3]) <= 5 &&
            (Right_Line[i] - Right_Line[i - 2]) <= -8 &&
            (Right_Line[i] - Right_Line[i - 3]) <= -15 &&
            (Right_Line[i] - Right_Line[i - 4]) <= -15)
        {
            Right_Down_Find = i;
        }
        if (Left_Down_Find != 0 && Right_Down_Find != 0) // �����ҵ����˳�
        {
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start, int end)
{
    int i, t;
    Left_Up_Find = 0;
    Right_Up_Find = 0;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5) // ��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��
        end = 5;
    if (start >= MT9V03X_H - 1 - 5) // ����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start = MT9V03X_H - 1 - 5;
    for (i = start; i >= end; i--)
    {
        if (Left_Up_Find == 0 && // ֻ�ҵ�һ�����������ĵ�
            abs(Left_Line[i] - Left_Line[i - 1]) <= 5 &&
            abs(Left_Line[i - 1] - Left_Line[i - 2]) <= 5 &&
            abs(Left_Line[i - 2] - Left_Line[i - 3]) <= 5 &&
            (Left_Line[i] - Left_Line[i + 2]) >= 8 &&
            (Left_Line[i] - Left_Line[i + 3]) >= 15 &&
            (Left_Line[i] - Left_Line[i + 4]) >= 15)
        {
            Left_Up_Find = i; // ��ȡ��������
        }
        if (Right_Up_Find == 0 &&                          // ֻ�ҵ�һ�����������ĵ�
            abs(Right_Line[i] - Right_Line[i - 1]) <= 5 && // ��������λ�ò��
            abs(Right_Line[i - 1] - Right_Line[i - 2]) <= 5 &&
            abs(Right_Line[i - 2] - Right_Line[i - 3]) <= 5 &&
            (Right_Line[i] - Right_Line[i + 2]) <= -8 &&
            (Right_Line[i] - Right_Line[i + 3]) <= -15 &&
            (Right_Line[i] - Right_Line[i + 4]) <= -15)
        {
            Right_Up_Find = i; // ��ȡ��������
        }
        if (Left_Up_Find != 0 && Right_Up_Find != 0) // ���������ҵ��ͳ�ȥ
        {
            break;
        }
    }
    if (abs(Right_Up_Find - Left_Up_Find) >= 30) // ����˺�ѹ�����Ϊ����
    {
        Right_Up_Find = 0;
        Left_Up_Find = 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ��߽��ӳ�
  @param     �ӳ���ʼ�������ӳ���ĳ��
  @return    null
  Sample     Stop_Detect(void)
  @note      ����ʼ��������5���㣬���б�ʣ������ӳ���ֱ��������
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start, int end)
{
    int i, t;
    float k = 0;
    if (start >= MT9V03X_H - 1) // ��ʼ��λ��У�����ų�����Խ��Ŀ���
        start = MT9V03X_H - 1;
    else if (start <= 0)
        start = 0;
    if (end >= MT9V03X_H - 1)
        end = MT9V03X_H - 1;
    else if (end <= 0)
        end = 0;
    if (end < start) //++���ʣ����껥��
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5) // ��Ϊ��Ҫ�ڿ�ʼ��������5���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Left_Add_Line(Left_Line[start], start, Left_Line[end], end);
    }

    else
    {
        k = (float)(Left_Line[start] - Left_Line[start - 4]) / 5.0; // �����k��1/б��
        for (i = start; i <= end; i++)
        {
            Left_Line[i] = (int)(i - start) * k + Left_Line[start]; //(x=(y-y1)*k+x1),��бʽ����
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

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����߽��ӳ�
  @param     �ӳ���ʼ�������ӳ���ĳ��
  @return    null
  Sample     Stop_Detect(void)
  @note      ����ʼ��������3���㣬���б�ʣ������ӳ���ֱ��������
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start, int end)
{
    int i, t;
    float k = 0;
    if (start >= MT9V03X_H - 1) // ��ʼ��λ��У�����ų�����Խ��Ŀ���
        start = MT9V03X_H - 1;
    else if (start <= 0)
        start = 0;
    if (end >= MT9V03X_H - 1)
        end = MT9V03X_H - 1;
    else if (end <= 0)
        end = 0;
    if (end < start) //++���ʣ����껥��
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5) // ��Ϊ��Ҫ�ڿ�ʼ��������5���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Right_Add_Line(Right_Line[start], start, Right_Line[end], end);
    }
    else
    {
        k = (float)(Right_Line[start] - Right_Line[start - 4]) / 5.0; // �����k��1/б��
        for (i = start; i <= end; i++)
        {
            Right_Line[i] = (int)(i - start) * k + Right_Line[start]; //(x=(y-y1)*k+x1),��бʽ����
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

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ʮ�ּ��
  @param     null
  @return    null
  Sample     Cross_Detect(void);
  @note      �����ĸ��յ��б����������ĸ��ǵ㣬�����ҵ��յ�ĸ��������Ƿ���
-------------------------------------------------------------------------------------------------------------------*/

void Cross_Detect()
{
    int down_search_start = 0; // �µ�������ʼ��
    Cross_Flag = 0;
    if (Island_State == 0 && Ramp_Flag == 0) // �뻷�����µ����⿪
    {
        Left_Up_Find = 0;
        Right_Up_Find = 0;
        if (Both_Lost_Time >= 10) // ʮ�ֱض���˫�߶��ߣ�����˫�߶��ߵ�������ٿ�ʼ�ҽǵ�
        {
            Find_Up_Point(MT9V03X_H - 1, 0);
            if (Left_Up_Find == 0 && Right_Up_Find == 0) // ֻҪû��ͬʱ�ҵ������ϵ㣬ֱ�ӽ���
            {
                return;
            }
        }
        if (Left_Up_Find != 0 && Right_Up_Find != 0) // �ҵ������ϵ㣬���ҵ�ʮ����
        {
            Cross_Flag = 1;                                                                  // ��Ӧ��־λ�����ڸ�Ԫ�ػ����
            down_search_start = Left_Up_Find > Right_Up_Find ? Left_Up_Find : Right_Up_Find; // �������Ϲյ����꿿������Ϊ�µ����������
            Find_Down_Point(MT9V03X_H - 5, down_search_start + 2);                           // ���Ϲյ���2����Ϊ�µ�Ľ�ֹ��
            if (Left_Down_Find <= Left_Up_Find)
            {
                Left_Down_Find = 0; // �µ㲻���ܱ��ϵ㻹����
            }
            if (Right_Down_Find <= Right_Up_Find)
            {
                Right_Down_Find = 0; // �µ㲻���ܱ��ϵ㻹����
            }
            if (Left_Down_Find != 0 && Right_Down_Find != 0)
            { // �ĸ��㶼�ڣ��������ߣ����������Ȼ����
                Left_Add_Line(Left_Line[Left_Up_Find], Left_Up_Find, Left_Line[Left_Down_Find], Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find], Right_Up_Find, Right_Line[Right_Down_Find], Right_Down_Find);
            }
            else if (Left_Down_Find == 0 && Right_Down_Find != 0) // 11//����ʹ�õĶ���б�ʲ���
            {                                                     // ������                                     //01
                Lengthen_Left_Boundry(Left_Up_Find - 1, MT9V03X_H - 1);
                Right_Add_Line(Right_Line[Right_Up_Find], Right_Up_Find, Right_Line[Right_Down_Find], Right_Down_Find);
            }
            else if (Left_Down_Find != 0 && Right_Down_Find == 0) // 11
            {                                                     // ������                                     //10
                Left_Add_Line(Left_Line[Left_Up_Find], Left_Up_Find, Left_Line[Left_Down_Find], Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find - 1, MT9V03X_H - 1);
            }
            else if (Left_Down_Find == 0 && Right_Down_Find == 0) // 11
            {                                                     // �����ϵ�                                   //00
                Lengthen_Left_Boundry(Left_Up_Find - 1, MT9V03X_H - 1);
                Lengthen_Right_Boundry(Right_Up_Find - 1, MT9V03X_H - 1);
            }
        }
        else
        {
            Cross_Flag = 0;
        }
    }
    // �ǵ���ر�����debugʹ��
    // ips200_showuint8(0,12,Cross_Flag);
    //    ips200_showuint8(0,13,Island_State);
    //    ips200_showuint8(50,12,Left_Up_Find);
    //    ips200_showuint8(100,12,Right_Up_Find);
    //    ips200_showuint8(50,13,Left_Down_Find);
    //    ips200_showuint8(100,13,Right_Down_Find);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����
  @param     ������ʼ�㣬�յ����꣬��һ������Ϊ2�ĺ���
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    ��һ�����
  @note     ���ľ���һ���ߣ���Ҫ����ɨ��
-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i, x, y;
    int start = 0, end = 0;
    if (startX >= MT9V03X_W - 1) // �޷�����
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

    if (startX == endX) // һ������
    {
        if (startY > endY) // ����
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

    else if (startY == endY) // ��һ������
    {
        if (startX > endX) // ����
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
    else // ����������ˮƽ����ֱ��������������ǳ������
    {
        if (startY > endY) // ��ʼ�����
        {
            start = endY;
            end = startY;
        }
        else
        {
            start = startY;
            end = endY;
        }
        for (i = start; i <= end; i++) // �����ߣ���֤ÿһ�ж��кڵ�
        {
            x = (int)(startX + (endX - startX) * (i - startY) / (endY - startY)); // ����ʽ����
            if (x >= MT9V03X_W - 1)
                x = MT9V03X_W - 1;
            else if (x <= 1)
                x = 1;
            binaryImage[i][x] = IMG_BLACK;
            binaryImage[i][x - 1] = IMG_BLACK;
        }
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
        for (i = start; i <= end; i++) // �����ߣ���֤ÿһ�ж��кڵ�
        {

            y = (int)(startY + (endY - startY) * (i - startX) / (endX - startX)); // ����ʽ����
            if (y >= MT9V03X_H - 1)
                y = MT9V03X_H - 1;
            else if (y <= 0)
                y = 0;
            binaryImage[y][i] = IMG_BLACK;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �������ж�
  @param     null
  @return    ��⵽�����߷���1�����򷵻�0
  Sample     Zebra_Detect();
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Zebra_Detect(void)
{
    uint8 zebra_count = 0;
    if (Longest_White_Column_Left[1] > 20 && Longest_White_Column_Left[1] < IMAGE_WIDTH - 20 &&
        Longest_White_Column_Right[1] > 20 && Longest_White_Column_Right[1] < IMAGE_WIDTH - 20 &&
        Search_Stop_Line >= 110)                                   // �����������ƣ�����ѭ������
        for (int i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - 3; i--) // ȡ���м���
        {
            for (int j = 20; j <= IMAGE_WIDTH - 1 - 20; j++) // ��ⷶΧ������20��֮��
            {
                if (binaryImage[i][j] == IMG_WHITE && binaryImage[i][j + 1] == IMG_BLACK && binaryImage[i][j + 2] == IMG_BLACK)
                {
                    zebra_count++;
                }
            }
            if (zebra_count >= 10) // �����ɫ�������ڵ���10����Ϊ�ǰ�����
            {
                return 1;
            }
        }

    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �µ��ж�
  @param     null
  @return
  Sample
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Ramp_offset;
void Ramp_Detect(void)
{
    static int Encoder_Sum_Last, Encoder_Sum;
    int i = 0;
    int count = 0;
    // ��ʮ�֡������Ļ���
    if (Cross_Flag != 0 || Island_State != 0)
    {
        return;
    }

    if (Search_Stop_Line >= 66) // ��ֹ�г���������ʱ������
    {
        // ��ͼ��ײ����ϱ���������ֹ�з�Χ�ڵ���
        for (i = MT9V03X_H - 1; i > MT9V03X_H - Search_Stop_Line; i--)
        {

            if (Road_Wide[i] > Road_Standard_Wide[i] + Ramp_offset) // �������ȳ�����׼����+10
            {
                count++; // �ۼƳ�������
            }
        }
    }

    if (count >= 10) // �����������ﵽ��ֵ
    {
        //buzzer_on(50);
				Ramp_Flag = 1;
        int err;
        Encoder_Sum = Encoder_Left + Encoder_Left;
        err = Encoder_Sum - Encoder_Sum_Last;
        Encoder_Sum_Last = Encoder_Sum;
        if (Ramp_Flag == 1) // ��ʼ״̬�������µ�ǰ
        {

            /*�µ�ǰ�ĵ��� */

            if (err < -100) // ���������µ��ϵ�����(����)
            {
                //buzzer_on(50);
                Ramp_Flag = 2;
            }
        }
        else if (Ramp_Flag == 2)
        {
            /*�µ���*/
            if (err > 100) // ��ʼ����(����)
            {
                //buzzer_on(50);
                Ramp_Flag = 3;
            }
        }
        else if (Ramp_Flag == 3) // ��������״̬
        {
            /*���µĵ���*/

            if (encoder_sum > 30000) // ����µ�������
            {
                //buzzer_on(50);
                Ramp_Flag = 0;
            }
        }
    }
		else
		{
				Ramp_Flag=0;
		}

    // �����ã���ʾ�����м���������Ŀ��������ʾ������
    // ips200_showuint8(50, 10*16, count, 5);
}

/**
 *
 * @brief  ����ĳ���е�ƽ�����
 * @param  start_point
 * @param  end_point
 * @retval err ���ֵ
 **/
float err_sum_average(uint8 start_point, uint8 end_point)
{
    // ��ֹ�����������
    if (end_point < start_point)
    {
        uint8 t = end_point;
        end_point = start_point;
        start_point = t;
    }

    if (start_point < MT9V03X_H - Search_Stop_Line)
        start_point = MT9V03X_H - Search_Stop_Line - 1; // ��ֹ���Խ��
    if (end_point < MT9V03X_H - Search_Stop_Line)
        end_point = MT9V03X_H - Search_Stop_Line - 2; // ��ֹ�յ�Խ��

    float err = 0;
    for (int i = start_point; i <= end_point; i++)
    {
        err += (MT9V03X_W / 2 - ((Left_Line[i] + Right_Line[i]) >> 1)); // λ������Ч����2
    }
    err = err / (end_point - start_point+1);
    return err;
}

/**
 * @brief   �����ж�,ѡȡͼ��ײ��м�10*2��������ж�
 * @param   *binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH]  ԭʼͼ��
 * @retval   0:���� 1:����
 */
uint8 image_out_of_bounds(uint8 binaryImage[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    if (Zebra_Detect())
    {
        return 0;
    }
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            sum += image_copy[IMAGE_HEIGHT - 1 - j][IMAGE_WIDTH / 2 - 5 + i];
        }
    }
    int average = sum / 20; // ����ƽ��ֵ
    if (average < 110)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}