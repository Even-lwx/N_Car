/*********************************************************************
 * 文件: menu_config.c
 * 用户菜单配置实现文件
 * 说明：用户只需修改此文件来添加/修改菜单，无需修改 menu.c 内核
 *
 * 使用步骤：
 * 1. 定义参数变量
 * 2. 定义步进数组（用于调参）
 * 3. 配置 CustomData 数组
 * 4. 定义 Page 页面结构
 * 5. 在 Menu_Config_Init() 中设置页面关系
 * 6. 在主菜单的 .enter[] 中添加页面指针
 ********************************************************************/

#include "menu_config.h"
#include "zf_common_headfile.h"


/**************** 外部声明（来自menu.c的内核函数） ****************/
extern void ips_clear(void);
extern void show_string(uint16 x, uint16 y, const char *str);
extern void show_int(uint16 x, uint16 y, int32 value, uint8 num);
extern void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);
extern uint8 Key_Scan(void);
extern uint8 Param_Save_All(void); // 返回值为 uint8
extern void buzzer_beep(uint8 times, uint16 on_time, uint16 off_time);
extern void servo_set_angle(float angle);
extern void motor_reset_protection(void);
extern void momentum_wheel_control(int16 pwm_value); // 参数类型为 int16
extern void drive_wheel_control(int16 pwm_value);    // 参数类型为 int16
extern void delayed_stop_start_with_param(void);     // 延迟停车函数

// 摄像头相关函数（来自Seekfree库）
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];    // 摄像头图像数据
extern void ips114_show_gray_image(uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold);
extern void ips114_draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);

// 图像处理相关数据
extern volatile int Left_Line[MT9V03X_H];  // 左边界数组
extern volatile int Right_Line[MT9V03X_H]; // 右边界数组
extern uint32 steer_sample_start;          // 转向PID采样起始行
extern uint32 steer_sample_end;            // 转向PID采样结束行

/**************** 外部变量引用 ****************/
extern volatile bool enable; // volatile 关键字
extern int16 encoder[2];     // 类型为 int16 (不是 int32)

/**************** 页面前置声明 ****************/
// 在页面相互引用前需要前置声明
extern Page page_turn_comp;     // 转弯补偿参数页面
extern Page page_motor_protect; // 电机保护参数页面
extern Page page_steer_pid;     // 转向PID参数页面

//============================================================
// 1. 示例菜单 - Example Menu（可选，供参考）
//============================================================
// 1.1 参数变量定义
float example_float1 = 1.5f;
float example_float2 = 2.0f;
int16 example_int16_1 = 100;
int16 example_int16_2 = 200;
uint32 example_uint32 = 1000;

// 1.2 步进数组定义
float float1_step[] = {0.01f, 0.1f, 1.0f};
float float2_step[] = {0.1f, 0.5f};
int16 int16_1_step[] = {1, 10, 100};
int16 int16_2_step[] = {5, 50};
uint32 uint32_step[] = {1, 10, 100, 1000};

// 1.3 参数配置数组
CustomData example_data[] = {
    {&example_float1, data_float_show, "Float Param 1", float1_step, 3, 0, 4, 4},
    {&example_float2, data_float_show, "Float Param 2", float2_step, 2, 0, 4, 4},
    {&example_int16_1, data_int16_show, "Int16 Param 1", int16_1_step, 3, 0, 4, 4},
    {&example_int16_2, data_int16_show, "Int16 Param 2", int16_2_step, 2, 0, 4, 4},
    {&example_uint32, data_uint32_show, "Uint32 Param", uint32_step, 4, 0, 4, 4},
};

// 1.4 页面定义
Page page_example = {
    .name = "Example Menu",
    .data = example_data,
    .len = 5,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 2. 舵机控制菜单 - Servo Control Menu
//============================================================
float servo_angle_step[] = {0.5f, 1.0f, 5.0f};

CustomData servo_data[] = {
    {&servo_motor_duty, data_float_show, "Servo Angle", servo_angle_step, 3, 0, 4, 1},
};

Page page_servo = {
    .name = "Servo Control",
    .data = servo_data,
    .len = 1,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 3. PID参数菜单 - PID Parameters
//============================================================
// 统一的PID步进数组
float pid_kp_step[] = {0.01f, 0.1f, 1.0f, 10.0f, 100.0f};
float pid_ki_step[] = {0.01f, 0.1f, 1.0f, 10.0f, 100.0f};
float pid_kd_step[] = {0.01f, 0.1f, 1.0f, 10.0f, 100.0f};
float pid_limit_step[] = {1.0f, 10.0f, 100.0f};
float gain_scale_step[] = {0.01f, 0.1f, 0.2f};

// 3.1 角速度环PID
CustomData gyro_pid_data[] = {
    {&gyro_pid.kp, data_float_show, "Kp", pid_kp_step, 5, 0, 4, 3},
    {&gyro_pid.ki, data_float_show, "Ki", pid_ki_step, 5, 0, 4, 3},
    {&gyro_pid.kd, data_float_show, "Kd", pid_kd_step, 5, 0, 4, 4},
    {&gyro_pid.max_integral, data_float_show, "Max Integral", pid_limit_step, 3, 0, 5, 1},
    {&gyro_pid.max_output, data_float_show, "Max Output", pid_limit_step, 3, 0, 5, 1},
    {&gyro_gain_scale, data_float_show, "Gain Scale", gain_scale_step, 3, 0, 3, 2},
};

Page page_gyro_pid = {
    .name = "Gyro PID",
    .data = gyro_pid_data,
    .len = 6,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.2 角度环PID
float machine_angle_step[] = {0.1f, 1.0f, 10.0f, 100.0f};
float deadzone_step[] = {0.1f, 1.0f, 5.0f, 10.0f};

CustomData angle_pid_data[] = {
    {&machine_angle, data_float_show, "Mech Zero", machine_angle_step, 4, 0, 2, 2},
    {&angle_pid.kp, data_float_show, "Kp", pid_kp_step, 5, 0, 4, 3},
    {&angle_pid.ki, data_float_show, "Ki", pid_ki_step, 5, 0, 4, 3},
    {&angle_pid.kd, data_float_show, "Kd", pid_kd_step, 5, 0, 4, 4},
    {&angle_pid.max_integral, data_float_show, "Max Integral", pid_limit_step, 3, 0, 5, 1},
    {&angle_pid.max_output, data_float_show, "Max Output", pid_limit_step, 3, 0, 5, 1},
    {&angle_deadzone, data_float_show, "Angle Deadzone", deadzone_step, 4, 0, 4, 1},
    {&angle_gain_scale, data_float_show, "Gain Scale", gain_scale_step, 3, 0, 3, 2},
};

Page page_angle_pid = {
    .name = "Angle PID",
    .data = angle_pid_data,
    .len = 8,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.3 速度环PID
CustomData speed_pid_data[] = {
    {&speed_pid.kp, data_float_show, "Kp", pid_kp_step, 5, 0, 4, 3},
    {&speed_pid.ki, data_float_show, "Ki", pid_ki_step, 5, 0, 4, 3},
    {&speed_pid.kd, data_float_show, "Kd", pid_kd_step, 5, 0, 4, 4},
    {&speed_pid.max_integral, data_float_show, "Max Integral", pid_limit_step, 3, 0, 5, 1},
    {&speed_pid.max_output, data_float_show, "Max Output", pid_limit_step, 3, 0, 5, 1},
};

Page page_speed_pid = {
    .name = "Speed PID",
    .data = speed_pid_data,
    .len = 5,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.4 行进轮速度环PID
float target_speed_step[] = {1.0f, 5.0f, 10.0f};
uint32 drive_enable_step[] = {1};                   // 行进轮速度环开关步进值（0/1切换）
float drive_open_loop_step[] = {10.0f, 100.0f, 500.0f}; // 行进轮开环输出步进

CustomData drive_speed_pid_data[] = {
    {&drive_speed_enable, data_uint32_show, "Enable(0/1)", drive_enable_step, 1, 0, 1, 0},
    {&target_drive_speed, data_float_show, "Target Speed", target_speed_step, 3, 0, 5, 1},
    {&drive_open_loop_output, data_float_show, "Open Loop Out", drive_open_loop_step, 3, 0, 6, 1},
    {&drive_speed_pid.kp, data_float_show, "Kp", pid_kp_step, 5, 0, 4, 3},
    {&drive_speed_pid.ki, data_float_show, "Ki", pid_ki_step, 5, 0, 4, 3},
    {&drive_speed_pid.kd, data_float_show, "Kd", pid_kd_step, 5, 0, 4, 4},
    {&drive_speed_pid.max_integral, data_float_show, "Max Integral", pid_limit_step, 3, 0, 5, 1},
    {&drive_speed_pid.max_output, data_float_show, "Max Output", pid_limit_step, 3, 0, 5, 1},
};

Page page_drive_speed_pid = {
    .name = "Drive Speed PID",
    .data = drive_speed_pid_data,
    .len = 8,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.5 输出平滑参数
float filter_coeff_step[] = {0.01f, 0.05f, 0.1f};

CustomData output_smooth_data[] = {
    {&output_filter_coeff, data_float_show, "Filter Coeff", filter_coeff_step, 3, 0, 1, 2},
};

Page page_output_smooth = {
    .name = "Output Smooth",
    .data = output_smooth_data,
    .len = 1,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.6 电机保护参数
uint32 protect_enable_step[] = {1};              // 开关步进值 (0/1切换)
float angle_protect_step[] = {0.5f, 1.0f, 5.0f}; // 角度保护阈值步进

CustomData motor_protect_data[] = {
    {&stall_protect_enable, data_uint32_show, "Stall Protect", protect_enable_step, 1, 0, 1, 0},
    {&angle_protect_enable, data_uint32_show, "Angle Protect", protect_enable_step, 1, 0, 1, 0},
    {&angle_protection, data_float_show, "Angle Threshold", angle_protect_step, 3, 0, 4, 1},
};

Page page_motor_protect = {
    .name = "Motor Protection",
    .data = motor_protect_data,
    .len = 3,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 3.7 PID主菜单
Page page_pid = {
    .name = "PID Params",
    .data = NULL,
    .len = 8,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {&page_gyro_pid, &page_angle_pid, &page_speed_pid, &page_drive_speed_pid, &page_output_smooth, &page_motor_protect, &page_turn_comp, &page_steer_pid},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 4. 转弯补偿参数菜单 - Turn Compensation (动态零点补偿)
//============================================================
float comp_deadzone_step[] = {0.01f, 0.1f, 0.5f};       // 死区阈值步进（度）
float comp_gain_step[] = {0.01f, 0.1f, 0.5f, 1.0f};    // 动态补偿增益步进
float comp_k_error_step[] = {0.01f, 0.1f, 0.5f};       // 图像误差系数步进
float comp_max_step[] = {0.5f, 1.0f, 2.0f};             // 最大补偿限制步进
float servo_center_step[] = {0.1f, 1.0f, 5.0f};         // 舵机中点角度步进
float image_error_threshold_step[] = {0.5f, 1.0f, 5.0f}; // 图像误差阈值步进

CustomData turn_comp_data[] = {
    {&turn_comp_k_speed, data_float_show, "Gain", comp_gain_step, 4, 0, 3, 2},
    {&turn_comp_k_error, data_float_show, "Error Coeff", comp_k_error_step, 3, 0, 3, 2},
    {&turn_comp_k_servo, data_float_show, "Deadzone", comp_deadzone_step, 3, 0, 2, 2},
    {&turn_comp_image_threshold, data_float_show, "Img Err Thres", image_error_threshold_step, 3, 0, 3, 1},
    {&turn_comp_max, data_float_show, "Max Comp", comp_max_step, 3, 0, 3, 1},
    {&servo_center_angle, data_float_show, "Servo Center", servo_center_step, 3, 0, 3, 1},
};

Page page_turn_comp = {
    .name = "Turn Compensation",
    .data = turn_comp_data,
    .len = 6,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 4.2 转向PID参数菜单 - Steering PID (Image Error P + Gyro Gz D)
//============================================================
uint32 steer_enable_step[] = {1};                   // 转向环开关步进值（0/1切换）
float steer_kp_step[] = {0.01f, 0.1f, 1.0f, 5.0f};
float steer_kd_step[] = {0.001f, 0.01f, 0.1f};
float steer_limit_step[] = {1.0f, 5.0f, 10.0f};
uint32 sample_row_step[] = {1, 5, 10};

CustomData steer_pid_data[] = {
    {&steer_enable, data_uint32_show, "Enable (0/1)", steer_enable_step, 1, 0, 1, 0},
    {&steer_kp, data_float_show, "Kp (Image)", steer_kp_step, 4, 0, 3, 2},
    {&steer_kd, data_float_show, "Kd (Gyro Gz)", steer_kd_step, 3, 0, 3, 3},
    {&steer_output_limit, data_float_show, "Output Limit", steer_limit_step, 3, 0, 4, 1},
    {&steer_sample_start, data_uint32_show, "Sample Start", sample_row_step, 3, 0, 3, 0},
    {&steer_sample_end, data_uint32_show, "Sample End", sample_row_step, 3, 0, 3, 0},
};

Page page_steer_pid = {
    .name = "Steer PID",
    .data = steer_pid_data,
    .len = 6,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 5. IMU菜单
//============================================================
// 4.1 IMU参数
int16 gyro_offset_step[] = {1, 10, 100};

CustomData imu_data_params[] = {
    {&gyro_x_offset, data_int16_show, "Gyro X Offset", gyro_offset_step, 3, 0, 5, 0},
    {&gyro_y_offset, data_int16_show, "Gyro Y Offset", gyro_offset_step, 3, 0, 5, 0},
    {&gyro_z_offset, data_int16_show, "Gyro Z Offset", gyro_offset_step, 3, 0, 5, 0},
};

Page page_imu_params = {
    .name = "Gyro Offsets",
    .data = imu_data_params,
    .len = 3,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

// 4.2 陀螺仪校准功能
void gyro_calibration_wrapper(void)
{
    ips_clear();
    show_string(0, 1, "Gyro Calibration");
    show_string(0, 4, "Place device");
    show_string(0, 6, "stable...");
    show_string(0, 9, "Starting in 2s");

    buzzer_beep(1, 50, 100);

    if (!imu_data.is_initialized)
    {
        ips_clear();
        show_string(0, 4, "IMU Not Init!");
        show_string(0, 7, "Press BACK");
        return;
    }

    ips_clear();
    show_string(0, 4, "Calibrating...");
    show_string(0, 7, "Please wait");

    imu_calibrate_gyro(1000);
    Param_Save_All();

    buzzer_beep(1, 200, 100);

    ips_clear();
    show_string(0, 1, "Calibration Done!");
    show_string(0, 4, "X Offset:");
    show_int(8, 4, gyro_x_offset, 5);
    show_string(0, 6, "Y Offset:");
    show_int(8, 6, gyro_y_offset, 5);
    show_string(0, 8, "Z Offset:");
    show_int(8, 8, gyro_z_offset, 5);
    show_string(0, 11, "Press BACK");
}

Page page_gyro_calibration = {
    .name = "Calibrate",
    .data = NULL,
    .len = 0,
    .stage = Funtion,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {.function = gyro_calibration_wrapper},
    .order = 0,
    .scroll_offset = 0,
};

// 4.3 IMU主菜单
Page page_imu = {
    .name = "IMU",
    .data = NULL,
    .len = 2,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {&page_imu_params, &page_gyro_calibration},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 5. 延迟停车参数菜单 - Delayed Stop Parameters
//============================================================
uint32 delay_step[] = {100, 1000, 5000, 10000}; // 0.1s, 1s, 5s, 10s
uint32 enabled_step[] = {1};                    // 开关步进值

CustomData delayed_stop_data[] = {
    {&delayed_stop_enabled, data_uint32_show, "Enabled(0/1)", enabled_step, 1, 0, 1, 0},
    {&delayed_stop_time_ms, data_uint32_show, "Delay Time(ms)", delay_step, 4, 0, 6, 0},
};

Page page_delayed_stop = {
    .name = "Delayed Stop",
    .data = delayed_stop_data,
    .len = 2,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 6. Cargo运行模式
//============================================================
void cargo_run_mode(void)
{
    motor_reset_protection();
    enable = true;

    ips_clear();
    show_string(0, 6, "Running...");
    show_string(0, 9, "Press BACK to exit");

    // 启动延迟停车（如果启用了延迟停车功能）
    delayed_stop_start_with_param();
}

Page page_cargo = {
    .name = "Cargo",
    .data = NULL,
    .len = 0,
    .stage = Funtion,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {.function = cargo_run_mode},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 7. 调试监控页面
//============================================================
void debug_monitor_mode(void)
{
    ips_clear();
    while (1)
    {
        show_string(0, 0, "Debug Monitor");

        show_string(0, 3, "Enc0:");
        show_int(6, 3, encoder[0], 5);

        show_string(0, 5, "Enc1:");
        show_int(6, 5, encoder[1], 5);

        show_string(0, 7, "GyroY:");
        show_int(7, 7, imu_data.gyro_y, 6);

        show_string(0, 9, "Pitch:");
        show_float(7, 9, imu_data.pitch, 3, 2);

        show_string(0, 12, "Press BACK");

        if (Key_Scan() == KEY_BACK)
        {
            break;
        }
    }
}

Page page_debug = {
    .name = "Debug Monitor",
    .data = NULL,
    .len = 0,
    .stage = Funtion,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {.function = debug_monitor_mode},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 8. 摄像头图像显示
//============================================================
void camera_display_mode(void)
{
    uint8 display_mode = 0;  // 显示模式：0=灰度图，1=二值化图（使用大津法阈值）
    uint8 key = KEY_NONE;

    ips_clear();

    while (1)
    {
        uint8 image_threshold = 0;

        // 如果是二值化模式，使用大津法动态计算阈值
        if (display_mode == 1)
        {
            image_threshold = (uint8)otsu_get_threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
            threshold = (int)image_threshold;  // 显式转换避免警告
        }

        // 每次循环都刷新图像（摄像头是实时采集的）
        ips114_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, image_threshold);
        image_process();

    

        // 绘制赛道线条（彩色叠加）
        // 从底部往上绘制左右边界和中线
        for (uint16 row = 0; row < MT9V03X_H - 1; row++)
        {
            // 绘制左边界（红色）
            if (Left_Line[row] >= 0 && Left_Line[row] < MT9V03X_W &&
                Left_Line[row + 1] >= 0 && Left_Line[row + 1] < MT9V03X_W)
            {
                ips114_draw_line(Left_Line[row], row, Left_Line[row + 1], row + 1, RGB565_RED);
            }

            // 绘制右边界（蓝色）
            if (Right_Line[row] >= 0 && Right_Line[row] < MT9V03X_W &&
                Right_Line[row + 1] >= 0 && Right_Line[row + 1] < MT9V03X_W)
            {
                ips114_draw_line(Right_Line[row], row, Right_Line[row + 1], row + 1, RGB565_BLUE);
            }

            // 计算中线并绘制（绿色）
            if (Left_Line[row] >= 0 && Right_Line[row] >= 0)
            {
                int16 center_x = (Left_Line[row] + Right_Line[row]) / 2;
                if (row < MT9V03X_H - 1 && Left_Line[row + 1] >= 0 && Right_Line[row + 1] >= 0)
                {
                    int16 center_x_next = (Left_Line[row + 1] + Right_Line[row + 1]) / 2;
                    ips114_draw_line(center_x, row, center_x_next, row + 1, RGB565_GREEN);
                }
            }
        }

        // 绘制转向PID采样范围的上下线（黄色）
        if (steer_sample_start < MT9V03X_H && steer_sample_end < MT9V03X_H)
        {
            // 采样起始行（下线）
            ips114_draw_line(0, (uint16)steer_sample_start, MT9V03X_W - 1, (uint16)steer_sample_start, RGB565_YELLOW);
            // 采样结束行（上线）
            ips114_draw_line(0, (uint16)steer_sample_end, MT9V03X_W - 1, (uint16)steer_sample_end, RGB565_YELLOW);
        }
        
    

        // 在屏幕右上角显示当前阈值（二值化模式下显示大津法计算的阈值）
        if (display_mode == 1)
        {
            show_string(18, 0, "T:");
            show_int(20, 0, threshold, 3);
        }

        // 扫描按键
        key = Key_Scan();

        // 检测OK键 - 切换显示模式
        if (key == KEY_OK)
        {
            display_mode = !display_mode;  // 在0和1之间切换
            system_delay_ms(200);          // 防止按键连续触发
        }
        // 检测返回键 - 退出
        else if (key == KEY_BACK)
        {
            ips_clear();  // 退出前清空屏幕，避免和菜单渲染冲突
            break;
        }

        system_delay_ms(50); // 适当延迟，避免刷新过快
    }
}

Page page_camera = {
    .name = "Camera View",
    .data = NULL,
    .len = 0,
    .stage = Funtion,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {NULL},
    .content = {.function = camera_display_mode},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 9. 主菜单
//============================================================
Page main_page = {
    .name = "Main Menu",
    .data = NULL,
    .len = 7, // 子菜单数量（增加了Camera View）
    .stage = Menu,
    .back = NULL,
    .enter = {&page_cargo, &page_delayed_stop, &page_servo, &page_pid, &page_imu, &page_debug, &page_camera},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 用户菜单配置初始化函数
//============================================================
/**
 * @brief 用户菜单配置初始化
 * @note 在这里设置所有页面之间的父子关系
 */
void Menu_Config_Init(void)
{
    // 设置主菜单的子页面的父指针
    page_cargo.back = &main_page;
    page_delayed_stop.back = &main_page;
    page_servo.back = &main_page;
    page_pid.back = &main_page;
    page_imu.back = &main_page;
    page_debug.back = &main_page;
    page_camera.back = &main_page; // 摄像头显示页面

    // 设置PID子页面的父指针
    page_gyro_pid.back = &page_pid;
    page_angle_pid.back = &page_pid;
    page_speed_pid.back = &page_pid;
    page_drive_speed_pid.back = &page_pid;
    page_output_smooth.back = &page_pid;
    page_motor_protect.back = &page_pid;
    page_turn_comp.back = &page_pid;
    page_steer_pid.back = &page_pid;  // 转向PID参数页面

    // 设置IMU子页面的父指针
    page_imu_params.back = &page_imu;
    page_gyro_calibration.back = &page_imu;
}
