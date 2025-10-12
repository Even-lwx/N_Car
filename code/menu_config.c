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
#include "pid.h"
#include "imu.h"
#include "motor.h"
#include "servo.h"
#include "delayed_stop.h"
#include "turn_compensation.h"

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

/**************** 外部变量引用 ****************/
extern volatile bool enable; // volatile 关键字
extern int16 encoder[2];     // 类型为 int16 (不是 int32)

/**************** 页面前置声明 ****************/
// 在页面相互引用前需要前置声明
extern Page page_turn_comp;     // 转弯补偿参数页面
extern Page page_motor_protect; // 电机保护参数页面

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

CustomData drive_speed_pid_data[] = {
    {&target_drive_speed, data_float_show, "Target Speed", target_speed_step, 3, 0, 5, 1},
    {&drive_speed_pid.kp, data_float_show, "Kp", pid_kp_step, 5, 0, 4, 3},
    {&drive_speed_pid.ki, data_float_show, "Ki", pid_ki_step, 5, 0, 4, 3},
    {&drive_speed_pid.kd, data_float_show, "Kd", pid_kd_step, 5, 0, 4, 4},
    {&drive_speed_pid.max_integral, data_float_show, "Max Integral", pid_limit_step, 3, 0, 5, 1},
    {&drive_speed_pid.max_output, data_float_show, "Max Output", pid_limit_step, 3, 0, 5, 1},
};

Page page_drive_speed_pid = {
    .name = "Drive Speed PID",
    .data = drive_speed_pid_data,
    .len = 6,
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
    .len = 7,
    .stage = Menu,
    .back = NULL, // 在 Menu_Config_Init() 中设置
    .enter = {&page_gyro_pid, &page_angle_pid, &page_speed_pid, &page_drive_speed_pid, &page_output_smooth, &page_motor_protect, &page_turn_comp},
    .content = {NULL},
    .order = 0,
    .scroll_offset = 0,
};

//============================================================
// 4. 转弯补偿参数菜单 - Turn Compensation
//============================================================
float comp_k_angle_step[] = {0.0001f, 0.001f, 0.01f, 0.1f};
float comp_k_speed_step[] = {0.01f, 0.1f, 1.0f};
float comp_max_step[] = {0.5f, 1.0f, 2.0f};
float servo_center_step[] = {0.1f, 1.0f, 5.0f};

CustomData turn_comp_data[] = {
    {&turn_comp_k_servo, data_float_show, "K_Servo", comp_k_angle_step, 4, 0, 2, 4},
    {&turn_comp_k_speed, data_float_show, "K_Speed", comp_k_speed_step, 3, 0, 2, 4},
    {&turn_comp_max, data_float_show, "Max Comp", comp_max_step, 3, 0, 3, 1},
    {&servo_center_angle, data_float_show, "Servo Center", servo_center_step, 3, 0, 3, 1},
};

Page page_turn_comp = {
    .name = "Turn Compensation",
    .data = turn_comp_data,
    .len = 4,
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
    uint8 display_mode = 0;  // 显示模式：0=灰度图，1=二值化图
    uint8 threshold = 128;    // 二值化阈值（默认128）
    uint8 key = KEY_NONE;

    ips_clear();

    while (1)
    {
        // 显示标题（根据模式显示不同提示）
        if (display_mode == 0)
        {
            show_string(0, 0, "Gray [OK:Binary]");
        }
        else
        {
            show_string(0, 0, "Binary [OK:Gray]");
        }

        // 显示摄像头图像
        // 参数说明：
        // (0, 8) - 显示起始位置（像素坐标，从第8像素开始，留出标题8像素）
        // mt9v03x_image[0] - 图像数据指针
        // MT9V03X_W, MT9V03X_H - 原始图像尺寸 (188x120)
        // MT9V03X_W, MT9V03X_H - 显示图像尺寸（1:1显示，不压缩）
        // threshold - 阈值（0=灰度图，非0=二值化图）
        //
        // 布局计算：
        // - 屏幕：240宽 × 135高
        // - 标题：8像素（字符行0）
        // - 图像：188宽 × 120高（1:1显示）
        // - 总高度：8 + 120 = 128像素 < 135像素 ✓
        uint8 image_threshold = (display_mode == 0) ? 0 : threshold;
        ips114_show_gray_image(0, 8, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, image_threshold);

        // 扫描按键
        key = Key_Scan();

        // 检测OK键 - 切换显示模式
        if (key == KEY_OK)
        {
            display_mode = !display_mode;  // 在0和1之间切换
            system_delay_ms(200);  // 防止按键连续触发
        }
        // 检测返回键 - 退出
        else if (key == KEY_BACK)
        {
            break;
        }

        system_delay_ms(50); // 约20fps刷新率
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

    // 设置IMU子页面的父指针
    page_imu_params.back = &page_imu;
    page_gyro_calibration.back = &page_imu;
}
